<#
.SYNOPSIS
  Read-only troubleshooter for Pico-ASHA audio break-up on Windows.

.DESCRIPTION
  Part A checks the PC (USB path, power settings, CPU/DPC load, wireless adapters).
  Part B talks to the Pico over USB serial, asks it for its status (the same
  "intro" request the GUI and web app send) and records the events it reports
  while you play audio: stream stop/start cycles, disconnects with reason codes,
  errors. Press M whenever you HEAR a stutter to drop a marker, and the report
  shows what the Pico reported around each marker.

  It changes NO settings and sends only the status request. Close the Pico-ASHA
  GUI and the web control app first: only one program can hold the serial port.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File Pico-ASHA-Troubleshoot.ps1 -DurationSeconds 120
#>
param(
    [int]$DurationSeconds = 120,
    [string]$Port = '',
    [string]$ReportDir = (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Pico-ASHA-reports'),
    [switch]$SkipPico,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'

# ----------------------------------------------------------------- protocol
# Mirrors website/docs/control-app/protocol/*.js in the Pico-ASHA repository.
$EventNames = @('ShortLog','PicoASHAInit','DeletePair','RemoteConnected','RemoteDisconnected',
    'DiscoverServices','PairAndBond','DataLengthExtension','DiscoverASHACharacteristics','ROPRead',
    'PSMRead','DiscoverGAPCharacteristics','DeviceNameRead','DiscoverDISCharacteristics',
    'ManufacturerRead','ModelRead','FirmwareRead','SoftwareRead','L2CAPConnected','L2CAPDisconnected',
    'ASPNotificationsEnabled','ACPStart','ACPStop','ACPStatus','ASPStart','ASPStop','ASPError',
    'StreamReady','StreamPause','AudioVolume','DiscoverMFICharacteristics','MFIBatteryRead',
    'MFIBatteryNotificationsEnabled','G722EncodeTimings')
$HciReasons = @{
    0x08 = 'Connection timeout (link lost: range, interference or the aid stopped answering)'
    0x13 = 'Remote user terminated (the hearing aid ended the link)'
    0x15 = 'Remote device terminated, power off'
    0x16 = 'Local host terminated (the Pico ended the link on purpose)'
    0x3D = 'MIC failure (security/encryption mismatch)'
    0x3E = 'Connection failed to be established'
    0x22 = 'LMP/LL response timeout'
    0x28 = 'Instant passed'
}

function Cobs-Encode([byte[]]$src) {
    $out = New-Object System.Collections.Generic.List[byte]
    $out.Add(0); $codeIdx = 0; $code = 1
    foreach ($b in $src) {
        if ($b -eq 0) {
            $out[$codeIdx] = [byte]$code; $codeIdx = $out.Count; $out.Add(0); $code = 1
        } else {
            $out.Add($b); $code++
            if ($code -eq 255) { $out[$codeIdx] = [byte]255; $codeIdx = $out.Count; $out.Add(0); $code = 1 }
        }
    }
    $out[$codeIdx] = [byte]$code
    return ,$out.ToArray()
}

function Cobs-Decode([byte[]]$src) {
    $out = New-Object System.Collections.Generic.List[byte]
    $i = 0
    while ($i -lt $src.Length) {
        $code = [int]$src[$i]
        if ($code -eq 0) { throw 'unexpected delimiter' }
        $i++
        $end = $i + $code - 1
        if ($end -gt $src.Length) { throw 'code runs past frame' }
        while ($i -lt $end) { $out.Add($src[$i]); $i++ }
        if ($code -ne 255 -and $i -lt $src.Length) { $out.Add(0) }
    }
    return ,$out.ToArray()
}

function Get-StatusRequestFrame {
    # 20-byte command packet: header(8) + cmd(1) + status(1) + data(10). Command 5 = IntroPacket (status request).
    $pkt = New-Object byte[] 20
    $pkt[0] = 4; $pkt[1] = 20; $pkt[8] = 5; $pkt[9] = 0
    $enc = Cobs-Encode $pkt
    return ,([byte[]](@(0) + $enc + @(0)))
}

function Read-CString([byte[]]$b, [int]$off, [int]$len) {
    $s = [System.Text.Encoding]::ASCII.GetString($b, $off, $len)
    $z = $s.IndexOf([char]0); if ($z -ge 0) { $s = $s.Substring(0, $z) }
    return $s
}

function Decode-Packet([byte[]]$b) {
    if ($b.Length -lt 8) { return $null }
    $type = $b[0]; $len = $b[1]
    if ($len -ne $b.Length) { return $null }
    $hdr = @{ Type = $type; ConnId = [BitConverter]::ToUInt16($b, 2); PicoMs = [BitConverter]::ToUInt32($b, 4) }
    switch ($type) {
        0 { if ($b.Length -ne 16) { return $null }
            $flags = [BitConverter]::ToUInt16($b, 12)
            return [pscustomobject]@{ Kind='intro'; Hdr=$hdr; Version="$($b[8]).$($b[9]).$($b[10])"; Connected=[int][sbyte]$b[11];
                ConnAllowed=[bool]($flags -band 1); Streaming=[bool]($flags -band 2); Uac=$(if ($flags -band 4) {2} else {1}) } }
        1 { if ($b.Length -ne 192) { return $null }
            $p = 8
            return [pscustomobject]@{ Kind='remote'; Hdr=$hdr; ConnId=[BitConverter]::ToUInt16($b,$p);
                Address=(($b[($p+4)..($p+9)] | ForEach-Object { '{0:x2}' -f $_ }) -join ':');
                Connected=[bool]$b[$p+10]; Bonded=[bool]$b[$p+11]; Model=(Read-CString $b ($p+80) 32);
                Side=@('Left','Right','Unknown')[[math]::Min([int]$b[$p+176],2)]; Mode=@('Mono','Binaural','Unknown')[[math]::Min([int]$b[$p+177],2)];
                Streaming=[bool]$b[$p+178]; Volume=[int][sbyte]$b[$p+179]; Battery=[int]$b[$p+180] } }
        2 { if ($b.Length -ne 44) { return $null }
            $p = 8; $et = [int]$b[$p]
            $name = if ($et -lt $EventNames.Count) { $EventNames[$et] } else { "Event$et" }
            $text = if ($et -eq 0) { Read-CString $b ($p+4) 32 } else { '' }
            return [pscustomobject]@{ Kind='event'; PicoMs=$hdr.PicoMs; ConnId=$hdr.ConnId; Type=$et; Name=$name;
                StatusType=[int]$b[$p+1]; Status=[int]$b[$p+2]; Reason=[int]$b[$p+3]; Text=$text } }
        6 { if ($b.Length -ne 16) { return $null }
            return [pscustomobject]@{ Kind='usb'; Hdr=$hdr; Uac=[BitConverter]::ToUInt16($b,8) } }
        default { return [pscustomobject]@{ Kind="other$type"; Hdr=$hdr } }
    }
}

# ----------------------------------------------------------------- self test
if ($SelfTest) {
    $fail = 0
    foreach ($case in @(@(), @(0), @(1,2,3), @(0,0,0), @(1,0,2,0,3), (1..300 | ForEach-Object { $_ % 256 }))) {
        $src = [byte[]]$case
        $rt = Cobs-Decode (Cobs-Encode $src)
        if ((@($rt) -join ',') -ne (@($src) -join ',')) { Write-Host "COBS round trip FAILED for length $($src.Length)"; $fail++ }
    }
    $frame = Get-StatusRequestFrame
    if ($frame[0] -ne 0 -or $frame[$frame.Length-1] -ne 0) { Write-Host 'Request framing FAILED'; $fail++ }
    $inner = Cobs-Decode ([byte[]]$frame[1..($frame.Length-2)])
    if ($inner.Length -ne 20 -or $inner[0] -ne 4 -or $inner[8] -ne 5) { Write-Host 'Request contents FAILED'; $fail++ }
    $ev = New-Object byte[] 44; $ev[0]=2; $ev[1]=44; $ev[8]=4; $ev[9]=3; $ev[10]=0; $ev[11]=8
    $d = Decode-Packet $ev
    if ($d.Name -ne 'RemoteDisconnected' -or $d.Reason -ne 8) { Write-Host 'Event decode FAILED'; $fail++ }
    if ($fail -eq 0) { Write-Host 'Self test passed.' } else { Write-Host "Self test: $fail failure(s)"; exit 1 }
    return
}

# ----------------------------------------------------------------- report helpers
$report = New-Object System.Collections.Generic.List[string]
function Say([string]$s = '') { Write-Host $s; $report.Add($s) }
function Section([string]$s) { Say ''; Say ('=' * 70); Say $s; Say ('=' * 70) }
$hints = New-Object System.Collections.Generic.List[string]
function Hint([string]$s) { $hints.Add($s) }

Section "Pico-ASHA troubleshooter  -  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"

# ----------------------------------------------------------------- Part A
Section 'PART A - this PC'
$os = Get-CimInstance Win32_OperatingSystem
Say "Windows: $($os.Caption) build $($os.BuildNumber)"
$cs = Get-CimInstance Win32_ComputerSystem
Say "Hypervisor present (Hyper-V etc.): $($cs.HypervisorPresent)"
Say ("Power plan: " + ((powercfg /getactivescheme) -replace '^.*:\s*',''))
$ss = powercfg /q SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 2>$null
$ac = ($ss | Select-String 'Current AC Power Setting Index:\s*0x(\w+)').Matches | Select-Object -First 1
if ($ac) {
    $acv = [Convert]::ToInt32($ac.Groups[1].Value, 16)
    Say "USB selective suspend (plugged in): $(if ($acv -eq 1) {'ENABLED'} else {'disabled'})"
    if ($acv -eq 1) { Hint 'USB selective suspend is enabled. Turning it off (Power Options > Advanced > USB settings) can help USB audio.' }
}

$wifi = Get-NetAdapter -Physical -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'Up' }
Say ('Active physical network adapters: ' + $(if ($wifi) { ($wifi | ForEach-Object { "$($_.Name) [$($_.InterfaceDescription)]" }) -join '; ' } else { 'none' }))
$wl = $wifi | Where-Object { $_.InterfaceDescription -match 'Wi-?Fi|Wireless|802\.11' }
if ($wl) { Hint 'This PC is on Wi-Fi. Wi-Fi on 2.4 GHz can disturb the Pico-ASHA Bluetooth link; try 5 GHz or Ethernet.' }
$bt = Get-PnpDevice -Class Bluetooth -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.FriendlyName -match 'Radio|Adapter|Wireless Bluetooth|Intel|Realtek|Qualcomm|MediaTek' }
Say ('Bluetooth radios in this PC: ' + $(if ($bt) { ($bt.FriendlyName -join '; ') } else { 'none found' }))

$picoCom = Get-PnpDevice -Class Ports -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -match 'VID_CAFE' } | Select-Object -First 1
$comName = ''
if ($picoCom) {
    if ($picoCom.FriendlyName -match '\((COM\d+)\)') { $comName = $Matches[1] }
    Say "Pico-ASHA serial port: $comName  ($($picoCom.InstanceId))"
    $chain = @(); $id = $picoCom.InstanceId
    for ($i = 0; $i -lt 8 -and $id; $i++) {
        $dev = Get-PnpDevice -InstanceId $id -ErrorAction SilentlyContinue
        if ($dev) { $chain += $(if ($dev.FriendlyName) { $dev.FriendlyName } else { $dev.Class }) }
        $id = (Get-PnpDeviceProperty -InstanceId $id -KeyName 'DEVPKEY_Device_Parent' -ErrorAction SilentlyContinue).Data
    }
    Say ('USB path (Pico up to the PC): ' + ($chain -join '  ->  '))
    if ($chain -match 'Generic USB Hub|USB Hub') {
        if (($chain | Where-Object { $_ -match 'Hub' -and $_ -notmatch 'Root' }).Count -gt 0) { Hint 'The Pico is behind a USB hub. Plug it straight into a rear port of the PC and compare.' }
    }
    if ($chain -match 'USB 3|eXtensible|xHCI') { Hint 'The Pico is on a USB 3 (xHCI) controller. USB 3 ports and cables can radiate noise near 2.4 GHz; try a USB 2 port or a short extension cable to move the Pico away from the PC.' }
} else {
    Say 'Pico-ASHA serial port: not found (is the Pico plugged in and running Pico-ASHA firmware?)'
}

# ----------------------------------------------------------------- Part B
$events = New-Object System.Collections.Generic.List[object]
$markers = New-Object System.Collections.Generic.List[object]
$remotes = @{}
$cpuSamples = New-Object System.Collections.Generic.List[object]
$intro = $null
$usbInfo = $null

if (-not $SkipPico) {
    Section "PART B - the Pico's own report ($DurationSeconds s)"
    if (-not $Port) { $Port = $comName }
    if (-not $Port) { $names = [System.IO.Ports.SerialPort]::GetPortNames(); if ($names.Count -eq 1) { $Port = $names[0] } }
    if (-not $Port) {
        Say 'No Pico serial port found, so Part B is skipped.'
    } else {
        $sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
        $sp.ReadTimeout = 200; $sp.WriteTimeout = 1000
        $opened = $false
        try { $sp.Open(); $sp.DtrEnable = $true; $opened = $true }
        catch {
            Say "Could not open $Port : $($_.Exception.Message)"
            Say 'Another program has the port. Close the Pico-ASHA GUI and the Chrome control-app tab (End task on Chrome in Task Manager if needed), unplug and replug the Pico, then run this again.'
            $holders = Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -match 'PicoASHA|chrome|msedge' } | Group-Object ProcessName | ForEach-Object { "$($_.Name) x$($_.Count)" }
            if ($holders) { Say ('Programs running that could hold it: ' + ($holders -join ', ')) }
            Hint "Part B could not run because $Port is held by another program."
        }
        if ($opened) {
            try {
                $sp.DiscardInBuffer()
                $req = Get-StatusRequestFrame
                $sp.Write($req, 0, $req.Length)
                Say "Connected to $Port. Play audio now. Nothing else to do: dropouts are detected automatically. Recording up to $DurationSeconds s; press Q any time to stop early and get the report (M is optional, to mark a stutter you hear)."
                $buf = New-Object System.Collections.Generic.List[byte]
                $sw = [Diagnostics.Stopwatch]::StartNew()
                $nextCpu = 0
                $rx = New-Object byte[] 4096
                while ($sw.Elapsed.TotalSeconds -lt $DurationSeconds) {
                    $keyReady = $false
                    try { $keyReady = [Console]::KeyAvailable } catch { $keyReady = $false }  # no console when run from a tool
                    if ($keyReady) {
                        $k = [Console]::ReadKey($true)
                        if ($k.Key -eq 'Q') { Write-Host '  Q pressed: stopping early and building the report...'; break }
                        if ($k.Key -eq 'M') { $markers.Add([pscustomobject]@{ HostMs = $sw.ElapsedMilliseconds }); Write-Host ("  marker at {0:n1}s" -f $sw.Elapsed.TotalSeconds) }
                    }
                    if ($sw.Elapsed.TotalSeconds -ge $nextCpu) {
                        $nextCpu += 10
                        try {
                            $c = Get-Counter -Counter '\Processor Information(_Total)\% Processor Time','\Processor Information(_Total)\% DPC Time','\Processor Information(_Total)\% Interrupt Time' -ErrorAction Stop
                            $v = $c.CounterSamples | ForEach-Object { $_.CookedValue }
                            $cpuSamples.Add([pscustomobject]@{ Cpu=$v[0]; Dpc=$v[1]; Int=$v[2] })
                        } catch {}
                    }
                    $n = 0
                    try { $n = $sp.BytesToRead } catch { Say "Serial port error: $($_.Exception.Message)"; break }
                    if ($n -le 0) { Start-Sleep -Milliseconds 15; continue }
                    $got = $sp.Read($rx, 0, [Math]::Min($n, $rx.Length))
                    for ($i = 0; $i -lt $got; $i++) {
                        $byte = $rx[$i]
                        if ($byte -eq 0) {
                            if ($buf.Count -gt 0) {
                                try {
                                    $dec = Cobs-Decode ($buf.ToArray())
                                    $pk = Decode-Packet $dec
                                    if ($pk) {
                                        switch ($pk.Kind) {
                                            'intro'  { $intro = $pk }
                                            'usb'    { $usbInfo = $pk }
                                            'remote' { $remotes[$pk.ConnId] = $pk }
                                            'event'  { $pk | Add-Member -NotePropertyName HostMs -NotePropertyValue $sw.ElapsedMilliseconds; $events.Add($pk) }
                                        }
                                    }
                                } catch {}
                                $buf.Clear()
                            }
                        } else { if ($buf.Count -lt 600) { $buf.Add($byte) } }
                    }
                }
                Say 'Recording finished.'
            } finally {
                if ($sp.IsOpen) { $sp.Close() }
            }
        }
    }
}

# ----------------------------------------------------------------- analysis
Section 'RESULTS'
if ($intro) {
    # The USB audio class is reported in the separate USB-info packet; the intro flag is not set by newer firmware.
    $uacText = if ($usbInfo) { "USB audio class $($usbInfo.Uac)" } else { 'USB audio class not reported' }
    Say "Pico-ASHA firmware $($intro.Version), $uacText, $($intro.Connected) hearing aid(s) connected, connections allowed: $($intro.ConnAllowed), streaming enabled: $($intro.Streaming)"
}
foreach ($r in $remotes.Values) {
    Say ("  {0} aid [{1}] {2}: connected={3}, bonded={4}, streaming={5}, battery {6}/10, mode {7}" -f $r.Side, $r.Address, $r.Model, $r.Connected, $r.Bonded, $r.Streaming, $r.Battery, $r.Mode)
}
if ($cpuSamples.Count -gt 0) {
    $maxDpc = ($cpuSamples | Measure-Object Dpc -Maximum).Maximum
    $avgCpu = ($cpuSamples | Measure-Object Cpu -Average).Average
    Say ("PC load during the run: CPU avg {0:n0}%, highest DPC time {1:n1}% (high DPC time can break up USB audio)" -f $avgCpu, $maxDpc)
    if ($maxDpc -gt 5) { Hint "DPC time reached $([math]::Round($maxDpc,1))%. A driver on this PC is delaying USB audio; LatencyMon (free) can name it." }
}

if ($events.Count -gt 0 -or $intro) {
    # The Pico stores recent events and hands them over when we connect. Anything older than its
    # answer to our request is HISTORY (before the run) and must not be counted as happening now.
    $refMs = if ($intro) { [int64]$intro.Hdr.PicoMs } else { 0 }
    foreach ($e in $events) {
        $e | Add-Member -NotePropertyName RelS -NotePropertyValue (([int64]$e.PicoMs - $refMs) / 1000.0) -Force
    }
    $history = @($events | Where-Object { $_.RelS -lt -0.05 })
    $live    = @($events | Where-Object { $_.RelS -ge -0.05 })
    $skipNames = 'AudioVolume','G722EncodeTimings'

    Say ''
    Say ("Stored events from BEFORE the run: {0} (shown separately, not counted as part of this run)" -f $history.Count)
    $hDisc = @($history | Where-Object { $_.Name -eq 'RemoteDisconnected' })
    foreach ($d in $hDisc) {
        $why = if ($HciReasons.ContainsKey($d.Reason)) { $HciReasons[$d.Reason] } else { 'unlisted reason' }
        Say ("  {0:n0}s before the run: a hearing aid disconnected, reason 0x{1:x2} - {2}" -f (-$d.RelS), $d.Reason, $why)
    }
    if (@($hDisc | Where-Object { $_.Reason -eq 8 }).Count -gt 0) { Hint ("Before this run the Pico recorded {0} link timeout(s) (reason 0x08). At least one hearing aid has lost the radio link recently. That points to range, body shadowing between the Pico and one ear, or interference." -f @($hDisc | Where-Object { $_.Reason -eq 8 }).Count) }

    Say ''
    Say "Events DURING the run: $($live.Count)"
    $live | Group-Object Name | Sort-Object Count -Descending | ForEach-Object { Say ("  {0,-26} {1}" -f $_.Name, $_.Count) }

    $mins = [Math]::Max($DurationSeconds / 60.0, 0.1)
    $stops = @($live | Where-Object { $_.Name -in 'ACPStop','ASPStop','StreamPause' }).Count
    $starts = @($live | Where-Object { $_.Name -in 'ACPStart','ASPStart' }).Count
    $disc = @($live | Where-Object { $_.Name -eq 'RemoteDisconnected' })
    $conn = @($live | Where-Object { $_.Name -eq 'RemoteConnected' })
    $l2 = @($live | Where-Object { $_.Name -eq 'L2CAPDisconnected' })
    $errs = @($live | Where-Object { $_.Name -eq 'ASPError' -or ($_.StatusType -ne 0 -and $_.Status -ne 0 -and $_.Name -ne 'RemoteDisconnected') })

    Say ''
    Say ("During the run: stream stop events {0} ({1:n1}/min), start events {2}, disconnects {3}, (re)connections {4}" -f $stops, ($stops / $mins), $starts, $disc.Count, $conn.Count)
    if ($intro -and $intro.Connected -lt 2) {
        Say ("NOTE: only {0} of 2 hearing aids were connected when the run began." -f $intro.Connected)
        Hint 'Only one hearing aid was connected when the run began, so you were not hearing both ears. Check which ear was silent.'
    }
    foreach ($c in $conn) { Say ("  a hearing aid (re)connected at {0:n0}s into the run" -f $c.RelS) }
    foreach ($d in $disc) {
        $why = if ($HciReasons.ContainsKey($d.Reason)) { $HciReasons[$d.Reason] } else { 'unlisted reason' }
        Say ("  disconnect at {0:n0}s: reason 0x{1:x2} - {2}" -f $d.RelS, $d.Reason, $why)
    }
    if ($disc.Count -gt 0) {
        if (@($disc | Where-Object { $_.Reason -eq 8 }).Count -gt 0) { Hint 'During the run a link timed out (reason 0x08). That points to radio trouble: distance, an obstruction, or interference.' }
        if (@($disc | Where-Object { $_.Reason -eq 0x13 }).Count -gt 0) { Hint 'During the run a hearing aid ended the link itself (0x13). Check the aids are not also connecting to a phone or another device.' }
    }
    if ($l2.Count -gt 0) { Say "L2CAP audio channel closed $($l2.Count) time(s) during the run." }
    if ($errs.Count -gt 0) { Say "Error events during the run: $($errs.Count)"; $errs | Select-Object -First 8 | ForEach-Object { Say ("  {0} status type {1} status {2}" -f $_.Name, $_.StatusType, $_.Status) } }
    $fell = @($live | Where-Object { $_.Name -eq 'ShortLog' -and $_.Text -match 'fell behind|underrun|overrun|Zero credits' })
    if ($fell.Count -gt 0) { Say "Pico warnings during the run: $($fell.Count)"; $fell | Group-Object Text | ForEach-Object { Say ("  {0} x{1}" -f $_.Name, $_.Count) } }

    # Link settings logged by the experimental firmware builds (connection, parameter update, PHY change)
    $linkLogs = @($events | Where-Object { $_.Name -eq 'ShortLog' -and $_.Text -match '^(link|upd|phy) ' })
    if ($linkLogs.Count -gt 0) {
        Say ''
        Say 'Bluetooth link settings reported by the Pico (link/upd = interval x1.25 ms / latency in events / timeout x10 ms; phy: 1 = 1M, 2 = 2M):'
        foreach ($l in ($linkLogs | Select-Object -Last 8)) { Say ("  {0:n0}s: {1}" -f $l.RelS, $l.Text) }
    }

    # AUTOMATIC DROPOUT DETECTION (firmware vE and later report one line per ear per second:
    # "L n50 g21 c7 l9" = SDUs delivered, longest gap ms, lowest credits, longest send ms)
    $statRows = New-Object System.Collections.Generic.List[object]
    foreach ($e in $live) {
        if ($e.Name -eq 'ShortLog' -and $e.Text -match '^([LR]) n(\d+) g(\d+) c(\d+) [lt](\d+)') {
            $statRows.Add([pscustomobject]@{ T = [double]$e.RelS; Ear = $Matches[1]; N = [int]$Matches[2]; Gap = [int]$Matches[3]; Cr = [int]$Matches[4]; Lat = [int]$Matches[5] })
        }
    }
    if ($statRows.Count -gt 0) {
        Section 'AUTOMATIC DROPOUT DETECTION (from the Pico, one reading per ear per second)'
        $csv = Join-Path $ReportDir ("pico-asha-stats-{0}.csv" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
        if (-not (Test-Path $ReportDir)) { New-Item -ItemType Directory -Force -Path $ReportDir | Out-Null }
        $statRows | Export-Csv -Path $csv -NoTypeInformation
        Say "Per-second readings saved to: $csv"
        # A second is flagged when an ear got fewer than 45 of its 50 audio packets, a gap of 40 ms or more
        # opened between packets, or the ear's credit count fell to 1 or less (it is about to run dry).
        # (Low credits alone are NOT counted: in testing, credits dipped to 0-1 while all 50 packets still
        # arrived on time. They are reported separately as near-misses.)
        function Test-Bad($r) { return ($r.N -lt 45 -or $r.Gap -ge 40) }
        $nearMiss = @($statRows | Where-Object { -not (Test-Bad $_) -and $_.Cr -le 1 }).Count
        if ($nearMiss -gt 0) { Say ("Near-misses (credits fell to 0-1 but every packet still arrived on time): {0} ear-seconds" -f $nearMiss) }
        foreach ($ear in 'L','R') {
            $rows = @($statRows | Where-Object { $_.Ear -eq $ear } | Sort-Object T)
            if ($rows.Count -eq 0) { continue }
            $bad = @($rows | Where-Object { Test-Bad $_ })
            $worstGap = ($rows | Measure-Object Gap -Maximum).Maximum
            $minCr = ($rows | Measure-Object Cr -Minimum).Minimum
            $maxLat = ($rows | Measure-Object Lat -Maximum).Maximum
            $avgN = [math]::Round(($rows | Measure-Object N -Average).Average, 1)
            Say ''
            Say ("{0} ear: {1} readings, average {2} packets/s (50 is perfect). Worst gap {3} ms, lowest credits {4}, longest Pico main-loop stall {5} ms (normally 1-3)." -f $ear, $rows.Count, $avgN, $worstGap, $minCr, $maxLat)
            Say ("  Seconds that likely sounded broken: {0} ({1:n1}%)" -f $bad.Count, (100.0 * $bad.Count / $rows.Count))
            # Group consecutive bad seconds into episodes
            $episodes = New-Object System.Collections.Generic.List[object]
            $cur = $null
            foreach ($r in $bad) {
                if ($cur -and ($r.T - $cur.End) -le 2.5) { $cur.End = $r.T; $cur.Secs++; $cur.WorstGap = [math]::Max($cur.WorstGap, $r.Gap); $cur.MinCr = [math]::Min($cur.MinCr, $r.Cr); $cur.MinN = [math]::Min($cur.MinN, $r.N) }
                else { if ($cur) { $episodes.Add($cur) }; $cur = [pscustomobject]@{ Start = $r.T; End = $r.T; Secs = 1; WorstGap = $r.Gap; MinCr = $r.Cr; MinN = $r.N } }
            }
            if ($cur) { $episodes.Add($cur) }
            Say ("  Dropout episodes: {0}" -f $episodes.Count)
            foreach ($ep in ($episodes | Select-Object -First 25)) {
                Say ("    {0}:{1:00} for {2}s  (worst gap {3} ms, lowest credits {4}, fewest packets {5}/s)" -f [int][math]::Floor($ep.Start / 60), [int]($ep.Start % 60), $ep.Secs, $ep.WorstGap, $ep.MinCr, $ep.MinN)
            }
            if ($episodes.Count -gt 25) { Say ("    ... and {0} more (see the CSV)" -f ($episodes.Count - 25)) }
            # Any stretch where the Pico stopped reporting this ear while the run continued
            $prev = $rows[0].T
            foreach ($r in ($rows | Select-Object -Skip 1)) {
                if (($r.T - $prev) -gt 3.0) { Say ("    {0} ear stream was stopped or missing from {1:n0}s to {2:n0}s" -f $ear, $prev, $r.T) }
                $prev = $r.T
            }
        }
        $badAll = @($statRows | Where-Object { Test-Bad $_ })
        $lowCr = @($badAll | Where-Object { $_.Cr -le 1 -or $_.N -lt 20 }).Count
        $gaps = @($badAll | Where-Object { $_.Gap -ge 40 }).Count
        if ($badAll.Count -eq 0) { Hint 'The Pico saw no dropouts during the run: every packet was delivered on time to both ears. If you still heard glitches, the problem is after the Pico (the hearing aids) or before it (the PC audio).' }
        else {
            $stalled = @($badAll | Where-Object { $_.Lat -ge 40 }).Count
            if ($stalled -gt 0) { Hint ("In {0} of the bad seconds the Pico's own main loop froze for 40 ms or more. That is the Pico (or its USB/flash timing) stalling, not the radio. Tell Claude: this changes the fix." -f $stalled) }
            elseif ($statRows.Count -gt 0 -and ($statRows | Where-Object { $_.Lat -ge 0 } | Measure-Object Lat -Maximum).Maximum -lt 40 -and (@($statRows | Where-Object { $_.Lat -gt 0 }).Count -gt 0)) { Hint 'The Pico main loop never stalled during the dropouts, so the Pico itself was running fine. That points to the radio (interference or the aids not answering).' }
            Hint ("The Pico itself saw {0} bad ear-seconds: {1} with the credits nearly empty and {2} with a gap of 40 ms or more between packets. These are the dropouts, and the list above says when each began." -f $badAll.Count, $lowCr, $gaps)
            if ($lowCr -gt 0) { Hint 'Credits running low means the hearing aid is not taking packets as fast as they are sent. That is the radio link losing packets and resending them (interference or distance), not a PC problem.' }
            $lOnly = @($badAll | Where-Object { $_.Ear -eq 'L' }).Count; $rOnly = @($badAll | Where-Object { $_.Ear -eq 'R' }).Count
            if ($lOnly -gt 0 -and $rOnly -gt 0) {
                $both = 0
                foreach ($r in ($badAll | Where-Object { $_.Ear -eq 'L' })) { if (@($badAll | Where-Object { $_.Ear -eq 'R' -and [math]::Abs($_.T - $r.T) -le 1.5 }).Count -gt 0) { $both++ } }
                if ($both -ge 0.6 * [math]::Min($lOnly, $rOnly)) { Hint 'Both ears went bad at the same moments. That points at something common to both: interference at the Pico, the Pico being blocked from both ears, or the USB audio feed. One ear alone would point at that ear.' }
                else { Hint 'The ears went bad at different moments. That points at one ear at a time (the ear worn on the side away from the Pico, or a single processor under interference).' }
            }
            elseif ($lOnly -gt 0 -or $rOnly -gt 0) { Hint ("Only the {0} ear had dropouts. The Pico is probably weaker to that ear (blocked by your head or body), so try moving the Pico." -f $(if ($lOnly -gt 0) { 'left' } else { 'right' })) }
        }
    } elseif ($live.Count -gt 0) {
        Say ''
        Say 'No per-second link readings came from the Pico (this firmware build does not send them). Flash the vE build to get automatic dropout detection.'
    }

    # OVER-THE-AIR ACKNOWLEDGEMENTS (firmware vI and later): "L a50 k21" = packets the aid acknowledged
    # this second, longest time between acknowledgements in ms.
    $ackRows = New-Object System.Collections.Generic.List[object]
    foreach ($e in $live) {
        if ($e.Name -eq 'ShortLog' -and $e.Text -match '^([LR]) a(\d+) k(\d+)') {
            $ackRows.Add([pscustomobject]@{ T = [double]$e.RelS; Ear = $Matches[1]; A = [int]$Matches[2]; K = [int]$Matches[3] })
        }
    }
    foreach ($e in $events) { if ($e.Name -eq 'ShortLog' -and $e.Text -match '^dle ') { Say ("Data length negotiated: {0}  (tx bytes/time x100us rx bytes/time x100us; the phone gets 167 bytes)" -f $e.Text) } }
    if ($ackRows.Count -gt 0) {
        Section 'OVER-THE-AIR ACKNOWLEDGEMENTS (when each hearing aid confirmed receiving packets)'
        Say 'Note: the Pico radio chip reports acknowledgements in batches, so gaps of 40-60 ms between reports are normal. Only gaps of 100 ms or more, or fewer than 45 per second, are flagged.'
        foreach ($ear in 'L','R') {
            $rr = @($ackRows | Where-Object { $_.Ear -eq $ear })
            if ($rr.Count -eq 0) { continue }
            $badA = @($rr | Where-Object { $_.K -ge 100 -or $_.A -lt 45 })
            $avgA = [math]::Round(($rr | Measure-Object A -Average).Average, 1)
            $worstK = ($rr | Measure-Object K -Maximum).Maximum
            Say ("{0} ear: {1} readings, average {2} acknowledged/s (50 is perfect), worst gap between acknowledgements {3} ms. Seconds with a missed or late over-the-air event: {4} ({5:n1}%)" -f $ear, $rr.Count, $avgA, $worstK, $badA.Count, (100.0 * $badA.Count / $rr.Count))
            foreach ($r in ($badA | Select-Object -First 15)) { Say ("    {0}:{1:00}  acknowledged {2}, longest gap {3} ms" -f [int][math]::Floor($r.T / 60), [int]($r.T % 60), $r.A, $r.K) }
        }
        $badAck = @($ackRows | Where-Object { $_.K -ge 100 -or $_.A -lt 45 })
        if ($badAck.Count -gt 0) { Hint ("The hearing aids acknowledged packets late or not at all in {0} ear-seconds. That means over-the-air radio events are being missed or retried even though the Pico handed packets over on time. That is the radio link (interference, the Pico's radio scheduling, or link settings), and the list above says when." -f $badAck.Count) }
        else { Hint 'Every packet was acknowledged by both aids on time for the whole run. The radio link kept up, so a stutter you still hear comes from inside the aids or from the audio data itself.' }
    }

    # USB AUDIO PATH (firmware vG and later): "usb u2 f1-3 d0" = silence insertions, fewest-most 1 ms frames
    # waiting in the buffer, worst lateness of the Pico's USB task in ms.
    $usbRows = New-Object System.Collections.Generic.List[object]
    foreach ($e in $live) {
        if ($e.Name -eq 'ShortLog' -and $e.Text -match '^usb u(\d+) f(\d+)-(\d+) d(\d+)(?: r(\d+))?(?: p(\d+))?') {
            $usbRows.Add([pscustomobject]@{ T = [double]$e.RelS; U = [int]$Matches[1]; Fmin = [int]$Matches[2]; Fmax = [int]$Matches[3]; D = [int]$Matches[4]; Rate = $(if ($Matches[5]) { [int]$Matches[5] } else { 0 }); Peak = $(if ($Matches[6]) { [int]$Matches[6] } else { -1 }) })
        }
    }
    if ($usbRows.Count -gt 0) {
        Section 'USB AUDIO PATH (PC -> Pico), one reading per second'
        $uSecs = @($usbRows | Where-Object { $_.U -gt 0 })
        $uTotal = ($usbRows | Measure-Object U -Sum).Sum
        $lateSecs = @($usbRows | Where-Object { $_.D -ge 3 })
        $worstLate = ($usbRows | Measure-Object D -Maximum).Maximum
        $fMin = ($usbRows | Measure-Object Fmin -Minimum).Minimum
        $fMax = ($usbRows | Measure-Object Fmax -Maximum).Maximum
        Say ("{0} readings. Silence had to be inserted {1} time(s) in {2} different second(s) ({3:n1}% of the run)." -f $usbRows.Count, $uTotal, $uSecs.Count, (100.0 * $uSecs.Count / $usbRows.Count))
        Say ("USB buffer held between {0} and {1} ms of audio (0 means empty; the PC feeds it in 1 ms pieces). Pico USB task worst lateness {2} ms, late (3 ms or more) in {3} second(s)." -f $fMin, $fMax, $worstLate, $lateSecs.Count)
        $rates = @($usbRows | Where-Object { $_.Rate -gt 0 } | ForEach-Object { $_.Rate } | Select-Object -Unique)
        if ($rates.Count -gt 0) { Say ('USB audio sample rate in use: {0} kHz' -f ($rates -join ', ')); if ($rates -contains 48) { Say '  (48 kHz means the Pico itself converts the audio down to 16 kHz before encoding.)' } }
        $pk = @($usbRows | Where-Object { $_.Peak -ge 0 })
        if ($pk.Count -gt 0) {
            $hot = @($pk | Where-Object { $_.Peak -ge 95 }).Count
            Say ('Incoming audio loudness: loudest second {0}% of full scale, average of the per-second peaks {1:n0}%, seconds at 95% or more: {2} of {3}' -f ($pk | Measure-Object Peak -Maximum).Maximum, ($pk | Measure-Object Peak -Average).Average, $hot, $pk.Count)
            if ($hot -gt 0.2 * $pk.Count) { Hint 'The audio arriving from the PC is very loud (at or near full scale in many seconds). Lower the volume inside the app or the Windows volume mixer and see whether the stutter changes.' }
        }
        $uEp = @($uSecs | Select-Object -First 20)
        foreach ($r in $uEp) { Say ("    {0}:{1:00}  {2} silence insertion(s), buffer {3}-{4} ms, task late {5} ms" -f [int][math]::Floor($r.T / 60), [int]($r.T % 60), $r.U, $r.Fmin, $r.Fmax, $r.D) }
        if ($uSecs.Count -gt 20) { Say ("    ... and {0} more" -f ($uSecs.Count - 20)) }
        if ($uSecs.Count -ge 0.02 * $usbRows.Count) { Hint ("The Pico had to insert silence into the live audio in {0} of {1} seconds because the PC's USB audio arrived late or at the wrong rate. Each insertion is a tiny hole you can hear as a click or stutter. That is a PC-to-Pico problem, not the radio." -f $uSecs.Count, $usbRows.Count) }
        elseif ($uTotal -eq 0) { Hint 'The USB audio feed from the PC was steady for the whole run: no silence had to be inserted.' }
        if ($worstLate -ge 3) { Hint ("The Pico's own USB task ran up to {0} ms late. Something on the Pico (for example the serial logging or the other core) is delaying the USB audio." -f $worstLate) }
        if ($fMax -ge 8) { Hint ("The USB buffer filled up to {0} ms, which means the PC delivers faster than the Pico uses audio (clock drift), adding delay and risking overflow." -f $fMax) }
    }

    if ($stops / $mins -ge 3) { Hint ("During the run the audio stream stopped and restarted {0:n1} times a minute. That pattern means the hearing aids keep running out of transmit credits (issues #47, #48, #54 in the project). Usually radio quality: try the Pico closer to your head, and fewer 2.4 GHz devices nearby." -f ($stops / $mins)) }

    if ($markers.Count -gt 0) {
        Say ''
        Say "You marked $($markers.Count) stutter(s). Pico events within 3 s before to 1 s after each (volume, timing and the once-a-second readings are ignored):"
        # How many marks line up with what the Pico measured automatically?
        $mBad = 0; $mUsb = 0; $mNone = 0; $mAck = 0
        foreach ($m in $markers) {
            $t = $m.HostMs / 1000.0
            $ak = @($ackRows | Where-Object { ($_.K -ge 100 -or $_.A -lt 45) -and $_.T -ge ($t - 4) -and $_.T -le ($t + 2) }).Count
            if ($ak -gt 0) { $mAck++ }
            $b = @($statRows | Where-Object { ($_.N -lt 45 -or $_.Gap -ge 40) -and $_.T -ge ($t - 4) -and $_.T -le ($t + 2) }).Count + $ak
            $u = @($usbRows  | Where-Object { ($_.U -gt 0 -or $_.D -ge 3) -and $_.T -ge ($t - 4) -and $_.T -le ($t + 2) }).Count
            if ($b -gt 0) { $mBad++ }
            if ($u -gt 0) { $mUsb++ }
            if ($b -eq 0 -and $u -eq 0) { $mNone++ }
        }
        if ($ackRows.Count -gt 0) { Say ("  Of your {0} marks, {1} coincide with a late or missed over-the-air acknowledgement from an aid." -f $markers.Count, $mAck) }
        Say ("  Of your {0} marks: {1} coincide with a radio/packet dropout the Pico detected, {2} coincide with USB audio trouble (silence inserted or a late USB task), {3} coincide with neither." -f $markers.Count, $mBad, $mUsb, $mNone)
        if ($mNone -ge 0.6 * $markers.Count) { Hint ("Most of your marks ({0} of {1}) happened while the Pico measured a perfect radio link AND a perfect USB feed. Then the glitch is not in anything the Pico can see: suspect the audio itself (Windows audio engine, sample rate or format, the encoder) or the processors." -f $mNone, $markers.Count) }
        if ($mUsb -ge 0.5 * $markers.Count) { Hint ("{0} of your {1} marks lined up with USB audio trouble inside the Pico (the PC did not deliver audio on time, so the Pico filled the hole with silence). That is a PC-to-Pico problem, not the radio." -f $mUsb, $markers.Count) }
        $quiet = 0
        foreach ($m in $markers) {
            $t = $m.HostMs / 1000.0
            $near = @($live | Where-Object { $_.RelS -ge ($t - 3) -and $_.RelS -le ($t + 1) -and $_.Name -notin $skipNames -and -not ($_.Name -eq 'ShortLog' -and $_.Text -match '^([LR] [nak]\d|usb u|link |upd |phy |setphy |dle )') })
            if ($near.Count -eq 0) { $quiet++ }
            elseif ($near.Count -le 12) { Say ("  stutter at {0:n1}s:" -f $t); foreach ($e in $near) { Say ("    {0:n1}s  {1}  reason 0x{2:x2}" -f $e.RelS, $e.Name, $e.Reason) } }
            else { Say ("  stutter at {0:n1}s: {1} Pico events nearby (busy: connection/reconnect activity)" -f $t, $near.Count) }
        }
        Say ("  {0} of {1} stutters happened with NO Pico event nearby (the Bluetooth side looked quiet)." -f $quiet, $markers.Count)
        if ($quiet -eq $markers.Count) { Hint 'Every stutter you marked happened while the Pico reported nothing unusual. Stream stops, disconnects and credit problems are not the cause. Look at the USB audio path and timing (PC, USB port/cable, Pico USB audio handling), or radio retransmissions the Pico cannot report.' }
        elseif ($quiet -ge 0.7 * $markers.Count) { Hint 'Most stutters happened while the Pico reported nothing unusual, so they are not caused by visible link drops or stream stalls.' }
        elseif ($quiet -eq 0) { Hint 'Every stutter you marked lined up with something the Pico reported. See the list above for which events.' }

        if ($markers.Count -ge 8) {
            $ts = @($markers | ForEach-Object { $_.HostMs / 1000.0 } | Sort-Object)
            $gaps = @(); for ($i = 1; $i -lt $ts.Count; $i++) { $g = $ts[$i] - $ts[$i-1]; if ($g -gt 0.5) { $gaps += $g } }
            if ($gaps.Count -ge 6) {
                $mean = ($gaps | Measure-Object -Average).Average
                $sd = [Math]::Sqrt((($gaps | ForEach-Object { ($_ - $mean) * ($_ - $mean) }) | Measure-Object -Sum).Sum / $gaps.Count)
                Say ("Your marks are spaced {0:n1}s apart on average (spread {1:n1}s)." -f $mean, $sd)
                if (($sd / $mean) -lt 0.45 -and $mean -ge 1.5 -and $mean -le 8) { Hint ("The stutters come at fairly regular intervals (about every {0:n1}s). Regular spacing usually means a timing problem, such as the PC and the Pico drifting apart on the USB audio clock until a buffer slips, and not random radio loss. Switching the Pico to USB Audio Class 1 in the control app, or trying the other firmware build, are good tests." -f $mean) }
            }
        }
    }
} elseif (-not $SkipPico) {
    Say 'No data came back from the Pico. If the port opened, the firmware may not answer this protocol version.'
}

Section 'WHAT THIS SUGGESTS (hints, not proof)'
if ($hints.Count -eq 0) { Say 'Nothing stood out. Send this report to Claude for a closer look.' }
$hints | ForEach-Object { Say "- $_" }

if (-not (Test-Path $ReportDir)) { New-Item -ItemType Directory -Force -Path $ReportDir | Out-Null }
$file = Join-Path $ReportDir ("pico-asha-report-{0}.txt" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
$report | Set-Content -Path $file -Encoding UTF8
Say ''
Say "Report saved to: $file"
