# Troubleshooting audio break-up

This page is for when audio from the PC reaches your hearing devices but
stutters, clicks or drops out. It describes a way to find out *where* the
problem is, and records what was learned while chasing a stutter on one setup.

!!! Note

    Everything below was worked out on one setup: a Windows 11 PC, a Pico 2 W,
    and two Cochlear Nucleus 8 (CP1110) processors. Other hearing devices and
    other operating systems may behave differently. Treat the findings as
    leads, not as proven fixes.

## Where can audio go wrong?

Audio takes this path, and each step can be checked separately:

1. **PC to Pico over USB.** The PC must deliver 1 ms of audio every millisecond.
2. **Pico encoder.** The Pico converts 48 kHz (or 16 kHz) PCM to G.722.
3. **Pico to hearing device over Bluetooth LE.** One 161 byte packet per ear
   every 20 ms. The hearing device must acknowledge it.
4. **The hearing device itself.** It decodes the stream, and its own sound
   processing (noise reduction and so on) acts on it.

The tools below measure steps 1 to 3. If all three are clean and you still
hear break-up, the cause is probably step 4 or the source audio.

## Quick checks

* Try a USB 2 port, or a short extension cable, so the Pico sits away from the
  PC. USB 3 ports can radiate noise near 2.4 GHz.
* On Windows, turn off *USB selective suspend* (Power Options, advanced
  settings, USB settings).
* Turn Bluetooth off on your phone, and restart the hearing devices. A device
  that is connected to the phone does not advertise to the Pico.
* Unplug and replug the Pico. One bad connection (for example one ear silent,
  or one ear's credits stuck low) was cleared this way.

## Measuring: the link statistics build

Build the firmware with `-DPICO_ASHA_LINK_STATS=ON`:

```sh
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2_w -DPICO_ASHA_LINK_STATS=ON ..
cmake --build .
```

While audio is streaming, the Pico then sends a few short log lines once a
second (they appear as `ShortLog` events in the control app and GUI log).
They are off by default so they do not fill the event history.

| Line | Meaning |
|------|---------|
| `L n50 g22 c4 t3` | Per ear (`L` or `R`): `n` packets delivered this second (50 is perfect), `g` longest gap between packets in ms (about 20 is perfect), `c` lowest L2CAP credit count, `t` longest stall of the Pico's own main loop in ms (1 to 3 is normal). |
| `L a50 k45` | Per ear: `a` packets the hearing device acknowledged over the air (50 is perfect), `k` longest time between acknowledgement reports in ms. |
| `usb u0 f4-4 d0 r48 p42` | USB side: `u` times silence had to be inserted because the PC was late, `f` fewest-most 1 ms frames waiting in the USB buffer, `d` worst lateness of the USB task in ms, `r` sample rate in kHz, `p` loudest incoming sample as a percent of full scale. |
| `link 16/10/100` | Connection interval (1.25 ms units), latency, supervision timeout (10 ms units) the hearing device agreed to. |
| `upd 24/5/600` | The same values after a connection update. |
| `phy st=0 tx=2 rx=2` | A PHY change completed (`2` is the 2M PHY). |
| `setphy ret=0` | The Pico asked for the 2M PHY (build option below). `0` means the controller accepted the request. |
| `dle tx167/14 rx27/3` | Data length extension: payload bytes / time in units of 100 µs. |

!!! Note

    The Pico's Bluetooth chip reports acknowledgements in batches. Gaps of 40
    to 60 ms between `a`/`k` reports are normal and do not mean lost packets.
    Look for `k` of 100 ms or more, or `a` below about 45.

### The troubleshooting script (Windows)

`tools/Pico-ASHA-Troubleshoot.ps1` reads those lines over the USB serial port
and finds the bad moments automatically, so you do not need to mark stutters
by ear. It changes no settings. Close the GUI and the web control app first,
because only one program can use the serial port.

```powershell
powershell -ExecutionPolicy Bypass -File tools\Pico-ASHA-Troubleshoot.ps1 -DurationSeconds 240
```

Play audio, and press `Q` when you have heard enough. The report covers the PC
(USB path, power plan, CPU and DPC load), dropout episodes per ear, the
over-the-air acknowledgements, the USB audio path and a short list of hints.
It also saves a CSV of the per-second readings. Pressing `M` marks a stutter
you hear, and the report then lines your marks up with what the Pico saw.

A second counts as bad when an ear received fewer than 45 packets or had a gap
of 40 ms or more between packets. The hints are guesses, not proof.

## Link tuning options

These build options exist for experiments. The defaults are the original
behaviour.

| Option | Default | Effect |
|--------|---------|--------|
| `PICO_ASHA_CONN_LATENCY` | `10` | Peripheral latency requested when connecting (connection events). |
| `PICO_ASHA_SUPERVISION_TIMEOUT` | `100` | Supervision timeout requested when connecting, in 10 ms units (`400` is 4 s). |
| `PICO_ASHA_REQUEST_2M_PHY` | `OFF` | Ask the controller to switch the link to the 2M PHY after connecting. |
| `PICO_ASHA_LINK_STATS` | `OFF` | The statistics above. |

## What was learned

### Android as a reference

A Bluetooth capture of an Android phone streaming to the same hearing devices
showed:

* It connects with an 18.75 ms interval, latency 0 and a 5 s timeout, requests
  the 2M PHY (both devices accepted), then moves to a 20 ms interval with
  latency 10 and a 1 s timeout, and a data length of 167 bytes.
* After each connection update it writes the ASHA control point *status*
  command `03 02 10` ("connection parameter updated, interval 16") to each
  device, and only then enables notifications and sets the volume.
* Its stream is gapless: one packet pair every 20 ms with consecutive sequence
  numbers.

The Pico did not send the `03 02 10` status write. It now sends it once per
connection.

### Things that were ruled out

* **Replaying the phone's recorded packets** from the Pico (same radio link,
  no encoder, no USB) was clean. So was feeding the phone's decoded audio
  through the Pico's own G.722 encoder. The radio link, the connection
  settings and the encoder are capable of a clean stream.
* **The 48 kHz to 16 kHz filter.** Windows stored the Pico's playback format as
  48 kHz. The filter was checked offline against a reference and is correct.
* **USB feed.** On a 4 minute run the USB buffer stayed at 3 to 4 ms, silence
  was never inserted, and the USB task was never late. So there was no clock
  drift between PC and Pico.
* **Signal level.** Peaks reached 79% of full scale and never clipped.

### Things to avoid

* **Do not advertise only 16 kHz** on Windows if the device was previously
  installed advertising 16 kHz and 48 kHz. Windows keeps the stored 48 kHz
  format and the stream fails to start ("audio renderer error"). Recovery
  needs a normal firmware, a browser restart and sometimes uninstalling the
  device in Device Manager.
* A pure test tone is a poor test signal: the hearing devices' noise reduction
  suppresses steady tones, so a tone can sound wrong when speech is fine.

### Open questions

* On the test setup, builds with the `03 02 10` write, a 4 s supervision timeout
  and the 2M PHY request streamed with about 0.6% bad seconds over 4 minutes,
  where earlier builds went choppy after a few minutes. **Which of those three
  changes is responsible was not isolated.** The 2M request was never confirmed
  by a `phy` line, so its benefit is unproven.
* A slight, repeating degradation (about every 30 seconds) was still heard,
  while the statistics showed a clean link. Its cause is unknown.

## Reporting a problem

Please open an [issue](https://github.com/shermp/Pico-ASHA/issues) and attach
the script's report and CSV. They contain no audio. They do include your PC's
power plan, network adapter and USB device names, and the hearing devices'
Bluetooth addresses, so remove those if you do not want to share them.
