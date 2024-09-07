# Pico-ASHA Command Protocol

Pico-ASHA implements a simple JSON based protocol over a USB CDC connection.

All JSON is packed without whitespace, with special attention to avoid new lines. 
This allows messages to be sent over serial using standard end-of-line framing.

Pico-ASHA responds to commands sent by the host computer. The only exception 
to this is Pico-ASHA can send log messages at any time. Note, log messages are 
not JSON. Any message received that cannot be decoded as JSON must be treated 
as log output.

Note: examples below include whitespace for clarity. They MUST be packed for transmission.

## Commands

### FW Version

#### Command

```json
{
    "cmd": "asha_fw_ver"
}
```

#### Response

```json
{
    "cmd": "asha_fw_ver",
    "version": "1.0.0-dev"
}
```

### Status

#### Command

```json
{
    "cmd": "status"
}
```

#### Response

```json
{
    "cmd": "status",
    "num_conn": 2,
    "full_set": true,
    "settings": {
        "log_level": "INFO",
        "hci_dump_enabled": true,
        "uart_enabled": true,
        "full_set_paired": true
    }
}
```

### Connected Devices

#### Command

```json
{
    "cmd": "conn_devices"
}
```

#### Response

```json
{
    "cmd": "conn_devices",
    "devices": [
        {
            "addr": "1A:2B:3C:4D:5E:6F",
            "name": "My Hearing Aid",
            "side": "left",
            "mono": true,
            "streaming": true,
            "paused": false
        }
    ]
}
```

### Unpair Device

#### Command

```json
{
    "cmd": "unpair",
    "addr": "1A:2B:3C:4D:5E:6F"
}
```

#### Response

```json
{
    "cmd": "unpair",
    "result": {
        "addr": "1A:2B:3C:4D:5E:6F",
        "success": true
    }
}
```

### Clear Device DB

#### Command

```json
{
    "cmd": "clear_dev_db"
}
```

#### Response

```json
{
    "cmd": "clear_dev_db",
    "success": true
}
```

### Set UART Serial

#### Command

```json
{
    "cmd": "uart_serial",
    "enable": true
}
```

#### Response

```json
{
    "cmd": "uart_serial",
    "success": true
}
```

### Set Log Level

#### Command

`"level"` can be one of `ERROR`, `INFO`, `SCAN`, `AUDIO`

```json
{
    "cmd": "log_level",
    "level": "INFO"
}
```

#### Response

```json
{
    "cmd": "log_level",
    "success": true
}
```

### Enable HCI Dump

Note, sending this command will restart Pico-ASHA.

#### Command

```json
{
    "cmd": "hci_dump",
    "enabled": true
}
```

#### Response

```json
{
    "cmd": "hci_dump",
    "success": true
}
```
