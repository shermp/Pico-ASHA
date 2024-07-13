# Pico-ASHA

Pico-ASHA is an attempt to implement Android's Audio Streaming for Hearing Aids (ASHA) protocol on a Raspberry Pi Pico W microcontroller.

## Status

**This project is still highly experimental.** Currently, it is possible to discover (Oticon) hearing aids, and limited audio streaming has been implemented.

**It should be safe, but I cannot guarantee it will not mess up your hearing aids!**

## How to test/develop

### Compile

Download [pico-sdk](https://github.com/raspberrypi/pico-sdk) 1.5.1, using the provided instructions.

Create an environment variable called `PICO_SDK_PATH` that points to the downloaded SDK location.

Obtain an updated BT firmware to fix an issue with Data Length Extensions (DLE). Download and apply the patch from [this github comment](https://github.com/raspberrypi/pico-sdk/issues/1465#issuecomment-1739329635). Hopefully the next version of the Pico SDK will include this fix.

Build pico-asha
```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

To load the program onto your Pico W, press the `BOOTSEL` button while plugging it into the PC over USB. Then copy the `asha.uf2` onto the mass storage device.

### Serial

By default, Pico-ASHA logs output via UART. To view debugging info over UART, you will need a method to interface with the UART pins. Such options include another pico using the [debugprobe](https://github.com/raspberrypi/debugprobe) project, a USB to UART adapter, The Raspberry Pi debug probe, a full Raspberry Pi or other SBC etc.

Alternatively, Pico-ASHA can log output via USB serial using the standard Raspberry Pi Pico serial settings:
- **Baud Rate:** 115200
- **Data Bits:** 8
- **Stop Bits:"** 1
- **Parity:** None
- **DTR** and **RTS** enabled

Pass `-DENABLE_USB_SERIAL=ON` to cmake during configure to enable USB serial.

### Reloading program

It is possbile to load new versions of the software without having to press the `BOOTSEL` button. If you have a SWD connection setup (by using a debug probe), you can use OpenOCD to upload new firmware.

## Licence

The source code in this repository is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico-ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico-ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.