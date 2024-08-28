# Pico-ASHA

Pico-ASHA is an attempt to implement Android's Audio Streaming for Hearing Aids (ASHA) protocol on a Raspberry Pi Pico W microcontroller.

## Status

**This project is still highly experimental.** Currently, it is possible to discover (Oticon) hearing aids, and limited audio streaming has been implemented.

**It should be safe, but I cannot guarantee it will not mess up your hearing aids!**

**Note, I have yet to settle on a USB VID/PID combo. Untill this occurs, I recommend against testing this on Windows. I have only tested on Linux thus far**

## How to test/develop

### Precompiled builds for testing

Any commits pushed to this repository compiles an ELF and UF2 binary that can be loaded onto a Pico W.

Both UART and USB serial binaries are compiled, you can choose whichever is most suitable for you.

Binaries are available for download from the Github Actions tab. You mist be logged into Github to download these binaries.

### Compile for dev/testing

Download [pico-sdk](https://github.com/raspberrypi/pico-sdk) 1.5.1, using the provided instructions.

Create an environment variable called `PICO_SDK_PATH` that points to the downloaded SDK location.

Obtain an updated BT firmware to fix an issue with Data Length Extensions (DLE). Download and apply the patch from [this github comment](https://github.com/raspberrypi/pico-sdk/issues/1465#issuecomment-1739329635). Hopefully the next version of the Pico SDK will include this fix.

Download a [modified btstack](https://github.com/shermp/btstack) with outgoing credit support:

`git clone https://github.com/shermp/btstack.git -b master-credits`.

Create an environment variable called `PICO_BTSTACK_PATH` that points to the downloaded btstack location.

Build pico-asha
```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..
cmake --build .
```

### Installing

To load the program onto your Pico W, press the `BOOTSEL` button while plugging it into the PC over USB. Then copy the `pico-asha.uf2` onto the mass storage device.

Developers with a suitable debug (SWD) connection to the Pico may alternatively load `pico-asha.elf` using OpenOCD. This provides a non-interactive method of loading Pico-ASHA without having to press the `BOOTSEL` button. 

Instructions for doing so are outside the scope of this readme.

### Serial

#### USB

By default, Pico-ASHA logs output via USB serial. The USB serial settings are as follows:

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Stop Bits:"** 1
- **Parity:** None
- **DTR** and **RTS** enabled

You can also reset the Pico into `BOOTSEL` mode (to upload new firmware) by changing the baud rate to 1200 baud.

#### UART

Alternatively, Pico-ASHA can log output via UART. To view debugging info over UART, you will need a method to interface with the UART pins. Such options include another pico using the [debugprobe](https://github.com/raspberrypi/debugprobe) project, a USB to UART adapter, The Raspberry Pi debug probe, a full Raspberry Pi or other SBC etc.

Pass `-DENABLE_USB_SERIAL=OFF` to cmake during configure to enable UART serial.

### Testing

**Again, I advise against testing on Windows at this time**

It is recommended that your hearing devices are not connected to any phone/tablet/streamer while testing Pico-ASHA. It should not be necessary to unpair from existing bluethooth devices, although this will be device specific.

Currently, Pico-ASHA attempts to pair/bond with any hearing aid devices that are advertising. So to start, you should put your hearing devices into their pairing mode, then start Pico-ASHA by plugging the Pico W into USB, or reloading the firmware, or resetting it.

Once hearing devices have paired/bonded the first time, they will automatically reconnect without needing to be in pairing mode.

Pico-ASHA should appear as a standard USB audio "sound card" in your operating system. To start streaming audio to your hearing devices, set Pico-ASHA as your current sound output device. Note, Pico-ASHA will also show a microphone. Ignore this.

I also highly recommend connecting to the Pico-ASHA serial for logs while testing, either UART or USB as described in previous sections.

## Licence

Unless otherwise noted in the file, the source code in this repository is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico-ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico-ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.