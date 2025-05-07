# Pico-ASHA

Pico-ASHA is an attempt to implement Android's Audio Streaming for Hearing Aids (ASHA) protocol on a Raspberry Pi Pico W microcontroller.

## Status

**This project is still a Work in Progress.** Currently, it is possible to discover and stream audio to Oticon More hearing aids.

### Device Status

| Device | Status |
| --- | --- |
| Oticon More 2 | Working |
| Starkey Arc AI | Can Pair, but not open L2CAP CoC Connection |

**It should be safe, but I cannot guarantee it will not mess up your hearing aids!**

## How to test/develop

### Precompiled builds for testing

Any commits pushed to this repository compiles an ELF and UF2 binary that can be loaded onto a Pico W.

Binaries are available for download from the Github Actions tab. You mist be logged into Github to download these binaries. Firmware is build for both the Pico W and Pico 2W, ensure you flash the correct firmware image for your device.

A GUI is also available, it is pre-compiled for Windows, and can be downloaded alongside the firmware images.

### Compile firmware for dev/testing

Download [pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.1.1, using the provided instructions.

Create an environment variable called `PICO_SDK_PATH` that points to the downloaded SDK location.

As of SDK 2.1.1, the BT firmware does not properly support Data Length Extensions (DLE). Fortunately the upstream driver has now merged the fixed firmware. Navigate to the Pico SDK directory and run

```sh
cd lib/cyw43-driver
git fetch
git checkut 7f422fef4ea5bff7285fb78340d3f28f5461cff2
cd ../..
git add lib/cyw43-driver
git commit -m "Update cyw43-driver for DLE fix"
```

Unfortunately the version of TinyUSB included in Pico SDK 2.1.1 has a bug that prevents USB audio from working. An open PR has a fix, for convenience this fix is supplied as a patch in `patches/tinyusb-v0.18-uac2-fix.patch`. cd to the `lib/tinyusb` directory and `git apply` the patch.

Build pico-asha
```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=MinSizeRel -DPICO_BOARD=pico_w ..
cmake --build .
```

### Compile GUI for dev/testing

A basic Qt GUI has been developed. You will need Qt6 development libraries available so that cmake can find it.

```sh
cd gui
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Installing

To load the program onto your Pico W, press the `BOOTSEL` button while plugging it into the PC over USB. Then copy the `pico-asha.uf2` onto the mass storage device.

Developers with a suitable debug (SWD) connection to the Pico may alternatively load `pico-asha.elf` using OpenOCD. This provides a non-interactive method of loading Pico-ASHA without having to press the `BOOTSEL` button. 

Instructions for doing so are outside the scope of this readme.

### GUI

Pico-ASHA now comes with a basic GUI to see information about connected hearing aids, and provide some diagnostic information. Once you start the GUI, it will automatically attempt to find and connect to Pico-ASHA. There is a "connected" status in the bottom right corner of the window.

The top section has some basic information about each connected hearing aid, as well as the current volume and audio streaming status.

Below that is the ability to dump HCI logs. These logs are only needed if you have issues connecting and streaming. Once saved, the log files can be opened in Wireshark.

The bottom area is a basic text log that can show some status and error codes.

### Testing

It is recommended that your hearing devices are not connected to any phone/tablet/streamer while testing Pico-ASHA. It should not be necessary to unpair from existing bluethooth devices, although this will be device specific.

Currently, Pico-ASHA attempts to pair/bond with any hearing aid devices that are advertising. So to start, you should put your hearing devices into their pairing mode, then start Pico-ASHA by plugging the Pico W into USB, or reloading the firmware, or resetting it.

Once hearing devices have paired/bonded the first time, they will automatically reconnect without needing to be in pairing mode.

Pico-ASHA should appear as a standard USB audio "sound card" in your operating system. To start streaming audio to your hearing devices, set Pico-ASHA as your current sound output device. Note, Pico-ASHA will also show a microphone. Ignore this.

I also highly recommend connecting to the Pico-ASHA serial for logs while testing, either UART or USB as described in previous sections.

## Licence

Unless otherwise noted in the file, the source code in this repository is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico-ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico-ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.