# Welcome to Pico ASHA

Pico ASHA is a project to stream audio to hearing devices from a computer or other devices which supports USB audio.

It is most useful for users who do not have LE Audio compatible hearing devices (which is most of them!).

It uses a Raspberry Pi Pico W microcontroller, which appears as a USB headphone device, 
and streams to hearing aids using ASHA, which is how Android phones stream audio to hearing aids.

Follow the [How-to Guide](guide.md) to get started.

## Licence

Unless otherwise noted in the file, the source code for Pico ASHA is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.