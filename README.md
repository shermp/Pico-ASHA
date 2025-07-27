# Pico ASHA

Pico ASHA is an attempt to implement Android's Audio Streaming for Hearing Aids (ASHA) protocol on a Raspberry Pi Pico W microcontroller.

## Status

**This project is in Beta** Currently, it is possible to discover and stream audio to Oticon hearing aids.

### Device Status

| Device | Status |
| --- | --- |
| Oticon More 2 | Working |
| Starkey Arc AI | Can Pair, but not open L2CAP CoC Connection |

**It should be safe, but I cannot guarantee it will not mess up your hearing aids!**

## Licence

Unless otherwise noted in the file, the source code in this repository is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico-ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico-ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.