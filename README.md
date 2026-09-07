# Pico ASHA

Pico ASHA is an attempt to implement Android's Audio Streaming for Hearing Aids (ASHA) protocol on a Raspberry Pi Pico W microcontroller.

Visit the [website](https://shermp.github.io/Pico-ASHA) for documentation.

## Status

**This project is in Beta.** Currently, it is possible to discover and stream audio to several hearing aid models, see the device status table below for details.

### Device Status

| Device | Status |
| --- | --- |
| Audio Service R S 7.6 | Working |
| Cochlear Baha 6 Max | Working |
| Cochlear Nucleus 7 | Working |
| Cochlear Nucleus 8 | Working |
| MED-EL SONNET 3 | Working |
| Oticon More 2 | Working |
| Philips HearLink 9030 | Working |
| Philips HearLink 9050 | Working |
| Signia 5AX | Working |
| Signia Pure Charge&Go T 7AX | Working |
| Sonic Radiant SE 60 | Working |
| Starkey Arc AI | Can Pair, but not open L2CAP CoC Connection |
| Widex Moment 220 | Working |

**It should be safe, but I cannot guarantee it will not mess up your hearing aids!**

## Licence

Unless otherwise noted in the file, the source code in this repository is licenced under the BSD 3-clause licence, however any binaries must be provided on a more restrictive licence, see below.

Pico-ASHA uses [BTstack by BlueKitchen](https://github.com/bluekitchen/btstack), which in general has a non-commercial free licence available. Raspberry Pi have provided a [commercial licence](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_btstack/LICENSE.RP) if using BTstack on official Raspberry Pi Pico W or WH. If Pico-ASHA is implemented on any other device, then the terms of the original BlueKitchen BTstack licence MUST be adhered to.
