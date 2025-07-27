# Development

Pico ASHA is developed in C++ (with a bit of C) using the [Pico SDK](https://github.com/raspberrypi/pico-sdk).

The build system is CMake, as that is what the SDK uses.

## Compile firmware

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
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w ..
cmake --build .
```

### Compile GUI

A basic Qt GUI has been developed. You will need Qt6 development libraries (including qtserialport) available so that cmake can find it.

```sh
cd gui
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

!!! Note

    Linux users will most likely need to belong to the `dialout` group to gain serial port permissions.