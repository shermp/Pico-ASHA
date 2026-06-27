# Development

Pico ASHA is developed in C++ (with a bit of C) using the [Pico SDK](https://github.com/raspberrypi/pico-sdk).

The build system is CMake, as that is what the SDK uses.

## Compile firmware

Download [pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.2.0, using the provided instructions.

Create an environment variable called `PICO_SDK_PATH` that points to the downloaded SDK location.

!!! Note

    Prior to Pico SDK 2.2.0, the BT firmware did not properly support Data Length Extensions (DLE), and you had to manually apply a patch.

    As of Pico SDK 2.2.0, the BT firmware has been updated to support DLE. 

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