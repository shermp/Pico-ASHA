cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_asha C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

add_executable(asha
  src/main.c
  src/pcm_audio.c
  src/asha.cpp
  src/usb_audio.c
  src/usb_descriptors.c
  src/perf_test.c
  lib/g722/g722_encode.cc
)

target_include_directories(asha PUBLIC src/ lib/)
# target_compile_definitions(asha PUBLIC CFG_TUSB_DEBUG=1)
# target_compile_definitions(asha PUBLIC ASHA_PERF_TEST)

pico_add_extra_outputs(asha)

target_link_libraries(asha
  pico_stdlib
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none
  pico_multicore
  tinyusb_device
  tinyusb_board
)

target_link_options(asha PUBLIC -Wl,--print-memory-usage)

pico_enable_stdio_usb(asha 0)
pico_enable_stdio_uart(asha 1)
