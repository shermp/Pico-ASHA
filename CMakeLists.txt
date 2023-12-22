cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_asha C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

add_executable(asha
  src/asha.cpp
)

target_include_directories(asha PUBLIC "src/")

pico_add_extra_outputs(asha)

target_link_libraries(asha
  pico_stdlib
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none
)

pico_enable_stdio_usb(asha 0)
pico_enable_stdio_uart(asha 1)
