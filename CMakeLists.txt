cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_asha C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

add_executable(pico-asha
  src/main.c
  src/asha_audio.c
  src/asha_bt.cpp
  src/usb_audio.c
  src/usb_descriptors.c
  lib/g722/g722_encode.cc
)

target_include_directories(pico-asha PUBLIC src/ lib/)

option(ASHA_BLE_LOG_ERROR "Enable error logging." OFF)
option(ASHA_BLE_LOG_INFO "Enable info logging (default)." ON)
option(ASHA_BLE_LOG_AUDIO "Enable audio logging" OFF)
option(HCI_DUMP "Enable HCI dump." OFF)
option(ASHA_USB_LOG "Enable USB logging" OFF)
option(ASHA_DEL_PAIRINGS "Delete all HA pairings" OFF)

if(ASHA_BLE_LOG_AUDIO)
  list(APPEND pico_asha_defines "ASHA_LOG_AUDIO")
elseif(ASHA_BLE_LOG_INFO)
  list(APPEND pico_asha_defines "ASHA_LOG_INFO")
elseif(ASHA_BLE_LOG_ERROR)
  list(APPEND pico_asha_defines "ASHA_LOG_ERROR")
endif()

if(HCI_DUMP)
  list(APPEND pico_asha_defines "ASHA_HCI_DUMP")
endif()

if (ASHA_USB_LOG)
  list(APPEND pico_asha_defines "CFG_TUSB_DEBUG=1")
endif()

if(ASHA_DEL_PAIRINGS)
  list(APPEND pico_asha_defines "ASHA_DELETE_PAIRINGS")
endif()

target_compile_definitions(pico-asha PUBLIC ${pico_asha_defines})

pico_add_extra_outputs(pico-asha)

target_link_libraries(pico-asha
  pico_stdlib
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none
  pico_multicore
  tinyusb_device
  tinyusb_board
)

target_link_options(pico-asha PUBLIC -Wl,--print-memory-usage)

pico_enable_stdio_usb(pico-asha 1)
pico_enable_stdio_uart(pico-asha 0)
