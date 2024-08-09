cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(pico_asha C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

Include(FetchContent)

FetchContent_Declare(
  etl
  GIT_REPOSITORY https://github.com/ETLCPP/etl
  GIT_TAG        20.38.17
)

FetchContent_MakeAvailable(etl)

add_executable(pico-asha
  src/main.cpp
  src/asha_audio.cpp
  src/asha_bt.cpp
  src/hearing_aid.cpp
  src/usb_descriptors.cpp
  src/usb_audio.cpp
  src/btstack_ad_dump.c
  lib/g722/g722_encode.cc
)

target_include_directories(pico-asha PUBLIC src/ lib/)

option(ENABLE_BLE_LOG_ERROR "Enable error logging." OFF)
option(ENABLE_BLE_LOG_INFO "Enable info logging (default)." ON)
option(ENABLE_BLE_LOG_SCAN "Enable scan logging." OFF)
option(ENABLE_BLE_LOG_AUDIO "Enable audio logging" OFF)
option(ENABLE_BLE_AD_DUMP "Enable dumping of advertisement packet data" OFF)
option(ENABLE_HCI_DUMP "Enable HCI dump." OFF)
option(ENABLE_UAC2_LOG "Enable USB logging" OFF)
option(ENABLE_PAIR_DEL "Delete all HA pairings" OFF)
option(ENABLE_USB_SERIAL "Enable USB serial support" OFF)

if(ENABLE_BLE_LOG_AUDIO)
  list(APPEND pico_asha_defines "ASHA_LOG_AUDIO")
elseif(ENABLE_BLE_LOG_SCAN)
  list(APPEND pico_asha_defines "ASHA_LOG_SCAN")
elseif(ENABLE_BLE_LOG_INFO)
  list(APPEND pico_asha_defines "ASHA_LOG_INFO")
elseif(ENABLE_BLE_LOG_ERROR)
  list(APPEND pico_asha_defines "ASHA_LOG_ERROR")
endif()

if(ENABLE_BLE_AD_DUMP)
  list(APPEND pico_asha_defines "ASHA_AD_DUMP")
endif()

if(ENABLE_HCI_DUMP)
  list(APPEND pico_asha_defines "ASHA_HCI_DUMP")
endif()

if (ENABLE_UAC2_LOG)
  list(APPEND pico_asha_defines "CFG_TUSB_DEBUG=1")
endif()

if(ENABLE_PAIR_DEL)
  list(APPEND pico_asha_defines "ASHA_DELETE_PAIRINGS")
endif()

if(ENABLE_USB_SERIAL)
  list(APPEND pico_asha_defines "ASHA_USB_SERIAL")
  pico_enable_stdio_usb(pico-asha 1)
  pico_enable_stdio_uart(pico-asha 0)
else()
  pico_enable_stdio_usb(pico-asha 0)
  pico_enable_stdio_uart(pico-asha 1)
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wdeprecated-enum-enum-conversion HAVE_DEP_ENNUM_ENUM_CONV)

if (HAVE_DEP_ENNUM_ENUM_CONV)
    target_compile_options(pico-asha PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)
endif (HAVE_DEP_ENNUM_ENUM_CONV)

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
  etl::etl
)

target_link_options(pico-asha PUBLIC -Wl,--print-memory-usage)
