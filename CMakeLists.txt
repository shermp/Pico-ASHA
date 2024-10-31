# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

SET(CMAKE_CXX_FLAGS_RELEASE "-O2")
SET(CMAKE_C_FLAGS_RELEASE "-O2")

set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(pico_asha C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(PICO_ASHA_VERSION "0.1.0-dev")

pico_sdk_init()

message("Pico SDK version is ${PICO_SDK_VERSION_STRING}")
if(PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
  message( FATAL_ERROR "Pico SDK version must be 2.0.0 or greater" )
endif()

Include(FetchContent)

FetchContent_Declare(
  etl
  GIT_REPOSITORY https://github.com/ETLCPP/etl
  GIT_TAG        20.38.17
)
FetchContent_MakeAvailable(etl)

FetchContent_Declare(ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG        v7.1.0
)
FetchContent_MakeAvailable(ArduinoJson)

set(ASHA_SRC
  src/main.cpp
  src/runtime_settings.cpp
  src/asha_audio.cpp
  src/asha_bt_ha.cpp
  src/asha_logging.cpp
  src/asha_led.cpp
  src/pico_bt.cpp
  src/usb_descriptors.cpp
  src/usb_audio.cpp
  src/btstack_ad_dump.c
  src/util.cpp
  lib/g722/g722_encode.cc
)

set_source_files_properties(${ASHA_SRC} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wno-unused-parameter;-Wno-unused-function;-Wno-unused-variable")

add_executable(pico_asha ${ASHA_SRC})

target_include_directories(pico_asha PUBLIC src/ lib/)

option(ENABLE_BLE_AD_DUMP "Enable dumping of advertisement packet data" OFF)
option(ENABLE_UAC2_LOG "Enable USB logging" OFF)
option(ENABLE_PAIR_DEL "Delete all HA pairings" OFF)

list(APPEND pico_asha_defines "PICO_ASHA_VERS=\"${PICO_ASHA_VERSION}\"")

list(APPEND pico_asha_defines "PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1")
pico_enable_stdio_usb(pico_asha 1)
pico_enable_stdio_uart(pico_asha 1)

list(APPEND pico_asha_defines "PICO_FLASH_BANK_TOTAL_SIZE=(FLASH_SECTOR_SIZE * 8u)")

if(ENABLE_BLE_AD_DUMP)
  list(APPEND pico_asha_defines "ASHA_AD_DUMP")
endif()

if (ENABLE_UAC2_LOG)
  list(APPEND pico_asha_defines "CFG_TUSB_DEBUG=1")
endif()

if(ENABLE_PAIR_DEL)
  list(APPEND pico_asha_defines "ASHA_DELETE_PAIRINGS")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wdeprecated-enum-enum-conversion HAVE_DEP_ENNUM_ENUM_CONV)

if (HAVE_DEP_ENNUM_ENUM_CONV)
    target_compile_options(pico_asha PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)
endif (HAVE_DEP_ENNUM_ENUM_CONV)

target_compile_options(pico_asha PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)

target_compile_definitions(pico_asha PUBLIC ${pico_asha_defines})

pico_add_extra_outputs(pico_asha)

target_link_libraries(pico_asha
  pico_stdlib
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none
  pico_multicore
  tinyusb_device
  tinyusb_board
  etl::etl
  ArduinoJson
)

target_link_options(pico_asha PUBLIC -Wl,--print-memory-usage)
