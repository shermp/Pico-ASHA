# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

SET(CMAKE_CXX_FLAGS_RELEASE "-O2")
SET(CMAKE_C_FLAGS_RELEASE "-O2")

# SET(LOG 2)

include(pico_sdk_import.cmake)

project(pico_asha VERSION 0.2.0 LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

message("Pico SDK version is ${PICO_SDK_VERSION_STRING}")
if(PICO_SDK_VERSION_STRING VERSION_LESS "2.2.0")
  message( FATAL_ERROR "Pico SDK version must be 2.2.0 or greater" )
endif()

Include(FetchContent)

FetchContent_Declare(
  etl
  GIT_REPOSITORY https://github.com/ETLCPP/etl
  GIT_TAG        20.38.17
)
FetchContent_MakeAvailable(etl)

set(ASHA_SRC
  src/main.cpp
  src/runtime_settings.cpp
  src/asha_audio.c
  src/asha_bt.cpp
  src/asha_comms.cpp
  src/asha_led.cpp
  src/hearing_aid.cpp
  src/usb_descriptors.cpp
  src/usb_audio.cpp
  src/util.cpp
  lib/g722/g722_encode.cc
  lib/nanocobs/cobs.c
)

set_source_files_properties(${ASHA_SRC} PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

add_executable(pico_asha ${ASHA_SRC})
pico_set_binary_type(pico_asha copy_to_ram)

target_include_directories(pico_asha PUBLIC src/ lib/ include/)

list(APPEND pico_asha_defines "PICO_ASHA_FW_VERS_MAJOR=${CMAKE_PROJECT_VERSION_MAJOR}")
list(APPEND pico_asha_defines "PICO_ASHA_FW_VERS_MINOR=${CMAKE_PROJECT_VERSION_MINOR}")
list(APPEND pico_asha_defines "PICO_ASHA_FW_VERS_PATCH=${CMAKE_PROJECT_VERSION_PATCH}")

list(APPEND pico_asha_defines "PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1")
pico_enable_stdio_usb(pico_asha 0)
pico_enable_stdio_uart(pico_asha 1)

list(APPEND pico_asha_defines "PICO_FLASH_BANK_TOTAL_SIZE=(FLASH_SECTOR_SIZE * 8u)")
list(APPEND pico_asha_defines "PICO_USE_FASTEST_SUPPORTED_CLOCK=1")

list(APPEND pico_asha_defines "CYW43_RESOURCE_ATTRIBUTE=__attribute__((aligned(4))) __in_flash(\"cyw43_firmware\")")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-Wdeprecated-enum-enum-conversion HAVE_DEP_ENNUM_ENUM_CONV)

if (HAVE_DEP_ENNUM_ENUM_CONV)
    target_compile_options(pico_asha PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)
endif (HAVE_DEP_ENNUM_ENUM_CONV)

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
)

target_link_options(pico_asha PUBLIC -Wl,--print-memory-usage)
