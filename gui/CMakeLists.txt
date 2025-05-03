cmake_minimum_required(VERSION 3.20)

project(PicoASHAGui VERSION 0.1 LANGUAGES CXX C)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

set(ASHA_COMM_INCLUDE_PATH_REL "${PROJECT_SOURCE_DIR}/../include")
cmake_path(NATIVE_PATH ASHA_COMM_INCLUDE_PATH_REL NORMALIZE ASHA_COMM_INCLUDE_PATH)

set(ASHA_COMM_LIB_PATH_REL "${PROJECT_SOURCE_DIR}/../lib")
cmake_path(NATIVE_PATH ASHA_COMM_LIB_PATH_REL NORMALIZE ASHA_COMM_LIB_PATH)

qt_add_executable(PicoASHAGui
    main.cpp
    picoashacomm.h
    picoashacomm.cpp
    remotedevice.h
    remotedevice.cpp
    ../include/asha_comms.hpp
    ../include/bt_status_err.hpp
    ../lib/nanocobs/cobs.c
    picoashamainwindow.h picoashamainwindow.cpp
)

target_include_directories(PicoASHAGui PUBLIC "${ASHA_COMM_INCLUDE_PATH}" "${ASHA_COMM_LIB_PATH}")

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(PicoASHAGui PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appPicoASHAGui
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(PicoASHAGui
    PRIVATE Qt6::Widgets Qt6::SerialPort
)

include(GNUInstallDirs)
install(TARGETS PicoASHAGui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

qt_generate_deploy_app_script(
    TARGET PicoASHAGui
    OUTPUT_SCRIPT deploy_script
    NO_COMPILER_RUNTIME
    NO_UNSUPPORTED_PLATFORM_ERROR
    DEPLOY_TOOL_OPTIONS
        "--no-system-d3d-compiler"
        "--no-opengl-sw"
        "--no-opengl"
        "--no-svg"
        "--no-network"
)
install(SCRIPT ${deploy_script})

set(CPACK_VERBATIM_VARIABLES TRUE)
set(CPACK_STRIP_FILES TRUE)
set(CPACK_PACKAGE_VENDOR "shermp")
set(CPACK_PACKAGE_DESCRIPTION "Simple GUI application for Pico-ASHA")
set(CPACK_GENERATOR ZIP)
include(CPack)
