cmake_minimum_required(VERSION 3.20)

project(PicoASHAGui VERSION 0.1 LANGUAGES CXX C)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)

find_package(Qt6 REQUIRED COMPONENTS Quick SerialPort)

set(ASHA_COMM_INCLUDE_PATH_REL "${PROJECT_SOURCE_DIR}/../include")
cmake_path(NATIVE_PATH ASHA_COMM_INCLUDE_PATH_REL NORMALIZE ASHA_COMM_INCLUDE_PATH)

set(ASHA_COMM_LIB_PATH_REL "${PROJECT_SOURCE_DIR}/../lib")
cmake_path(NATIVE_PATH ASHA_COMM_LIB_PATH_REL NORMALIZE ASHA_COMM_LIB_PATH)

qt_add_executable(appPicoASHAGui
    main.cpp
    remotedevicemodel.h
    remotedevicemodel.cpp
)

target_include_directories(appPicoASHAGui PUBLIC "${ASHA_COMM_INCLUDE_PATH}" "${ASHA_COMM_LIB_PATH}")

qt_add_qml_module(appPicoASHAGui
    URI PicoASHAGui
    VERSION 1.0
    QML_FILES
        Main.qml
        SOURCES
            picoashacomm.h
            picoashacomm.cpp
            remotedevice.h
            remotedevice.cpp
            ../include/asha_comms.hpp
            ../include/bt_status_err.hpp
            ../lib/nanocobs/cobs.c
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appPicoASHAGui PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appPicoASHAGui
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appPicoASHAGui
    PRIVATE Qt6::Quick Qt6::SerialPort
)

include(GNUInstallDirs)
install(TARGETS appPicoASHAGui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
