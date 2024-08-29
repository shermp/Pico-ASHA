#pragma once

#include <stdio.h>
#include "pico/time.h"
#include "hci_dump_embedded_stdout.h"
#include "btstack_debug.h"
#include "runtime_settings.hpp"

namespace asha
{

#define asha_log(level, fmt, ...) if (runtime_settings.hci_dump_enabled) { \
    switch ((level)) { \
    case LogLevel::Error: \
        log_error(fmt, ##__VA_ARGS__); \
        break; \
    case LogLevel::Info: \
    case LogLevel::Scan: \
    case LogLevel::Audio: \
        log_info(fmt, ##__VA_ARGS__); \
        break; \
    } \
} else { \
    switch ((level)) { \
    case LogLevel::Error: \
        printf("[ASHA ERROR : %u ms] " fmt "\n", to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__); \
        break; \
    case LogLevel::Info: \
        printf("[ASHA  INFO : %u ms] " fmt "\n", to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__); \
        break; \
    case LogLevel::Scan: \
        printf("[ASHA  SCAN : %u ms] " fmt "\n", to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__); \
        break; \
    case LogLevel::Audio: \
        printf("[ASHA AUDIO : %u ms] " fmt "\n", to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__); \
        break; \
    } \
}

#define LOG_ERROR(fmt, ...) if (runtime_settings.log_level >= LogLevel::Error) asha_log(LogLevel::Error, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Info) asha_log(LogLevel::Info, fmt, ##__VA_ARGS__)
#define LOG_SCAN(fmt, ...) if (runtime_settings.log_level >= LogLevel::Scan) asha_log(LogLevel::Scan, fmt, ##__VA_ARGS__)
#define LOG_AUDIO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Audio) asha_log(LogLevel::Audio, fmt, ##__VA_ARGS__)

} // namespace asha
