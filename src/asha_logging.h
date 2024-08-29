#pragma once

#include <stdio.h>
#include "pico/time.h"
#include "hci_dump_embedded_stdout.h"
#include "btstack_debug.h"
#include "runtime_settings.hpp"

namespace asha
{

#define asha_log(level, fmt, ...) if (runtime_settings.hci_dump_enabled) { \
    if ((level) == LogLevel::Error) { \
        log_error(fmt, ##__VA_ARGS__); \
    } else { \
        log_info(fmt, ##__VA_ARGS__); \
    } \
} else { \
    printf("[ASHA %5s : %u ms] " fmt "\n", log_level_to_str((level)), to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__); \
}

#define LOG_ERROR(fmt, ...) if (runtime_settings.log_level >= LogLevel::Error) { asha_log(LogLevel::Error, fmt, ##__VA_ARGS__) }
#define LOG_INFO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Info) { asha_log(LogLevel::Info, fmt, ##__VA_ARGS__) }
#define LOG_SCAN(fmt, ...) if (runtime_settings.log_level >= LogLevel::Scan) { asha_log(LogLevel::Scan, fmt, ##__VA_ARGS__) }
#define LOG_AUDIO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Audio) { asha_log(LogLevel::Audio, fmt, ##__VA_ARGS__) }

} // namespace asha
