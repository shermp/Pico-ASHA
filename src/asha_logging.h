#pragma once

#include <stdio.h>
#include "etl/circular_buffer.h"
#include "pico/time.h"
#include "hci_dump_embedded_stdout.h"
#include "btstack_debug.h"
#include "runtime_settings.hpp"

namespace asha
{

extern etl::circular_buffer<char[80], 128> log_buffer;

template <typename ... Arg>
static void asha_log(enum LogLevel level, const char* fmt, Arg...args)
{
    if (runtime_settings.hci_dump_enabled) {
        int log_level = (level == LogLevel::Error) ? HCI_DUMP_LOG_LEVEL_ERROR : HCI_DUMP_LOG_LEVEL_INFO;
        hci_dump_log(log_level, fmt, log_level_to_str(level), to_ms_since_boot(get_absolute_time()), args...);
    } else {
        printf(fmt, log_level_to_str(level), to_ms_since_boot(get_absolute_time()), args...);
    }
}

#define ASHA_LOG(level, fmt, ...) asha_log((level), "[ASHA %5s : %u ms] " fmt "\n", ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) if (runtime_settings.log_level >= LogLevel::Error) { ASHA_LOG(LogLevel::Error, fmt, ##__VA_ARGS__); }
#define LOG_INFO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Info) { ASHA_LOG(LogLevel::Info, fmt, ##__VA_ARGS__); }
#define LOG_SCAN(fmt, ...) if (runtime_settings.log_level >= LogLevel::Scan) { ASHA_LOG(LogLevel::Scan, fmt, ##__VA_ARGS__); }
#define LOG_AUDIO(fmt, ...) if (runtime_settings.log_level >= LogLevel::Audio) { ASHA_LOG(LogLevel::Audio, fmt, ##__VA_ARGS__); }

} // namespace asha
