#pragma once

#if defined (ASHA_LOG_ERROR) || defined (ASHA_LOG_INFO) || defined (ASHA_LOG_AUDIO)
#include <stdio.h>
#include "pico/time.h"
#ifdef ASHA_HCI_DUMP
    #include "hci_dump_embedded_stdout.h"
    #include "btstack_debug.h"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ASHA_HCI_DUMP
    #if defined (ASHA_LOG_ERROR)
        #define LOG_ERROR(fmt, ...) log_error(fmt, ##__VA_ARGS__)
        #define LOG_INFO(fmt, ...)
        #define LOG_SCAN(fmt, ...)
        #define LOG_AUDIO(fmt, ...)
    #elif defined (ASHA_LOG_INFO)
        #define LOG_ERROR(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_INFO(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_SCAN(fmt, ...)
        #define LOG_AUDIO(fmt, ...)
    #elif defined (ASHA_LOG_SCAN)
        #define LOG_ERROR(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_INFO(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_SCAN(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_AUDIO(fmt, ...) 
    #elif defined (ASHA_LOG_AUDIO)
        #define LOG_ERROR(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_INFO(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_SCAN(fmt, ...) log_info(fmt, ##__VA_ARGS__)
        #define LOG_AUDIO(fmt, ...) log_info(fmt, ##__VA_ARGS__)
    #else
        #define LOG_ERROR(fmt, ...)
        #define LOG_INFO(fmt, ...)
        #define LOG_SCAN(fmt, ...)
        #define LOG_AUDIO(fmt, ...)
    #endif
#else
#if defined (ASHA_LOG_ERROR)
    #define LOG_ERROR(fmt, ...) printf("[ASHA ERROR : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_INFO(fmt, ...)
    #define LOG_SCAN(fmt, ...)
    #define LOG_AUDIO(fmt, ...)
#elif defined (ASHA_LOG_INFO)
    #define LOG_ERROR(fmt, ...) printf("[ASHA ERROR : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_INFO(fmt, ...) printf("[ASHA  INFO : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_SCAN(fmt, ...)
    #define LOG_AUDIO(fmt, ...)
#elif defined (ASHA_LOG_SCAN)
    #define LOG_ERROR(fmt, ...) printf("[ASHA ERROR : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_INFO(fmt, ...) printf("[ASHA  INFO : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_SCAN(fmt, ...) printf("[ASHA  SCAN : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_AUDIO(fmt, ...) 
#elif defined (ASHA_LOG_AUDIO)
    #define LOG_ERROR(fmt, ...) printf("[ASHA ERROR : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_INFO(fmt, ...) printf("[ASHA  INFO : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_SCAN(fmt, ...) printf("[ASHA  SCAN : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
    #define LOG_AUDIO(fmt, ...) printf("[ASHA AUDIO : %u ms] " fmt, to_ms_since_boot(get_absolute_time()), ##__VA_ARGS__)
#else
    #define LOG_ERROR(fmt, ...)
    #define LOG_INFO(fmt, ...)
    #define LOG_SCAN(fmt, ...)
    #define LOG_AUDIO(fmt, ...)
#endif

#ifdef __cplusplus
}
#endif
