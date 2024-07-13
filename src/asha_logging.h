#pragma once

#if defined (ASHA_LOG_ERROR) || defined (ASHA_LOG_INFO) || defined (ASHA_LOG_AUDIO)
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined (ASHA_LOG_ERROR)
    #define LOG_ERROR(...) printf("[Pico-ASHA ERROR] " __VA_ARGS__)
    #define LOG_INFO(...)
    #define LOG_SCAN(...)
    #define LOG_AUDIO(...)
#elif defined (ASHA_LOG_INFO)
    #define LOG_ERROR(...) printf("[Pico-ASHA ERROR] " __VA_ARGS__)
    #define LOG_INFO(...) printf("[Pico-ASHA  INFO] " __VA_ARGS__)
    #define LOG_SCAN(...)
    #define LOG_AUDIO(...)
#elif defined (ASHA_LOG_SCAN)
    #define LOG_ERROR(...) printf("[Pico-ASHA ERROR] " __VA_ARGS__)
    #define LOG_INFO(...) printf("[Pico-ASHA  INFO] " __VA_ARGS__)
    #define LOG_SCAN(...) printf("[Pico-ASHA  SCAN] " __VA_ARGS__)
    #define LOG_AUDIO(...) 
#elif defined (ASHA_LOG_AUDIO)
    #define LOG_ERROR(...) printf("[Pico-ASHA ERROR] " __VA_ARGS__)
    #define LOG_INFO(...) printf("[Pico-ASHA  INFO] " __VA_ARGS__)
    #define LOG_SCAN(...) printf("[Pico-ASHA  SCAN] " __VA_ARGS__)
    #define LOG_AUDIO(...) printf("[Pico-ASHA AUDIO] " __VA_ARGS__)
#else
    #define LOG_ERROR(...)
    #define LOG_INFO(...)
    #define LOG_AUDIO(...)
#endif

#ifdef __cplusplus
}
#endif
