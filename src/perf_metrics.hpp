#pragma once

#ifdef ASHA_PERF_METRICS
#include <vector>
#include "pico/time.h"
#include "stdio.h"

namespace asha
{

constexpr ssize_t metrics_vector_size = 1 * 1024;
struct PerfMetrics {
    std::vector<absolute_time_t> index_changed;
    std::vector<absolute_time_t> send_left;
    std::vector<absolute_time_t> sent_left;
    std::vector<absolute_time_t> send_right;
    std::vector<absolute_time_t> sent_right;

    PerfMetrics() 
    {
        index_changed.reserve(metrics_vector_size);
        send_left.reserve(metrics_vector_size);
        sent_left.reserve(metrics_vector_size);
        send_right.reserve(metrics_vector_size);
        sent_right.reserve(metrics_vector_size);
    }

    void add_index_changed(absolute_time_t t) {if (index_changed.size() < metrics_vector_size) index_changed.push_back(t);}
    void add_send_left(absolute_time_t t) {if (send_left.size() < metrics_vector_size) send_left.push_back(t);}
    void add_sent_left(absolute_time_t t) {if (sent_left.size() < metrics_vector_size) sent_left.push_back(t);}
    void add_send_right(absolute_time_t t) {if (send_right.size() < metrics_vector_size) send_right.push_back(t);}
    void add_sent_right(absolute_time_t t) {if (sent_right.size() < metrics_vector_size) sent_right.push_back(t);}

    bool should_dump_metrics() 
    {
        return  index_changed.size() >= metrics_vector_size; //|| 
                send_left.size() >= metrics_vector_size || 
                sent_left.size() >= metrics_vector_size ||
                send_right.size() >= metrics_vector_size || 
                sent_right.size() >= metrics_vector_size;
    }
    void dump_metrics()
    {
        // for (auto t : index_changed) {
        //     printf("%llu,", t);
        // }
        // printf("\n");
        printf("index_changed,send_left,sent_left,send_right,sent_right\n");
        for (size_t i = 0; i < metrics_vector_size; ++i) {
            absolute_time_t index = (i < index_changed.size()) ? index_changed[i] : 0;
            absolute_time_t send_left_packet = (i < send_left.size()) ? send_left[i] : 0;
            absolute_time_t sent_left_packet = (i < sent_left.size()) ? sent_left[i] : 0;
            absolute_time_t send_right_packet = (i < send_right.size()) ? send_right[i] : 0;
            absolute_time_t sent_rightpacket = (i < sent_right.size()) ? sent_right[i] : 0;
            printf("%llu,%llu,%llu,%llu,%llu\n", index, send_left_packet, sent_left_packet, send_right_packet, sent_rightpacket);
        }
    }
};

extern PerfMetrics perf_metrics;
} // namespace asha
#endif