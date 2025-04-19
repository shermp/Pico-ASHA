#include <pico/time.h>
#include <pico/stdio_usb.h>

#include <etl/circular_buffer.h>
#include <nanocobs/cobs.h>

#include "asha_audio.hpp"
#include "asha_comms.hpp"
#include "asha_vers.h"

namespace asha
{

namespace comm
{
    // BTSnoop header based on https://fte.com/webhelpii/bpa600/Content/Technical_Information/BT_Snoop_File_Format.htm
    struct BTSnoopPacketHeader
    {
        uint32_t orig_len;
        uint32_t incl_len;
        uint32_t pkt_flags;
        uint32_t cuml_drops;
        uint64_t ts_us;

        /** 
         * The header is in network byte order, because of course it is...
         * 
         * Assumes RP2040/RP2350 in little endian mode
         */
        void byte_swap_fields()
        {
            orig_len = __builtin_bswap32(orig_len);
            incl_len = __builtin_bswap32(incl_len);
            pkt_flags = __builtin_bswap32(pkt_flags);
            cuml_drops = __builtin_bswap32(pkt_flags);
            ts_us = __builtin_bswap64(ts_us);
        }
    };

    constexpr size_t zero_prefix = 1;
    constexpr size_t cobs_ev_buff_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(EventPacket));

    static_assert(cobs_ev_buff_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    constexpr size_t max_hci_packet_len = 180; // Actually 167 for ASHA, but add a few more bytes
    constexpr size_t hci_packet_type = 1;
    constexpr size_t cobs_hci_buff_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(BTSnoopPacketHeader) + hci_packet_type + max_hci_packet_len);
    
    static_assert(cobs_hci_buff_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    static uint8_t cobs_enc_buff[COBS_TINYFRAME_SAFE_BUFFER_SIZE];

    static etl::circular_buffer<std::array<uint8_t, cobs_ev_buff_size>, 200> event_buff;

    template<typename T>
    static auto construct_packet(Type header_type, uint16_t conn_id, T const& packet)
    {
        struct {
            HeaderPacket head;
            T pkt;
        } p {
            .head = {
                .type = header_type,
                .len = sizeof(HeaderPacket) + sizeof(T),
                .conn_id = conn_id,
                .ts_ms = to_ms_since_boot(get_absolute_time())
            },
            .pkt = packet
        };

        static_assert(sizeof(p) == sizeof(HeaderPacket) + sizeof(T));
        static_assert((zero_prefix + COBS_ENCODE_MAX(sizeof(p))) <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

        return p;
    }

    template<typename T>
    static void construct_and_send_packet(Type header_type, uint16_t conn_id, T const& packet)
    {
        auto pkt = construct_packet(header_type, conn_id, packet);

        size_t enc_len = 0;
        cobs_encode(&pkt, sizeof(pkt), cobs_enc_buff + zero_prefix, sizeof(cobs_enc_buff) - zero_prefix, &enc_len);
        stdio_put_string((const char*)cobs_enc_buff, enc_len + zero_prefix, false, false);
        stdio_flush();
    }

    void add_event_to_buffer(uint16_t const conn_id, EventPacket const& event)
    {        
        auto pkt = construct_packet(Type::Event, conn_id, event);
        event_buff.push({0});
        auto& buff = event_buff.back();
        size_t enc_len = 0;
        cobs_encode(&pkt, sizeof(pkt), buff.data() + zero_prefix, buff.size() - zero_prefix, &enc_len);
    }

    void try_send_events()
    {
        bool flush_req = false;
        while (!event_buff.empty() && stdio_usb_connected()) {
            const auto& buff = event_buff.front();
            stdio_put_string((const char*)buff.data(), buff.size(), false, false);
            flush_req = true;
            event_buff.pop();
        }
        if (flush_req) {
            stdio_flush();
        }
    }

    void send_intro_packet(int8_t num_connections)
    {
        construct_and_send_packet(Type::Intro, unset_conn_id, IntroPacket{.pa_version = {
            .major = PICO_ASHA_FW_VERS_MAJOR,
            .minor = PICO_ASHA_FW_VERS_MINOR,
            .patch = PICO_ASHA_FW_VERS_PATCH
        },
        .num_connected = num_connections});
    }

    void send_remote_info_packet(RemoteInfo const& remote_info)
    {
        construct_and_send_packet(Type::RemInfo, unset_conn_id, remote_info);
    }

} // namespace comm

} // namespace asha