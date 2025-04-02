#include <pico/time.h>
#include <pico/stdio_usb.h>

#include <etl/circular_buffer.h>
#include <nanocobs/cobs.h>

#include "asha_audio.hpp"
#include "asha_comms.hpp"

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

    static etl::circular_buffer<std::array<uint8_t, cobs_ev_buff_size>, 200> event_buff;

    void add_event_to_buffer(uint16_t const conn_id, EventPacket const& event)
    {
        struct {
            HeaderPacket head;
            EventPacket ev;
        } pkt = {
            .head = {
                .type = Type::Event,
                .len = sizeof(HeaderPacket) + sizeof(EventPacket),
                .conn_id = conn_id,
                .ts_ms = to_ms_since_boot(get_absolute_time())
            },
            .ev = event
        };
        
        event_buff.push({0});
        auto& buff = event_buff.back();
        size_t enc_len = 0;
        cobs_encode(&pkt, sizeof(pkt), buff.data() + 1, buff.size() - 1, &enc_len);
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

} // namespace comm

} // namespace asha