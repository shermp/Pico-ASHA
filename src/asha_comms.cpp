#include <pico/time.h>
#include <pico/stdio_usb.h>

#include <etl/circular_buffer.h>
#include <nanocobs/cobs.h>

#include <btstack.h>

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
    constexpr size_t cmd_buff_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(CmdPacket));
    
    static_assert(cobs_hci_buff_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    static uint8_t cobs_enc_buff[COBS_TINYFRAME_SAFE_BUFFER_SIZE];

    static etl::circular_buffer<std::array<uint8_t, cobs_ev_buff_size>, 200> event_buff;

    static etl::vector<uint8_t, cmd_buff_size> cmd_buff_enc;

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

    bool get_cmd_packet(HeaderPacket& header, CmdPacket& cmd_packet)
    {
        int ch;
        bool got_cmd = false;
        while ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            uint8_t c = (uint8_t)ch;
            if (c == 0) {
                if (cmd_buff_enc.size() == 0) {
                    continue;
                }
                cmd_buff_enc.push_back(c);
                if (cmd_buff_enc.size() > sizeof(HeaderPacket) + sizeof(CmdPacket)) {
                    auto ret = cobs_decode_tinyframe(cmd_buff_enc.data(), cmd_buff_enc.size());
                    if (ret == COBS_RET_SUCCESS) {
                        memcpy(&header, cmd_buff_enc.data() + 1, sizeof(header));
                        if (header.type == Type::Cmd) {
                            memcpy(&cmd_packet, cmd_buff_enc.data() + 1 + sizeof(header), sizeof(cmd_packet));
                            got_cmd = true;
                        }
                    }
                }
                cmd_buff_enc.clear();
                return got_cmd;
            }
            cmd_buff_enc.push_back(c);
        }
        return got_cmd;
    }

    void send_cmd_resp(uint16_t const conn_id, CmdPacket const& resp)
    {
        construct_and_send_packet(Type::Cmd, conn_id, resp);        
    }

    void send_hci_reset()
    {}

    void send_hci_packet(uint8_t packet_type, uint8_t in, uint8_t *packet, uint16_t len)
    {
        if (packet_type == LOG_MESSAGE_PACKET) return;
        auto abs_time = get_absolute_time();
        uint32_t incl_len = (len > max_hci_packet_len) ? max_hci_packet_len : len;
        BTSnoopPacketHeader snoop_header = {};
        snoop_header.orig_len = sizeof(packet_type) + len;
        snoop_header.incl_len = sizeof(packet_type) + incl_len;
        if (in) {
            snoop_header.pkt_flags |= 1;
        }
        if (packet_type == HCI_COMMAND_DATA_PACKET || packet_type == HCI_EVENT_PACKET) {
            snoop_header.pkt_flags |= 2;
        }
        snoop_header.ts_us = to_us_since_boot(abs_time);
        HeaderPacket header = {
            .type = Type::HCI,
            .len = uint8_t(sizeof(HeaderPacket) + sizeof(BTSnoopPacketHeader) + snoop_header.incl_len),
            .conn_id = unset_conn_id,
            .ts_ms = to_ms_since_boot(abs_time)
        };
        snoop_header.byte_swap_fields();

        cobs_enc_buff[0] = 0;
        cobs_enc_ctx_t enc_ctx = {};
        size_t enc_len = 0;

        cobs_encode_inc_begin(cobs_enc_buff + zero_prefix, sizeof(cobs_enc_buff) - zero_prefix, &enc_ctx);
        cobs_encode_inc(&enc_ctx, &header, sizeof(header));
        cobs_encode_inc(&enc_ctx, &snoop_header, sizeof(snoop_header));
        cobs_encode_inc(&enc_ctx, &packet_type, sizeof(packet_type));
        cobs_encode_inc(&enc_ctx, packet, incl_len);
        cobs_encode_inc_end(&enc_ctx, &enc_len);

        stdio_put_string((const char*)cobs_enc_buff, enc_len + zero_prefix, false, false);
        stdio_flush();
    }

    void send_hci_message([[maybe_unused]] int log_level, 
                          [[maybe_unused]] const char * format, 
                          [[maybe_unused]] va_list argptr)
    {}

} // namespace comm

} // namespace asha