#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>

#include <pico/time.h>
#include <pico/util/queue.h>

#include <class/cdc/cdc_device.h>

#include <etl/circular_buffer.h>
#include <nanocobs/cobs.h>

#include <btstack.h>

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
            cuml_drops = __builtin_bswap32(cuml_drops);
            ts_us = __builtin_bswap64(ts_us);
        }
    };

    // Don't empty a large event buffer all at once
    constexpr int send_limit = 2;

    constexpr size_t zero_prefix = 1;
    constexpr size_t cobs_ev_buff_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(EventPacket));

    static_assert(cobs_ev_buff_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    constexpr size_t max_hci_packet_len = 180; // Actually 167 for ASHA, but add a few more bytes
    constexpr size_t hci_packet_type = 1;
    constexpr size_t cobs_hci_buff_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(BTSnoopPacketHeader) + hci_packet_type + max_hci_packet_len);
    constexpr size_t frame_buff_size = cobs_hci_buff_size;
    
    static_assert(cobs_hci_buff_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    struct EncodedFrame
    {
        uint16_t len = 0;
        std::array<uint8_t, frame_buff_size> data = {};
    };

    struct CommandMessage
    {
        HeaderPacket header;
        CmdPacket packet;
    };

    static_assert(sizeof(CommandMessage) == sizeof(HeaderPacket) + sizeof(CmdPacket));

    // Below 254 decoded bytes, every valid command has exactly this encoded size.
    static_assert(sizeof(CommandMessage) < 254);
    constexpr size_t cmd_buff_size = COBS_ENCODE_MAX(sizeof(CommandMessage));
    constexpr uint control_tx_queue_depth = 8;
    constexpr uint telemetry_tx_queue_depth = 8;
    constexpr uint command_queue_depth = 8;

    static queue_t control_tx_queue;
    static queue_t telemetry_tx_queue;
    static queue_t command_queue;
    static std::atomic_bool serial_connected = false;

    // Encoding is performed only on core 1. queue_try_add() copies the frame,
    // so this scratch buffer is never shared with the USB consumer on core 0.
    static EncodedFrame enc_frame;

    static etl::circular_buffer<std::array<uint8_t, cobs_ev_buff_size>, 200> event_buff;

    // CDC parser and transmitter state are owned exclusively by core 0.
    static std::array<uint8_t, cmd_buff_size> usb_rx_frame = {};
    static size_t usb_rx_frame_len = 0;
    static bool usb_rx_discarding = false;
    static bool usb_was_connected = false;
    static EncodedFrame usb_tx_frame;
    static size_t usb_tx_offset = 0;

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
        static_assert((zero_prefix + COBS_ENCODE_MAX(sizeof(p))) <= frame_buff_size);

        return p;
    }

    template<typename T, size_t N>
    static uint16_t encode_packet(Type header_type, uint16_t conn_id, T const& packet,
                                  std::array<uint8_t, N>& buffer)
    {
        auto pkt = construct_packet(header_type, conn_id, packet);
        static_assert(zero_prefix + COBS_ENCODE_MAX(sizeof(pkt)) <= N);

        size_t enc_len = 0;
        buffer[0] = 0;
        if (cobs_encode(&pkt, sizeof(pkt), buffer.data() + zero_prefix,
                        buffer.size() - zero_prefix, &enc_len) != COBS_RET_SUCCESS) {
            return 0;
        }
        return static_cast<uint16_t>(enc_len + zero_prefix);
    }

    template<typename T>
    static bool construct_and_queue_packet(Type header_type, uint16_t conn_id, T const& packet,
                                           queue_t& queue)
    {
        enc_frame.len = encode_packet(header_type, conn_id, packet, enc_frame.data);
        return enc_frame.len != 0 && queue_try_add(&queue, &enc_frame);
    }

    void init()
    {
        queue_init(&control_tx_queue, sizeof(EncodedFrame), control_tx_queue_depth);
        queue_init(&telemetry_tx_queue, sizeof(EncodedFrame), telemetry_tx_queue_depth);
        queue_init(&command_queue, sizeof(CommandMessage), command_queue_depth);
    }

    bool usb_connected()
    {
        return serial_connected.load(std::memory_order_relaxed);
    }

    void add_event_to_buffer(uint16_t const conn_id, EventPacket const& event)
    {
        event_buff.push({0});
        // The encoder checks at compile time that this slot fits the whole packet.
        encode_packet(Type::Event, conn_id, event, event_buff.back());
    }

    void try_send_events()
    {
        int send_count = 0;
        while (!event_buff.empty() && send_count < send_limit && usb_connected()) {
            const auto& buff = event_buff.front();
            enc_frame.len = static_cast<uint16_t>(buff.size());
            memcpy(enc_frame.data.data(), buff.data(), buff.size());
            if (!queue_try_add(&telemetry_tx_queue, &enc_frame)) {
                break;
            }
            ++send_count;
            event_buff.pop();
        }
    }

    void send_intro_packet(int8_t num_connections, uint16_t flags)
    {
        construct_and_queue_packet(Type::Intro, unset_conn_id, IntroPacket{.pa_version = {
            .major = PICO_ASHA_FW_VERS_MAJOR,
            .minor = PICO_ASHA_FW_VERS_MINOR,
            .patch = PICO_ASHA_FW_VERS_PATCH
        },
        .num_connected = num_connections,
        .flags = flags}, control_tx_queue);
    }

    void send_usb_info_packet(USBInfo const &usb_info)
    {
        construct_and_queue_packet(Type::USBInfo, unset_conn_id, usb_info, control_tx_queue);
    }

    void send_remote_info_packet(RemoteInfo const& remote_info)
    {
        construct_and_queue_packet(Type::RemInfo, unset_conn_id, remote_info, control_tx_queue);
    }

    void send_advertising_packet(AdvertisingPacket const& ad_packet)
    {
        if (usb_connected()) {
            construct_and_queue_packet(Type::Advert, unset_conn_id, ad_packet, telemetry_tx_queue);
        }
    }

    bool get_cmd_packet(HeaderPacket& header, CmdPacket& cmd_packet)
    {
        CommandMessage message = {};
        if (!queue_try_remove(&command_queue, &message)) {
            return false;
        }
        header = message.header;
        cmd_packet = message.packet;
        return true;
    }

    void send_cmd_resp(uint16_t const conn_id, CmdPacket const& resp)
    {
        construct_and_queue_packet(Type::Cmd, conn_id, resp, control_tx_queue);
    }

    void send_hci_reset()
    {}

    void send_hci_packet(uint8_t packet_type, uint8_t in, uint8_t *packet, uint16_t len)
    {
        if (packet_type == LOG_MESSAGE_PACKET || !usb_connected()) return;
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

        enc_frame.data[0] = 0;
        cobs_enc_ctx_t enc_ctx = {};
        size_t enc_len = 0;

        if (cobs_encode_inc_begin(enc_frame.data.data() + zero_prefix,
                                  enc_frame.data.size() - zero_prefix, &enc_ctx) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &header, sizeof(header)) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &snoop_header, sizeof(snoop_header)) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &packet_type, sizeof(packet_type)) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, packet, incl_len) != COBS_RET_SUCCESS
            || cobs_encode_inc_end(&enc_ctx, &enc_len) != COBS_RET_SUCCESS) {
            return;
        }

        enc_frame.len = static_cast<uint16_t>(enc_len + zero_prefix);
        queue_try_add(&telemetry_tx_queue, &enc_frame);
    }

    void send_hci_message([[maybe_unused]] int log_level, 
                          [[maybe_unused]] const char * format, 
                          [[maybe_unused]] va_list argptr)
    {}

    static void reset_usb_rx_parser()
    {
        usb_rx_frame_len = 0;
        usb_rx_discarding = false;
    }

    static void process_usb_rx(uint8_t const* data, size_t len)
    {
        uint8_t const* current = data;
        uint8_t const* const end = data + len;

        while (current != end) {
            uint8_t const* const delimiter = std::find(current, end, COBS_FRAME_DELIMITER);
            size_t const segment_len = static_cast<size_t>(delimiter - current);

            if (!usb_rx_discarding) {
                // Reserve one byte for the delimiter required by cobs_decode().
                size_t const available = usb_rx_frame.size() - 1U - usb_rx_frame_len;
                if (segment_len > available) {
                    usb_rx_frame_len = 0;
                    usb_rx_discarding = true;
                } else {
                    memcpy(usb_rx_frame.data() + usb_rx_frame_len, current, segment_len);
                    usb_rx_frame_len += segment_len;
                }
            }

            if (delimiter == end) {
                return;
            }

            if (!usb_rx_discarding && usb_rx_frame_len != 0) {
                usb_rx_frame[usb_rx_frame_len++] = COBS_FRAME_DELIMITER;
                CommandMessage message = {};
                size_t decoded_len = 0;
                if (cobs_decode(usb_rx_frame.data(), usb_rx_frame_len,
                                &message, sizeof(message), &decoded_len) == COBS_RET_SUCCESS
                    && decoded_len == sizeof(message)
                    && message.header.type == Type::Cmd
                    && message.header.len == sizeof(message)) {
                    queue_try_add(&command_queue, &message);
                }
            }

            reset_usb_rx_parser();
            current = delimiter + 1;
        }
    }

    static void discard_queue(queue_t& queue)
    {
        while (queue_try_remove(&queue, nullptr)) {}
    }

    static bool load_next_tx_frame()
    {
        if (queue_try_remove(&control_tx_queue, &usb_tx_frame)
            || queue_try_remove(&telemetry_tx_queue, &usb_tx_frame)) {
            usb_tx_offset = 0;
            return true;
        }
        return false;
    }

    void usb_task()
    {
        bool const connected = tud_cdc_connected();
        serial_connected.store(connected, std::memory_order_relaxed);

        if (!connected) {
            reset_usb_rx_parser();
            usb_tx_frame.len = 0;
            usb_tx_offset = 0;
            discard_queue(control_tx_queue);
            discard_queue(telemetry_tx_queue);
            if (usb_was_connected) {
                tud_cdc_write_clear();
            }
            usb_was_connected = false;
            return;
        }
        usb_was_connected = true;

        size_t const free_commands = command_queue_depth - queue_get_level(&command_queue);
        if (free_commands != 0) {
            // Leave excess bytes in CDC so USB applies backpressure. Account for
            // a partially received command; only core 0 can fill these slots.
            size_t const rx_budget = free_commands * cmd_buff_size - usb_rx_frame_len;
            std::array<uint8_t, 64> rx;
            uint32_t const rx_count = tud_cdc_read(rx.data(), std::min(rx.size(), rx_budget));
            process_usb_rx(rx.data(), rx_count);
        }

        bool wrote = false;
        while (usb_tx_frame.len != 0 || load_next_tx_frame()) {
            uint32_t const available = tud_cdc_write_available();
            if (available == 0) {
                break;
            }
            size_t const remaining = usb_tx_frame.len - usb_tx_offset;
            uint32_t const requested = static_cast<uint32_t>(std::min<size_t>(available, remaining));
            uint32_t const written = tud_cdc_write(usb_tx_frame.data.data() + usb_tx_offset, requested);
            if (written == 0) {
                break;
            }
            wrote = true;
            usb_tx_offset += written;
            if (usb_tx_offset == usb_tx_frame.len) {
                usb_tx_frame.len = 0;
                usb_tx_offset = 0;
            }
        }
        if (wrote) {
            tud_cdc_write_flush();
        }
    }

} // namespace comm

} // namespace asha
