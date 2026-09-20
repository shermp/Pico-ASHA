#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <utility>

#include <pico/time.h>

#include <class/cdc/cdc_device.h>

#include <nanocobs/cobs.h>

#include <btstack.h>

#include "asha_comms.hpp"
#include "asha_vers.h"
#include "fixed_queue.hpp"

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

    constexpr size_t zero_prefix = 1;
    constexpr size_t max_hci_packet_len = 180; // Actually 167 for ASHA, but add a few more bytes
    constexpr size_t hci_packet_type = 1;

    constexpr size_t control_frame_size = zero_prefix + std::max({
        COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(CmdPacket)),
        COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(IntroPacket)),
        COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(USBInfo)),
        COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(RemoteInfo))
    });
    constexpr size_t advertising_frame_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(AdvertisingPacket));
    constexpr size_t event_frame_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(EventPacket));
    constexpr size_t hci_frame_size = zero_prefix + COBS_ENCODE_MAX(sizeof(HeaderPacket) + sizeof(BTSnoopPacketHeader) + hci_packet_type + max_hci_packet_len);
    constexpr size_t usb_frame_size = std::max({control_frame_size, advertising_frame_size, event_frame_size, hci_frame_size});

    static_assert(usb_frame_size <= COBS_TINYFRAME_SAFE_BUFFER_SIZE);

    template <size_t N>
    struct EncodedFrame
    {
        uint16_t len = 0;
        std::array<uint8_t, N> data = {};
    };

    using ControlFrame = EncodedFrame<control_frame_size>;
    using AdvertisingFrame = EncodedFrame<advertising_frame_size>;
    using EventFrame = EncodedFrame<event_frame_size>;
    using HciFrame = EncodedFrame<hci_frame_size>;
    using UsbTxFrame = EncodedFrame<usb_frame_size>;

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
    constexpr uint hci_tx_queue_depth = 8;
    constexpr uint advertising_tx_queue_depth = 8;
    constexpr uint event_tx_queue_depth = 8;
    constexpr uint command_queue_depth = 8;
    constexpr size_t hci_history_depth = 32;
    constexpr size_t event_history_depth = 64;
    constexpr int hci_send_limit = 2;
    constexpr int event_send_limit = 2;

    static FixedSpscQueue<ControlFrame, control_tx_queue_depth> control_tx_queue;
    static FixedSpscQueue<HciFrame, hci_tx_queue_depth> hci_tx_queue;
    static FixedSpscQueue<AdvertisingFrame, advertising_tx_queue_depth> advertising_tx_queue;
    static FixedSpscQueue<EventFrame, event_tx_queue_depth> event_tx_queue;
    static FixedSpscQueue<CommandMessage, command_queue_depth> command_queue;
    static std::atomic_bool serial_connected = false;

    // These histories are owned exclusively by core 1. They overwrite their
    // oldest entries so a long serial disconnection retains recent data.
    static FixedOverwriteQueue<HciFrame, hci_history_depth> hci_history;
    static FixedOverwriteQueue<EventFrame, event_history_depth> event_history;
    static uint32_t hci_drop_count = 0;

    // CDC parser and transmitter state are owned exclusively by core 0.
    static std::array<uint8_t, cmd_buff_size> usb_rx_frame = {};
    static size_t usb_rx_frame_len = 0;
    static bool usb_rx_discarding = false;
    static bool usb_was_connected = false;
    static UsbTxFrame usb_tx_frame;
    static size_t usb_tx_offset = 0;
    static bool usb_tx_preserve = false;

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

        return p;
    }

    template<typename T, size_t N>
    static uint16_t encode_packet(Type header_type, uint16_t conn_id, T const& packet,
                                  std::array<uint8_t, N>& buffer)
    {
        auto pkt = construct_packet(header_type, conn_id, packet);
        static_assert(zero_prefix + COBS_ENCODE_MAX(sizeof(pkt)) <= N);

        size_t encoded_len = 0;
        buffer[0] = COBS_FRAME_DELIMITER;
        if (cobs_encode(&pkt, sizeof(pkt), buffer.data() + zero_prefix,
                        buffer.size() - zero_prefix, &encoded_len) != COBS_RET_SUCCESS) {
            return 0;
        }
        return static_cast<uint16_t>(encoded_len + zero_prefix);
    }

    template<typename T, size_t N, size_t Capacity>
    static bool construct_and_queue_packet(Type header_type, uint16_t conn_id, T const& packet,
                                           FixedSpscQueue<EncodedFrame<N>, Capacity>& queue)
    {
        EncodedFrame<N> frame = {};
        frame.len = encode_packet(header_type, conn_id, packet, frame.data);
        return frame.len != 0 && queue.try_push(std::move(frame));
    }

    void init()
    {
        control_tx_queue.reset();
        hci_tx_queue.reset();
        advertising_tx_queue.reset();
        event_tx_queue.reset();
        command_queue.reset();
        hci_history.clear();
        event_history.clear();
        hci_drop_count = 0;
    }

    bool usb_connected()
    {
        return serial_connected.load(std::memory_order_relaxed);
    }

    void add_event_to_buffer(uint16_t const conn_id, EventPacket const& event)
    {
        EventFrame frame = {};
        frame.len = encode_packet(Type::Event, conn_id, event, frame.data);
        if (frame.len != 0) {
            event_history.push(std::move(frame));
        }
    }

    static void try_send_hci_packets(int limit)
    {
        int send_count = 0;
        while (!hci_history.empty() && send_count < limit && usb_connected()) {
            if (!hci_tx_queue.try_push(hci_history.front())) {
                break;
            }
            ++send_count;
            [[maybe_unused]] bool const popped = hci_history.pop();
        }
    }

    void service_tx_queues()
    {
        try_send_hci_packets(hci_send_limit);

        int send_count = 0;
        while (!event_history.empty() && send_count < event_send_limit && usb_connected()) {
            if (!event_tx_queue.try_push(event_history.front())) {
                break;
            }
            ++send_count;
            [[maybe_unused]] bool const popped = event_history.pop();
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
            construct_and_queue_packet(Type::Advert, unset_conn_id, ad_packet, advertising_tx_queue);
        }
    }

    bool get_cmd_packet(HeaderPacket& header, CmdPacket& cmd_packet)
    {
        CommandMessage message = {};
        if (!command_queue.try_pop(message)) {
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
        if (packet_type == LOG_MESSAGE_PACKET) return;

        // Move one older packet first when possible. This keeps the local
        // overwrite history available for a disconnect or a USB backlog.
        try_send_hci_packets(1);

        auto abs_time = get_absolute_time();
        uint32_t incl_len = (len > max_hci_packet_len) ? max_hci_packet_len : len;
        if (hci_history.full()) {
            ++hci_drop_count;
        }
        BTSnoopPacketHeader snoop_header = {};
        snoop_header.orig_len = sizeof(packet_type) + len;
        snoop_header.incl_len = sizeof(packet_type) + incl_len;
        snoop_header.cuml_drops = hci_drop_count;
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

        HciFrame frame = {};
        frame.data[0] = COBS_FRAME_DELIMITER;
        cobs_enc_ctx_t enc_ctx = {};
        size_t encoded_len = 0;

        if (cobs_encode_inc_begin(frame.data.data() + zero_prefix,
                                  frame.data.size() - zero_prefix, &enc_ctx) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &header, sizeof(header)) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &snoop_header, sizeof(snoop_header)) != COBS_RET_SUCCESS
            || cobs_encode_inc(&enc_ctx, &packet_type, sizeof(packet_type)) != COBS_RET_SUCCESS
            || (incl_len != 0 && cobs_encode_inc(&enc_ctx, packet, incl_len) != COBS_RET_SUCCESS)
            || cobs_encode_inc_end(&enc_ctx, &encoded_len) != COBS_RET_SUCCESS) {
            return;
        }

        frame.len = static_cast<uint16_t>(encoded_len + zero_prefix);
        hci_history.push(std::move(frame));
        try_send_hci_packets(1);
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
                    // usb_task() budgets reads against available queue slots,
                    // so this can only fail if that invariant is broken.
                    [[maybe_unused]] bool const queued = command_queue.try_push(std::move(message));
                }
            }

            reset_usb_rx_parser();
            current = delimiter + 1;
        }
    }

    template<typename T, size_t Capacity>
    static void discard_queue(FixedSpscQueue<T, Capacity>& queue)
    {
        T discarded = {};
        while (queue.try_pop(discarded)) {}
    }

    template<size_t N>
    static void prepare_usb_tx(EncodedFrame<N> const& frame, bool preserve_on_disconnect)
    {
        usb_tx_frame.len = frame.len;
        memcpy(usb_tx_frame.data.data(), frame.data.data(), frame.len);
        usb_tx_offset = 0;
        usb_tx_preserve = preserve_on_disconnect;
    }

    static bool load_next_tx_frame()
    {
        ControlFrame control = {};
        HciFrame hci = {};
        AdvertisingFrame advertising = {};
        EventFrame event = {};

        if (control_tx_queue.try_pop(control)) {
            prepare_usb_tx(control, false);
        } else if (hci_tx_queue.try_pop(hci)) {
            prepare_usb_tx(hci, true);
        } else if (advertising_tx_queue.try_pop(advertising)) {
            prepare_usb_tx(advertising, false);
        } else if (event_tx_queue.try_pop(event)) {
            prepare_usb_tx(event, true);
        } else {
            return false;
        }
        return true;
    }

    void usb_task()
    {
        bool const connected = tud_cdc_connected();
        serial_connected.store(connected, std::memory_order_relaxed);

        if (!connected) {
            reset_usb_rx_parser();
            if (usb_tx_preserve) {
                // A leading delimiter lets the receiver discard any partial
                // pre-disconnect copy before this frame is retransmitted.
                usb_tx_offset = 0;
            } else {
                usb_tx_frame.len = 0;
                usb_tx_offset = 0;
            }
            discard_queue(control_tx_queue);
            discard_queue(advertising_tx_queue);
            if (usb_was_connected) {
                tud_cdc_write_clear();
            }
            usb_was_connected = false;
            return;
        }
        usb_was_connected = true;

        size_t const free_commands = command_queue.available();
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
