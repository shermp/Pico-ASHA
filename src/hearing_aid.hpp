#pragma once

#include <array>
#include <cstdint>
//#include <vector>
#include "etl/vector.h"

#include "btstack.h"
#include "asha_audio.hpp"

namespace asha 
{

constexpr size_t max_num_ha = 2;

constexpr uint32_t ha_cache_size = 4ul;

constexpr uint32_t ha_cache_mask = ha_cache_size - 1ul;

class HA
{
public:
    enum class Side {Left = 0, Right = 1};
    enum class Mode {Mono = 0, Binaural = 1};
    enum class State {
        Invalid,
        Cached,
        GATTConnected,
        GATTDisconnect,
        L2Connecting,
        L2Connected,
        SubscribeASPNotification,
        ACPStart,
        ASPStartOk,
        ASPStopOK,
        AudioPacketReady,
        AudioPacketSending,
        AudioPacketSent,
        ACPStop,
    };
    enum class ACPOpCode : uint8_t {
        Start = 1,
        Stop = 2,
        Status = 3,
    };

    HA();

    explicit operator bool() const;

    // Get the side of the hearing aid
    Side side() const;

    // Gets the mode (monoaural/binaural) of the hearing aid
    Mode mode() const;

    void on_gatt_connected();
    void subscribe_to_asp_notification();

    void create_l2cap_channel();
    void on_l2cap_channel_created(uint8_t status);

    void write_acp_start();
    void write_acp_stop();
    void write_acp_status(uint8_t status);

    void on_gatt_event_query_complete(uint8_t att_status);
    void on_asp_notification(int8_t asp_status);

    void set_volume(AudioBuffer::Volume& volume);
    void write_volume();

    void set_write_index(uint32_t write_index);
    void send_audio_packet();
    void on_can_send_audio_packet_now();
    void on_audio_packet_sent();
    
    bool is_streaming_audio();

    void reset_uncached_vars();

    // Current state of the hearing aid
    State state = State::Invalid;

    // HA Bluetooth address
    bd_addr_t addr = {};

    // HA HCI connection handle
    hci_con_handle_t conn_handle = HCI_CON_HANDLE_INVALID;

    // HA L2CAP CoC connection ID
    uint16_t cid = 0u;

    // ASHA L2CAP CoC PSM
    uint8_t psm = 0u;

    // ASHA service
    gatt_client_service_t service = {};

    // ASHA volume
    int8_t volume = -128;

    HA* other_ha = nullptr;

    const char* side_str = "Unknown";

    // ASHA characteristics
    struct {
        gatt_client_characteristic_t rop = {};
        gatt_client_characteristic_t acp = {};
        gatt_client_characteristic_t asp = {};
        gatt_client_characteristic_t vol = {};
        gatt_client_characteristic_t psm = {};  
    } chars = {};

    // ASHA ReadOnlyProperties
    struct ROP {
        uint8_t version;
        Side side;
        Mode mode;
        bool csis_supported;
        struct HiSyncID {
            uint16_t manufacturer_id;
            std::array<uint8_t, 6> unique_id;

            bool operator==(const HiSyncID& other) const = default;
            bool operator!=(const HiSyncID& other) const {return !(operator==(other));}
        } id;
        bool le_coc_supported;
        uint16_t render_delay;
        bool codec_16khz;
        bool codec_24khz;

        ROP();
        void read(const uint8_t* data);
        bool operator==(const ROP& other) const = default;
        bool operator!=(const ROP& other) const {return !(operator==(other));}
        void print_values();
    } rop = {};

    // Pointer to current audio packet
    uint8_t* audio_packet = nullptr;

    // Array to store current AudioControlPoint command packet
    std::array<uint8_t, 5> acp_cmd_packet = {};

    // Creating an L2CAP channel requires a receive buffer
    // This is unused, as we only ever send data
    std::array<uint8_t, sdu_size_bytes> recv_buff = {};

    btstack_packet_handler_t l2cap_packet_handler = nullptr;
    btstack_packet_handler_t gatt_packet_handler = nullptr;

    uint32_t curr_read_index = 0u;
private:
    bool change_vol = false;
    uint32_t curr_write_index = 0u;
};

class HAManager
{
public:
    HAManager();

    HA& get_by_addr(bd_addr_t const addr);
    HA& get_by_conn_handle(hci_con_handle_t const conn_handle);
    HA& get_by_cid(uint16_t const cid);

    bool exists(HA const& ha);
    bool set_complete();

    HA& get_from_cache(const bd_addr_t addr);
    void add_to_cache(HA const& ha);

    HA& add(HA const& new_ha);
    void remove_by_conn_handle(hci_con_handle_t handle);

    etl::vector<HA, max_num_ha> hearing_aids;
private:
    etl::vector<HA, ha_cache_size> cache;
    uint32_t cache_write_index = 0;
    HA invalid_ha = {};
};

} // namespace asha