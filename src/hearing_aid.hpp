#pragma once

#include <array>
#include <cstdint>
//#include <vector>
#include <etl/vector.h>
#include <etl/string.h>

#include <btstack.h>
#include "asha_audio.hpp"

namespace asha 
{

constexpr size_t max_num_ha = 2;

constexpr uint32_t ha_cache_size = 4ul;

constexpr uint32_t ha_cache_mask = ha_cache_size - 1ul;

constexpr uint16_t max_mtu = 512;

class HA
{
public:
    enum class Side {Left = 0, Right = 1, Unset = 2};
    enum class Mode {Mono = 0, Binaural = 1, Unset = 2};

    /*  Current state of device
        Note: Order is important, as previous state
        enum set on error  */
    enum class State {
        Invalid,
        Cached,
        ServicesDiscovered,
        DiscoverASHAChars,
        ASHACharsDiscovered,
        ReadROP,
        ROPRead,
        ReadPSM,
        PSMRead,
        DiscoverGAPChars,
        GAPCharsDiscovered,
        DiscoverDISChars,
        DISCharsDiscovered,
        ReadDeviceName,
        DeviceNameRead,
        ReadManufacturerName,
        ManufacturerNameRead,
        ReadModelNum,
        ModelNumRead,
        ReadFWVers,
        FWVersRead,
        L2Connecting,
        L2Connected,
        SubscribeASPNotification,
        ASPNotificationSubscribed,
        ACPStart,
        ASPStartOk,
        ASPStopOK,
        AudioPacketReady,
        AudioPacketSending,
        AudioPacketSent,
        ACPStop,
        GAPDisconnect,
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

    void discover_chars();
    void on_char_discovered(uint8_t* char_query_res_packet);
    //void on_char_discovery_complete(uint8_t status);

    void read_char();
    void on_read_char_value(uint8_t* char_val_query_res_packet);
    //void on_read_char_complete(uint8_t status);

    void subscribe_to_asp_notification();

    void create_l2cap_channel();
    void on_l2cap_channel_created(uint8_t status);

    void write_acp_start();
    void write_acp_stop();
    void write_acp_status(uint8_t status);

    void on_gatt_event_query_complete(uint8_t* query_complete_packet);
    void on_asp_notification(int8_t asp_status);

    void set_volume(AudioBuffer::Volume& volume);
    void write_volume();

    void set_write_index(uint32_t write_index);
    void send_audio_packet();
    void on_audio_packet_sent();
    
    bool is_streaming_audio();

    void reset_uncached_vars();

    // Current state of the hearing aid
    State state = State::Invalid;

    // HA Bluetooth address
    bd_addr_t addr = {};

    // HA Bluetooth address type
    bd_addr_type_t addr_type = BD_ADDR_TYPE_UNKNOWN;

    // HA HCI connection handle
    hci_con_handle_t conn_handle = HCI_CON_HANDLE_INVALID;

    // HA L2CAP CoC connection ID
    uint16_t cid = 0u;

    // ASHA L2CAP CoC PSM
    uint8_t psm = 0u;

    etl::string<32> name = {};
    etl::string<32> manufacturer = {};
    etl::string<32> model = {};
    etl::string<32> fw_vers = {};

    // ASHA volume
    int8_t volume = -128;

    HA* other_ha = nullptr;

    const char* side_str = "Unknown";

    uint16_t avail_credits = 0;
    int zero_credit_count = 0;

    struct {
        gatt_client_service_t service = {};
        gatt_client_characteristic_t device_name = {};
    } gap_service = {};

    struct {
        gatt_client_service_t service = {};
        gatt_client_characteristic_t manufacture_name = {};
        gatt_client_characteristic_t model_num = {};
        gatt_client_characteristic_t fw_vers = {};
    } dis_service = {};

    struct {
        gatt_client_service_t service = {};
        gatt_client_characteristic_t rop = {};
        gatt_client_characteristic_t acp = {};
        gatt_client_characteristic_t asp = {};
        gatt_client_characteristic_t vol = {};
        gatt_client_characteristic_t psm = {};  
    } asha_service = {};

    // ASHA ReadOnlyProperties
    struct ROP {
        uint8_t version;
        Side side = Side::Unset;
        Mode mode = Mode::Unset;
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

    int gatt_wait_count = 0;

    // Pointer to current audio packet
    uint8_t* audio_packet = nullptr;

    // Array to store current AudioControlPoint command packet
    std::array<uint8_t, 5> acp_cmd_packet = {};

    // Creating an L2CAP channel requires a receive buffer
    // This is unused, as we only ever send data
    std::array<uint8_t, max_mtu> recv_buff = {};

    btstack_packet_handler_t l2cap_packet_handler = nullptr;
    btstack_packet_handler_t gatt_packet_handler = nullptr;

    uint32_t curr_read_index = 0u;
private:
    bool change_vol = false;
    uint32_t curr_write_index = 0u;
    void dec_state_enum();
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

    HA& add(HA const& new_ha);
    void remove_by_conn_handle(hci_con_handle_t handle);

    etl::vector<HA, max_num_ha> hearing_aids;
private:
    HA invalid_ha = {};
};

} // namespace asha