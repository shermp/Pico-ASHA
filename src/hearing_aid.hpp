#pragma once

#include <array>
#include <cstdint>

#include <etl/string.h>

#include "asha_audio.h"
#include "asha_bt.hpp"
#include "asha_comms.hpp"

namespace asha
{

constexpr int ha_process_delay_ticks = 500;

enum class Side {Left = 0, Right = 1, Unset = 2};
enum class Mode {Mono = 0, Binaural = 1, Unset = 2};

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
    uint8_t raw_rop_data[17];

    ROP();
    void read(const uint8_t* data);
    bool operator==(const ROP& other) const = default;
    bool operator!=(const ROP& other) const {return !(operator==(other));}
    void print_values();
};

struct HearingAid
{
    enum ProcessState : uint32_t {
        ProcessUnset               = 1U <<  0,
        DiscoverServices    = 1U <<  1,
        PairBond            = 1U <<  2,
        DataLength          = 1U <<  3,
        DiscoverASHAChar    = 1U <<  4,
        ReadROP             = 1U <<  5,
        ReadPSM             = 1U <<  6,
        DiscoverGAPChar     = 1U <<  7,
        ReadDeviceName      = 1U <<  8,
        DiscoverDISChar     = 1U <<  9,
        ReadMfgName         = 1U << 10,
        ReadModelNum        = 1U << 11,
        ReadFWVers          = 1U << 12,
        ReadSWVers          = 1U << 13,
        ConnectL2CAP        = 1U << 14,
        EnASPNotification   = 1U << 15,
        Finalize            = 1U << 16,
        Audio               = 1U << 17,
        Disconnect          = 1U << 29,
        Done                = 1U << 30,
        ProcessBusy         = 1U << 31
    };

    enum AudioState : uint32_t {
        AudioUnset               = 1U <<  0,
        Ready               = 1U <<  1,
        Start               = 1U <<  2,
        Streaming           = 1U <<  3,
        Stop                = 1U <<  4,
        AudioBusy           = 1U << 31
    };

    bool connected = false;
    bool cached    = false;

    uint32_t process_state = ProcessState::ProcessUnset;
    uint32_t audio_state   = AudioState::AudioUnset;

    HearingAid* other = nullptr;

    /* BT Remote connection vars */

    bd_addr_t addr = {};
    hci_con_handle_t conn_handle = HCI_CON_HANDLE_INVALID;
    uint16_t cid = 0U;

    /* Common BT vars */

    etl::string<32> device_name = {};
    etl::string<32> manufacturer = {};
    etl::string<32> model = {};
    etl::string<32> fw_vers = {};
    etl::string<32> sw_vers = {};

    /* ASHA vars */

    uint8_t psm = 0U;
    ROP rop = {};

    const char* side_str = "Unknown";

    /* SM vars */

    inline static uint8_t auth_req;
    bool paired_and_bonded = false;

    /* Member functions */
    
    HearingAid();

    static void process();
    static int num_connected();
    static bool full_set_connected();
    static std::array<HearingAid*,2> connected_has();

    static void on_serial_host_connected();
    static bool is_addr_connected(const bd_addr_t addr);
    static void set_connections_allowed(bool allowed);
    static void set_audio_streaming_enabled(bool enabled);
    static void set_auto_pair_enabled(bool enabled);
    static void start_scan();
    static void on_ad_report(const AdvertisingReport& report);
    static void connect(const bd_addr_t addr, bd_addr_type_t addr_type);
    static void on_connected(bd_addr_t addr, hci_con_handle_t handle);
    static void on_disconnected(hci_con_handle_t handle, uint8_t status, uint8_t reason);
    static void on_data_len_set(hci_con_handle_t handle, uint16_t rx_octets, uint16_t rx_time, uint16_t tx_octets, uint16_t tx_time);
    static void delete_pair();
    static void delete_pair(uint16_t conn_id);
    static void handle_sm(PACKET_HANDLER_PARAMS);
    static void handle_service_discovery(PACKET_HANDLER_PARAMS);
    static void handle_char_discovery(PACKET_HANDLER_PARAMS);
    static void handle_char_read(PACKET_HANDLER_PARAMS);
    static void handle_acp_write(PACKET_HANDLER_PARAMS);
    static void handle_l2cap_cbm(PACKET_HANDLER_PARAMS);
    static void handle_asp_notification_reg(PACKET_HANDLER_PARAMS);
    static void handle_gatt_notification(PACKET_HANDLER_PARAMS);
    static void process_audio();

private:
    /* GATT structures */

    struct {
        struct {
            gatt_client_service_t service = {};
            gatt_client_characteristic_t device_name = {};
        } gap = {};

        struct {
            gatt_client_service_t service = {};
            gatt_client_characteristic_t manufacture_name = {};
            gatt_client_characteristic_t model_num = {};
            gatt_client_characteristic_t fw_vers = {};
            gatt_client_characteristic_t sw_vers = {};
        } dis = {};

        struct {
            gatt_client_service_t service = {};
            gatt_client_characteristic_t rop = {};
            gatt_client_characteristic_t acp = {};
            gatt_client_characteristic_t asp = {};
            gatt_client_characteristic_t vol = {};
            gatt_client_characteristic_t psm = {};  
        } asha = {};
    } services = {};

    /* L2CAP credit management */

    uint16_t credits = 0;
    int8_t curr_vol = -128;

    // Array to store current AudioControlPoint command packet
    std::array<uint8_t, 5> acp_cmd_packet = {};

    inline static std::array<HearingAid*, 2> hearing_aids;

    int process_delay_ticks = 0;
    int error_count = 0;

    uint32_t curr_read_index = 0U;
    bool first_audio_send = false;
    uint8_t* audio_data = nullptr;

    std::array<uint8_t, ASHA_SDU_SIZE_BYTES> recv_buff = {};

    uint16_t conn_id = comm::unset_conn_id;

    inline static uint16_t next_conn_id;

    inline static bool connections_allowed;
    inline static bool audio_streaming_enabled;
    inline static bool auto_pair_enabled;

    /* Member functions */

    static HearingAid* get_by_con_handle(hci_con_handle_t handle);
    static HearingAid* get_by_cid(uint16_t cid);
    static HearingAid* get_by_conn_id(uint16_t conn_id);
    static HearingAid* get_by_cached_addr(bd_addr_t addr);
    static void set_other_side_ptrs();
    void assign_next_conn_id();
    bool is_connected();
    bool is_streaming();
    void set_process_busy();
    void unset_process_busy();
    void set_audio_busy();
    void unset_audio_busy();
    void set_data_langth();
    void send_acp_start();
    void send_acp_stop();
    void send_acp_status(uint8_t status);
    void send_volume(int8_t volume);
    void disconnect();
    void reset();
    const char* get_side_str();
};

} // namespace asha