#pragma once

#include <array>
#include <cstdint>

#include <etl/bitset.h>
#include <etl/delegate.h>

#include "asha_audio.hpp"
#include "pico_bt.hpp"

using namespace picobt;

namespace asha
{

enum class Side {Left = 0, Right = 1};
enum class Mode {Mono = 0, Binaural = 1};

constexpr int8_t asha_mute = -128;

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

    ROP() {}

    void read(const uint8_t* data);
    void print_values();

    bool operator==(const ROP& other) const = default;
    bool operator!=(const ROP& other) const {return !(operator==(other));}
};

struct HearingAid
{
    enum class State {
        Connected,
        Disconnecting,
        DiscoveringServices,
        ServicesDiscovered,
        SetDataLength,
        DataLengthSet,
        PairAndBonding,
        PairedAndBonded,
        DiscoverChars,
        CharsDiscovered,
        ReadChars,
        CharsRead,
        ConnectL2AP,
        L2CAPConnected,
        EnableASPNot,
        ASPNotEnabled,
        AudioReady,
        AudioStarting,
        AudioStarted,
        AudioPending,
        AudioSending,
        AudioStopping,
    };

    enum CharValRead : int {
        DeviceName,
        ReadOnlyProps,
        PSM,
        MaxNumVals
    };

    enum class ACPOpCode : uint8_t {
        Start = 1,
        Stop = 2,
        Status = 3,
    };

    State state = State::Connected;
    int delay_counter = 0;
    BT::Remote* remote = nullptr;
    const char* side_str = "Unknown";
    HearingAid* other_ha = nullptr;

    uint32_t audio_w_index = 0U;
    uint32_t audio_r_index = 0U;
    const uint8_t* audio_data = nullptr;

    uint16_t psm = 0U;
    uint16_t outgoing_credits = 0U;
    ROP rop = {};
    etl::string<32> device_name = {};
    struct Characteristics {
        gatt_client_characteristic_t* acp = nullptr;
        gatt_client_characteristic_t* asp = nullptr;
        gatt_client_characteristic_t* vol = nullptr;
    } asha_chars = {};
    etl::array<uint16_t, CharValRead::MaxNumVals> read_val_handles = {};
    etl::array<uint8_t, sdu_size_bytes> receive_buff = {};
    etl::array<uint8_t, 5> acp_cmd_packet = {};
    int8_t volume = asha_mute;
    uint8_t bt_err = 0U;

    HearingAid(BT::Remote* r);

    bool is_streaming();
    void set_characteristic_data();
    void disconnect() ;
    void discover_services();
    void on_services_discovered(uint8_t status, BT::Remote*);
    bool service_filter(BT::Service const& service);
    void set_data_length();
    void on_data_length_set(BT::Remote*);
    void pair_and_bond();
    void on_paired_and_bonded(uint8_t status, uint8_t reason, BT::Remote*);
    void discover_characteristics();
    void on_chars_discovered(uint8_t status, picobt::BT::Remote*);
    bool discover_chars_filter(gatt_client_characteristic_t*);
    void read_characteristics();
    void on_read_char_value(BT::Remote*, uint16_t val_handle, const uint8_t *data, uint16_t len);
    void on_char_values_read(uint8_t status, picobt::BT::Remote*);
    void connect_l2cap();
    void on_l2cap_connected(uint8_t status, BT::Remote*);
    void on_l2cap_write(BT::Remote*);
    void enable_asp_notification();
    void on_asp_notification_enabled(uint8_t status, BT::Remote*);
    void on_asp_notification_received(BT::Remote*, const uint8_t *data, uint16_t len);
    void write_status(uint8_t acp_status);
    void start_audio();
    void on_audio_start_written(uint8_t status, BT::Remote*);
    void stop_audio();
    void on_audio_stop_written(uint8_t status, BT::Remote*);
    void set_volume(int8_t new_volume);
    void send_audio(uint32_t write_index);

private:
    /* Delegate variables */
    etl::delegate<void(uint8_t, BT::Remote*)> on_services_discovered_d;
    etl::delegate<bool(BT::Service const&)> service_filter_d;
    etl::delegate<void(BT::Remote*)> on_data_length_set_d;
    etl::delegate<void(uint8_t, uint8_t, BT::Remote*)> on_paired_and_bonded_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_chars_discovered_d;
    etl::delegate<bool(gatt_client_characteristic_t*)> discover_chars_filter_d;
    etl::delegate<void(BT::Remote*, uint16_t, const uint8_t*, uint16_t)> on_read_char_value_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_char_values_read_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_l2cap_connected_d;
    etl::delegate<void(BT::Remote*)> on_l2cap_write_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_asp_notification_enabled_d;
    etl::delegate<void(BT::Remote*, const uint8_t*, uint16_t)> on_asp_notification_received_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_audio_start_written_d;
    etl::delegate<void(uint8_t, BT::Remote*)> on_audio_stop_written_d;
};

} // namespace asha