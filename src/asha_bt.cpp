#include <btstack.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio_uart.h>
#include <pico/stdio_usb.h>

#include <hardware/watchdog.h>

#include "asha_bt.hpp"
#include "asha_comms.hpp"
#include "asha_uuid.hpp"
#include "hearing_aid.hpp"
#include "util.hpp"

namespace asha
{
/* Connection parameters for ASHA
   Note, connection interval is in units of 1.25ms */
constexpr uint16_t asha_conn_interval = 20 / 1.25f;

constexpr uint16_t asha_conn_latency  = 10;

static bool enable_send_intro_packet = false;
static bool usb_serial_is_connected = false;

LEDManager led_mgr = {};

LEDManager::Pattern none_connected = {
    .len = 2,
    .interval_ms = 250,
    .delay_ms = 0,
    .pattern = 0b10
};

LEDManager::Pattern one_connected = {
    .len = 2,
    .interval_ms = 100,
    .delay_ms = 0,
    .pattern = 0b10
};

static btstack_packet_callback_registration_t hci_event_cb_reg;
static btstack_packet_callback_registration_t sm_event_cb_reg;

static gatt_client_notification_t notification_listener = {};

static void hci_event_handler(PACKET_HANDLER_PARAMS);

constexpr btstack_time_t ha_process_interval_ms = 2;
static btstack_timer_source_t process_timer = {};
static void process_timer_handler(btstack_timer_source_t * timer);

constexpr btstack_time_t ha_audio_interval_ms = 1;
static btstack_timer_source_t audio_timer = {};

static hci_dump_t pa_hci_dump_impl = {
    .reset = &comm::send_hci_reset,
    .log_packet = &comm::send_hci_packet,
    .log_message = &comm::send_hci_message
};

static void audio_timer_handler(btstack_timer_source_t * timer);

static btstack_timer_source_t auto_pair_timer = {};
static void auto_pair_timer_handler(btstack_timer_source_t * timer);

static btstack_timer_source_t delay_start_timer = {};
static void delay_start_timer_handler(btstack_timer_source_t * timer);

static void add_bonded_to_fal();

static void add_bonded_to_fal()
{
    gap_load_resolving_list_from_le_device_db();

    sm_key_t irk;
    bd_addr_t addr = {};
    int type = BD_ADDR_TYPE_UNKNOWN;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &type, addr, irk);
        if (type != BD_ADDR_TYPE_UNKNOWN) {
            gap_whitelist_add((bd_addr_type_t)type, addr);
        }
    }
}

extern "C" void bt_main()
{
    using namespace comm;

    if (cyw43_arch_init()) {
        return;
    }

    runtime_settings.init();
    if (!runtime_settings) {
        //LOG_ERROR("Runtime settings not initialised");
        add_event_to_buffer(unset_conn_id, EventPacket(EventType::PicoASHAInit, StatusType::PAStatus, PAError::PARuntimeSettingsErr));
    }

    async_context_t *ctx = cyw43_arch_async_context();

    led_mgr.set_ctx(ctx);

    if (runtime_settings.get_hci_dump_enabled()) {
        hci_dump_init(&pa_hci_dump_impl);
    }    

    /* L2CAP init required for basic btstack functionality */
    //LOG_INFO("L2CAP Init.");
    l2cap_init();

    /* Init the security manager. This takes care of pairing
       with hearing aids, and also allows reconnecting to already
       paired hearing aids */
    //LOG_INFO("SM Init.");
    sm_init();
    sm_set_secure_connections_only_mode(false);
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(HearingAid::auth_req);
    sm_allow_ltk_reconstruction_without_le_device_db_entry(0);

    /* Init GATT client. Required to read/write GATT characteristics */
    //LOG_INFO("GATT Client Init.");
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_0);

    /* Setup main packet handlers */
    hci_event_cb_reg.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_cb_reg);

    sm_event_cb_reg.callback = &HearingAid::handle_sm;
    sm_add_event_handler(&sm_event_cb_reg);

    /* Register notification listener */
    gatt_client_listen_for_characteristic_value_updates(
        &notification_listener, 
        &HearingAid::handle_gatt_notification,
        GATT_CLIENT_ANY_CONNECTION,
        NULL
    );

    /* Start BTStack */
    //LOG_INFO("HCI power on.");
    hci_power_control(HCI_POWER_ON);

    /* Do nothing. Audio streaming is scheduled using async_context */
    while (1) {
        sleep_ms(1000);
    }
}

static void process_serial_cmds()
{
    using namespace comm;
    HeaderPacket header = {};
    CmdPacket cmd_pkt = {};
    if (get_cmd_packet(header, cmd_pkt)) {
        cmd_pkt.cmd_status = CmdStatus::CmdOk;
        switch (cmd_pkt.cmd) {
            case Command::HCIDump:
                if (runtime_settings.set_hci_dump_enabled(cmd_pkt.data.enable_hci)) {
                    //LOG_INFO("Enabling Watchdog");
                    watchdog_enable(250, true);
                }
                break;
            case Command::DeletePair:
                if (header.conn_id != unset_conn_id) {
                    HearingAid::delete_pair(header.conn_id);
                } else {
                    HearingAid::delete_pair();
                }
                break;
            case Command::Restart:
                watchdog_enable(250, true);
                break;
            case Command::AllowConnect:
                HearingAid::set_connections_allowed(cmd_pkt.data.allow_connect);
                break;
            case Command::AudioStreaming:
                HearingAid::set_audio_streaming_enabled(cmd_pkt.data.audio_streaming_enabled);
                break;
            case Command::IntroPacket:
                enable_send_intro_packet = true;
                break;
            case Command::PairBond:
                HearingAid::connect(cmd_pkt.data.pair_bond.addr, (bd_addr_type_t)cmd_pkt.data.pair_bond.addr_type);
                break;
            case Command::USBSettings:
                {
                    USBInfo& info = cmd_pkt.data.usb_settings;
                    USBSettings settings = {.uac_version = info.uac_vers, .min_vol = info.min_vol, .max_vol = info.max_vol};
                    if (settings && (settings != usb_settings) && runtime_settings.set_usb_settings(settings)) {
                        watchdog_enable(250, true);
                    }
                }
            default:
                cmd_pkt.cmd_status = CmdStatus::CmdError;
                break;
        }
        send_cmd_resp(header.conn_id, cmd_pkt);
    }
}

static void process_timer_handler(btstack_timer_source_t* timer)
{
    btstack_run_loop_set_timer(timer, ha_process_interval_ms);
    btstack_run_loop_add_timer(timer);

    process_serial_cmds();
    if (stdio_usb_connected()) {
        if (enable_send_intro_packet) {
            HearingAid::on_serial_host_connected();
            enable_send_intro_packet = false;
            usb_serial_is_connected = true;
        }
    } else {
        usb_serial_is_connected = false;
    }
    if (usb_serial_is_connected) {
        comm::try_send_events();
    }
    HearingAid::process();
}

static void audio_timer_handler(btstack_timer_source_t* timer)
{
    btstack_run_loop_set_timer(timer, ha_audio_interval_ms);
    btstack_run_loop_add_timer(timer);

    HearingAid::process_audio();
}

static void auto_pair_timer_handler([[maybe_unused]] btstack_timer_source_t * timer)
{
    if (!usb_serial_is_connected) {
        HearingAid::set_auto_pair_enabled(true);
    }
}

void delay_start_timer_handler(btstack_timer_source_t *timer)
{
    // If dumping HCI packets, we want wait for USB CDC to ensure
    // no packets are lost
    if (runtime_settings.get_hci_dump_enabled() && !stdio_usb_connected()) {
        btstack_run_loop_set_timer(timer, 100);
        btstack_run_loop_add_timer(timer);
    }
    // Set the process timer handler
    btstack_run_loop_set_timer_handler(&process_timer, &process_timer_handler);
    btstack_run_loop_set_timer(&process_timer, ha_process_interval_ms);
    btstack_run_loop_add_timer(&process_timer);

    // Set the audio timer handler
    btstack_run_loop_set_timer_handler(&audio_timer, &audio_timer_handler);
    btstack_run_loop_set_timer(&audio_timer, ha_audio_interval_ms);
    btstack_run_loop_add_timer(&audio_timer);

    // Set the auto pair timer handler
    btstack_run_loop_set_timer_handler(&auto_pair_timer, &auto_pair_timer_handler);
    btstack_run_loop_set_timer(&auto_pair_timer, 10000);
    btstack_run_loop_add_timer(&auto_pair_timer);

    HearingAid::start_scan();

}

static void hci_event_handler(PACKET_HANDLER_PARAMS)
{
    if (packet_type != HCI_EVENT_PACKET) { return; }

    bd_addr_t local_addr;
    uint8_t hci_ev_type = hci_event_packet_get_type(packet);
    AdvertisingReport report;

    switch(hci_ev_type) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                led_mgr.set_led_pattern(none_connected);
                add_bonded_to_fal();
                // Set connection parameters including connection interval
                // by default. Values taken from Android
                gap_set_connection_parameters(0x0030, 0x0030, asha_conn_interval, asha_conn_interval, asha_conn_latency, 100, 12, 12);
                gap_local_bd_addr(local_addr);
                //LOG_INFO("BTstack up and running on %s", bd_addr_to_str(local_addr));
                if (!runtime_settings.get_full_set_paired()) {
                    HearingAid::delete_pair();
                }
                btstack_run_loop_set_timer_handler(&delay_start_timer, &delay_start_timer_handler);
                btstack_run_loop_set_timer(&delay_start_timer, 0);
                btstack_run_loop_add_timer(&delay_start_timer);
            }
            break;
        /* Every time a GAP advertising report is received, handle it here */
        case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
            //LOG_SCAN("Got extended advertising report.");
            [[fallthrough]];
        case GAP_EVENT_ADVERTISING_REPORT:
            if (hci_ev_type == GAP_EVENT_EXTENDED_ADVERTISING_REPORT) {
                report = AdvertisingReport(packet, true);
            } else {
                report = AdvertisingReport(packet, false);
            }
            
            if (HearingAid::is_addr_connected(report.address)) return;
            HearingAid::on_ad_report(report);
            break;
        case HCI_EVENT_META_GAP:
            if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
                hci_con_handle_t handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                bd_addr_t addr = {};
                gap_subevent_le_connection_complete_get_peer_address(packet, addr);
                HearingAid::on_connected(addr, handle);
            }
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE) {
                hci_con_handle_t handle = hci_subevent_le_data_length_change_get_connection_handle(packet);
                uint16_t rx_octets = hci_subevent_le_data_length_change_get_max_rx_octets(packet);
                uint16_t rx_time = hci_subevent_le_data_length_change_get_max_rx_time(packet);
                uint16_t tx_octets = hci_subevent_le_data_length_change_get_max_tx_octets(packet);
                uint16_t tx_time = hci_subevent_le_data_length_change_get_max_tx_time(packet);
                HearingAid::on_data_len_set(handle, rx_octets, rx_time, tx_octets, tx_time);
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
        {
            hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            uint8_t status = hci_event_disconnection_complete_get_status(packet);
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            HearingAid::on_disconnected(handle, status, reason);
            break;
        }
        default:
            break;
    }
}

AdvertisingReport::AdvertisingReport(uint8_t* packet, bool extended)
{
    uint8_t         length;
    const uint8_t * data;
    if (extended) {
        gap_event_extended_advertising_report_get_address(packet, address);
        event_type = gap_event_extended_advertising_report_get_advertising_event_type(packet);
        address_type = (bd_addr_type_t)gap_event_extended_advertising_report_get_address_type(packet);
        rssi = gap_event_extended_advertising_report_get_rssi(packet);
        length = gap_event_extended_advertising_report_get_data_length(packet);
        data = gap_event_extended_advertising_report_get_data(packet);
    } else {
        gap_event_advertising_report_get_address(packet, address);
        event_type = gap_event_advertising_report_get_advertising_event_type(packet);
        address_type = (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet);
        rssi = gap_event_advertising_report_get_rssi(packet);
        length = gap_event_advertising_report_get_data_length(packet);
        data = gap_event_advertising_report_get_data(packet);
    }
    check_if_ha(length, data);
#ifdef ASHA_AD_DUMP
    dump_advertisement_data(data, length);
#endif
}

void AdvertisingReport::check_if_ha(uint8_t length, const uint8_t * data)
{
    ad_context_t context;
    for (ad_iterator_init(&context, length, data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)) {
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t size         = ad_iterator_get_data_len(&context);
        const uint8_t * adv_data = ad_iterator_get_data(&context);
        int i = 0;
        uint8_t tmp_uuid128[16] = {};
        switch(data_type) {
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += sizeof(uint16_t)) {
                if (little_endian_read_16(adv_data, i) == AshaUUID::service16) {
                    //LOG_SCAN("ASHA 16 bit service discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += AshaUUID::service.size()) {
                reverse_128(adv_data + i, tmp_uuid128);
                if (AshaUUID::service == tmp_uuid128) {
                    //LOG_SCAN("ASHA 128 bit UUID service discovered");
                    is_hearing_aid = true;
                } else if (MfiUUID::service == tmp_uuid128) {
                    //LOG_SCAN("MFI UUID discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
            name.clear();
            name.append((const char*)adv_data, size);
            break;
        default:
            break;
        }
    }
}

} // namespace asha