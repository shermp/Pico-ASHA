#include <stdio.h>
#include <pico/cyw43_arch.h>
#include <pico/time.h>
#include <hardware/watchdog.h>

#include <pico/stdio_usb.h>
#include <pico/stdio_uart.h>

#include <etl/string.h>

#include <ArduinoJson.h>

#include "asha_logging.h"
#include "asha_uuid.hpp"
#include "asha_bt.hpp"
#include "asha_usb_serial.hpp"
#include "asha_led.hpp"

#include "util.hpp"

namespace asha {

#define ASHA_ASSERT_PACKET_TYPE(pt) if (packet_type != (pt)) return

/* Connection parameters for ASHA
   Note, connection interval is in units of 1.25ms */
constexpr uint16_t asha_conn_interval = 20 / 1.25f;

constexpr uint16_t asha_conn_latency  = 10;

/* Data length variables */
static constexpr uint16_t pdu_len = 167u;

static constexpr uint16_t max_tx_time = 1064;

/* Bonding requirements */

// Bonding with LE Secure Connections
static constexpr uint8_t authreq_le_sc = SM_AUTHREQ_BONDING | SM_AUTHREQ_SECURE_CONNECTION;

// Bonding without LE Secure Connections
static constexpr uint8_t authreq_bonding = SM_AUTHREQ_BONDING;

static constexpr size_t json_resp_str_size = 4096;

static etl::string<json_resp_str_size> response_json = {};

static btstack_packet_callback_registration_t hci_event_cb_reg;
static btstack_packet_callback_registration_t sm_event_cb_reg;

static void hci_event_handler           (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_event_handler            (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_cbm_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void scan_gatt_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void connected_gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void handle_bt_audio_pending_worker(async_context_t *context, async_when_pending_worker_t *worker);
static void handle_stdin_line_worker(async_context_t *context, async_when_pending_worker_t *worker);

static void set_data_length();
static void discover_services();
static void finalise_curr_discovery();
static void delete_paired_devices();
static void delete_paired_device(const bd_addr_t addr);

static LEDManager led_mgr = {};

static LEDManager::Pattern none_connected = {
    .len = 2,
    .interval_ms = 250,
    .delay_ms = 0,
    .pattern = 0b10
};

static LEDManager::Pattern one_connected = {
    .len = 2,
    .interval_ms = 100,
    .delay_ms = 0,
    .pattern = 0b10
};

#ifdef ASHA_AD_DUMP
extern "C" void dump_advertisement_data(const uint8_t * adv_data, uint8_t adv_size);
#endif

static void start_scan();

gatt_client_notification_t notification_listener = {};

static ScanState  scan_state = ScanState::Stop;
static ScanResult curr_scan;
static HAManager  ha_mgr;

AdvertisingReport::AdvertisingReport(uint8_t* packet, bool extended)
{
    uint8_t         length;
    const uint8_t * data;
    if (extended) {
        gap_event_extended_advertising_report_get_address(packet, address);
        event_type = gap_event_extended_advertising_report_get_advertising_event_type(packet);
        address_type = gap_event_extended_advertising_report_get_address_type(packet);
        rssi = gap_event_extended_advertising_report_get_rssi(packet);
        length = gap_event_extended_advertising_report_get_data_length(packet);
        data = gap_event_extended_advertising_report_get_data(packet);
    } else {
        gap_event_advertising_report_get_address(packet, address);
        event_type = gap_event_advertising_report_get_advertising_event_type(packet);
        address_type = gap_event_advertising_report_get_address_type(packet);
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
                    LOG_SCAN("ASHA 16 bit service discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += AshaUUID::service.size()) {
                reverse_128(adv_data + i, tmp_uuid128);
                if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                    LOG_SCAN("ASHA 128 bit UUID service discovered");
                    is_hearing_aid = true;
                } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                    LOG_SCAN("MFI UUID discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        default:
            break;
        }
    }
}

void ScanResult::reset()
{
    service_found = false;
    ha = HA();
    report = AdvertisingReport();
    auth_downgrade_in_progress = false;
}

static void handle_bt_audio_pending_worker([[maybe_unused]] async_context_t *context, 
                                           [[maybe_unused]] async_when_pending_worker_t *worker)
{
    // No need to do anything if no hearing aids are connected!
    using enum HA::State;
    if (ha_mgr.hearing_aids.empty()) return;
    
    uint32_t write_index = audio_buff.get_write_index();
    AudioBuffer::Volume vol = audio_buff.get_volume();
    bool pcm_is_streaming = audio_buff.pcm_streaming.Load();
    for (auto& ha : ha_mgr.hearing_aids) {
        switch(ha.state) {
        case ServicesDiscovered:
            ha.state = DiscoverASHAChars;
            ha.discover_chars();
            break;
        case ASHACharsDiscovered:
            ha.state = ReadROP;
            ha.read_char();
            break;
        case ROPRead:
            ha.state = ReadPSM;
            ha.read_char();
            break;
        case PSMRead:
            ha.state = DiscoverGAPChars;
            ha.discover_chars();
            break;
        case GAPCharsDiscovered:
            ha.state = DiscoverDISChars;
            ha.discover_chars();
            break;
        case DISCharsDiscovered:
            ha.state = ReadDeviceName;
            ha.read_char();
            break;
        case DeviceNameRead:
            ha.state = ReadManufacturerName;
            ha.read_char();
            break;
        case ManufacturerNameRead:
            ha.state = ReadModelNum;
            ha.read_char();
            break;
        case ModelNumRead:
            ha.state = ReadFWVers;
            ha.read_char();
            break;
        case FWVersRead:
            ha.state = SubscribeASPNotification;
            ha.subscribe_to_asp_notification();
            break;
        case ASPNotificationSubscribed:
            if (ha_mgr.set_complete()) {
                runtime_settings.set_full_set_paired(true);
                led_mgr.set_led(LEDManager::State::On);
                scan_state = ScanState::Complete;
            }
            ha.state = L2Connecting;
            ha.create_l2cap_channel();
            break;
        case L2Connected:
            ha.avail_credits = l2cap_cbm_available_credits(ha.cid);
            /* Ensure sufficient credits are available to (re)start
               audio streaming */
            if (pcm_is_streaming && ha.avail_credits >= 8) {
                ha.write_acp_start();
            }
            break;
        case AudioPacketReady:
        case AudioPacketSending:
        case AudioPacketSent:
        case ASPStartOk:
            ha.avail_credits = l2cap_cbm_available_credits(ha.cid);
            if (!pcm_is_streaming) {
                LOG_INFO("%s: USB audio no longer streaming. Stopping ASHA stream", ha.side_str);
                ha.write_acp_stop();
                audio_buff.encode_audio = false;
                continue;
            }
            audio_buff.encode_audio = true;

            if (write_index == 0) continue;

            ha.set_write_index(write_index);
            ha.set_volume(vol);
            ha.send_audio_packet();
            break;
        default:
            break;
        }
    }
}

static void handle_stdin_line_worker([[maybe_unused]] async_context_t *context, 
                                     [[maybe_unused]] async_when_pending_worker_t *worker)
{
    JsonDocument cmd_doc;
    JsonDocument resp_doc;
    auto err = deserializeJson(cmd_doc, complete_std_line);
    if (err != DeserializationError::Ok) {
        // Dump the log if the user started monitoring USB serial late
        if (err == DeserializationError::EmptyInput) {
            if (logging_ctx) {
                async_context_set_work_pending(logging_ctx, &logging_pending_worker);
            }
        } else {
            printf("%s\r\n", R"("{"cmd":"error"}")");
        }
        return;
    }
    const char* cmd = cmd_doc["cmd"];
    resp_doc["cmd"] = cmd;

    auto cmd_is = [&](const char* command) {return str_eq(cmd, command);};

    if (cmd_is(SerCmd::AshaFWVers)) {
        resp_doc[SerCmd::AshaFWVers] = PICO_ASHA_VERS;

    } else if (cmd_is(SerCmd::Status)) {
        resp_doc["num_conn"] = ha_mgr.hearing_aids.size();
        resp_doc["full_set"] = ha_mgr.set_complete();
        JsonObject settings = resp_doc["settings"].to<JsonObject>();
        settings["log_level"] = log_level_to_str(runtime_settings.log_level);
        settings["hci_dump_enabled"] = runtime_settings.hci_dump_enabled;
        settings["uart_enabled"] = runtime_settings.serial_uart_enabled;
        settings["full_set_paired"] = runtime_settings.full_set_paired;

    } else if (cmd_is(SerCmd::ConnDevices)) {
        JsonArray devices = resp_doc["devices"].to<JsonArray>();
        for (auto& ha : ha_mgr.hearing_aids) {
            JsonObject dev = devices.add<JsonObject>();
            dev["addr"] = bd_addr_to_str(ha.addr);
            dev["name"] = ha.name;
            dev["side"] = ha.side_str;
            dev["mono"] = ha.rop.mode == HA::Mode::Mono;
            dev["streaming"] = ha.is_streaming_audio();
            dev["paused"] = false;
        }

    } else if (cmd_is(SerCmd::ClearDevDb)) {
        delete_paired_devices();
        runtime_settings.set_full_set_paired(false);
        resp_doc["success"] = true;

    } else if (cmd_is(SerCmd::UartSerial)) {
        bool uart_enabled = cmd_doc["enabled"];
        if (runtime_settings.set_uart_enabled(uart_enabled)) {
            stdio_set_driver_enabled(&stdio_uart, runtime_settings.serial_uart_enabled);
        }
        resp_doc["success"] = true;

    } else if (cmd_is(SerCmd::LogLevel)) {
        const char* log_level = cmd_doc["level"];
        enum LogLevel ll = str_to_log_level(log_level);
        if (ll != LogLevel::None) {
            runtime_settings.log_level = ll;
            resp_doc["success"] = true;
        } else {
            resp_doc["success"] = false;
        }
    
    } else if (cmd_is(SerCmd::HCIDump)) {
        bool hci_dump_enabled = cmd_doc["enabled"];
        if (runtime_settings.set_hci_dump_enabled(hci_dump_enabled)) {
            LOG_INFO("Enabling Watchdog");
            watchdog_enable(250, true);
        }
        resp_doc["success"] = true;
    }
    // else {
    //     resp_doc["cmd"] = "unknown";
    // }
    size_t len = serializeJson(resp_doc, response_json.data(), response_json.capacity() - 1);
    response_json.uninitialized_resize(len);
    printf("%s\r\n", response_json.c_str());
}

extern "C" void bt_main()
{
    if (cyw43_arch_init()) {
        return;
    }
    runtime_settings.init();
    runtime_settings.get_settings();

    stdio_set_driver_enabled(&stdio_uart, runtime_settings.serial_uart_enabled);

    async_context_t *ctx = cyw43_arch_async_context();
    bt_audio_pending_worker.do_work = handle_bt_audio_pending_worker;
    async_context_add_when_pending_worker(ctx, &bt_audio_pending_worker);
    bt_async_ctx = ctx;

    stdin_pending_worker.do_work = handle_stdin_line_worker;
    async_context_add_when_pending_worker(ctx, &stdin_pending_worker);
    usb_ser_ctx = ctx;

    logging_pending_worker.do_work = handle_logging_pending_worker;
    async_context_add_when_pending_worker(ctx, &logging_pending_worker);
    logging_ctx = ctx;

    // gatt_worker.do_work = handle_gatt_worker;
    // gatt_ctx = ctx;

    led_mgr.set_ctx(ctx);

    if (runtime_settings.hci_dump_enabled) {
        // Allow time for USB serial to connect before proceeding
        while (!stdio_usb_connected()) {
            sleep_ms(250);
        }
        sleep_ms(250);
    }

    if (!runtime_settings) {
        LOG_ERROR("Runtime settings not initialised");
    }
    LOG_INFO("BT ASHA starting.");
    
    /* Start init BTStack systems */
    if (runtime_settings.hci_dump_enabled) {
        hci_dump_init(hci_dump_embedded_stdout_get_instance());
    }
    /* L2CAP init required for basic btstack functionality */
    LOG_INFO("L2CAP Init.");
    l2cap_init();
    l2cap_set_max_le_mtu(max_mtu);

    /* Init the security manager. This takes care of pairing
       with hearing aids, and also allows reconnecting to already
       paired hearing aids */
    LOG_INFO("SM Init.");
    sm_init();
    sm_set_secure_connections_only_mode(false);
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_allow_ltk_reconstruction_without_le_device_db_entry(0);

    /* Init GATT client. Required to read/write GATT characteristics */
    LOG_INFO("GATT Client Init.");
    gatt_client_init();
    gatt_client_set_required_security_level(LEVEL_2);

    /* Setup main packet handlers */
    hci_event_cb_reg.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_cb_reg);

    sm_event_cb_reg.callback = &sm_event_handler;
    sm_add_event_handler(&sm_event_cb_reg);

    /* Register notification listener */
    gatt_client_listen_for_characteristic_value_updates(
        &notification_listener, 
        &connected_gatt_event_handler,
        GATT_CLIENT_ANY_CONNECTION,
        NULL
    );

    /* Start BTStack */
    LOG_INFO("HCI power on.");
    hci_power_control(HCI_POWER_ON);

    /* Do nothing. Audio streaming is scheduled using async_context */
    while (1) {
        sleep_ms(1000);
    }
}

/* Start scanning for suitable peripherals */
static void start_scan()
{
    LOG_INFO("Start scanning.");
    scan_state = ScanState::Scan;
    gap_set_scan_params(1, 0x0030, 0x0030, 0);
    gap_start_scan();
}

static void set_data_length()
{
    if (scan_state != ScanState::DataLen) return;
    while (1) {
        if (hci_can_send_command_packet_now()) {
            hci_send_cmd(&hci_le_set_data_length, curr_scan.ha.conn_handle, pdu_len, max_tx_time);
            break;
        }
    }
}

static void discover_services()
{
    if (scan_state != ScanState::ServiceDiscovery) return;

    LOG_INFO("Device paired. Discovering ASHA service");
    auto res = gatt_client_discover_primary_services(&scan_gatt_event_handler, curr_scan.ha.conn_handle);
    //auto res = gatt_client_discover_primary_services_by_uuid128(&scan_gatt_event_handler, curr_scan.ha.conn_handle, AshaUUID::service);
    if (res != ERROR_CODE_SUCCESS) {
        LOG_ERROR("Could not register service query: %d", static_cast<int>(res));
        scan_state = ScanState::Disconnecting;
        gap_disconnect(curr_scan.ha.conn_handle);
    }
}

static void hci_event_handler(uint8_t packet_type, 
                             [[maybe_unused]] uint16_t channel, 
                             uint8_t *packet, 
                             [[maybe_unused]] uint16_t size)
{
    ASHA_ASSERT_PACKET_TYPE(HCI_EVENT_PACKET);
    bd_addr_t local_addr;

    uint8_t hci_ev_type = hci_event_packet_get_type(packet);

    switch(hci_ev_type) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            led_mgr.set_led_pattern(none_connected);
            // Set 2M PHY
            gap_set_connection_phys(2);
            // Set connection parameters including connection interval
            // by default. Values taken from Android
            gap_set_connection_parameters(0x0030, 0x0030, asha_conn_interval, asha_conn_interval, asha_conn_latency, 100, 12, 12);
            gap_local_bd_addr(local_addr);
            LOG_INFO("BTstack up and running on %s", bd_addr_to_str(local_addr));
        if (!runtime_settings.full_set_paired) {
            delete_paired_devices();
        }
            start_scan();
        } else {
            scan_state = ScanState::Stop;
        }
        break;
    /* Every time a GAP advertising report is received, handle it here */
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        LOG_SCAN("Got extended advertising report.");
        [[fallthrough]];
    case GAP_EVENT_ADVERTISING_REPORT:
        if (scan_state != ScanState::Scan) return;
        sm_set_authentication_requirements(authreq_le_sc);
        if (hci_ev_type == GAP_EVENT_EXTENDED_ADVERTISING_REPORT) {
            curr_scan.report = AdvertisingReport(packet, true);
        } else {
            curr_scan.report = AdvertisingReport(packet, false);
        }
        
        if (ha_mgr.get_by_addr(curr_scan.report.address)) return;

        // See if HA is already paired. If not paired, check if
        // advertising report contains hearing aid service data
        LOG_SCAN("Got Ad Report from %s", bd_addr_to_str(curr_scan.report.address));
        LOG_SCAN("RSSI: %d", (int)((int8_t)curr_scan.report.rssi));
        scan_state = ScanState::IdentityResolving;
        sm_address_resolution_lookup(curr_scan.report.address_type, curr_scan.report.address);
        break;
    case HCI_EVENT_META_GAP:
        switch(hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
        {
            if (scan_state != ScanState::Connecting) return;
            curr_scan.ha.conn_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
            uint16_t ci = gap_subevent_le_connection_complete_get_conn_interval(packet);
            uint16_t cl = gap_subevent_le_connection_complete_get_conn_latency(packet);
            if (ci != asha_conn_interval || cl != asha_conn_latency) {
                LOG_ERROR("Bad connection params. Got Interval: %hu  latency: %hu", ci, cl);
            }
            //curr_scan.ha.supervision_timeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
            scan_state = ScanState::Pairing;
            LOG_INFO("Device connected. Attempt pairing");
            sm_request_pairing(curr_scan.ha.conn_handle);
            break;
        }
        default:
            break;
        }
        break;
    case HCI_EVENT_LE_META:
        switch(hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE:
        {
            if (scan_state != ScanState::DataLen) return;
            auto rx_octets = hci_subevent_le_data_length_change_get_max_rx_octets(packet);
            auto rx_time = hci_subevent_le_data_length_change_get_max_rx_time(packet);
            auto tx_octets = hci_subevent_le_data_length_change_get_max_tx_octets(packet);
            auto tx_time = hci_subevent_le_data_length_change_get_max_tx_time(packet);
            LOG_INFO("DL set to: RX Octets: %hu, RX Time: %hu us, TX Octets: %hu, TX Time: %hu us",
                     rx_octets, rx_time, tx_octets, tx_time);
            scan_state =ScanState::ServiceDiscovery;
            discover_services();
            break;
        }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
    {
        if (curr_scan.auth_downgrade_in_progress) {
            LOG_INFO("Auth requirements not met. Attempting downgrade");
            curr_scan.ha.conn_handle = HCI_CON_HANDLE_INVALID;
            scan_state = ScanState::Connecting;
            auto gap_res = gap_connect(curr_scan.ha.addr, curr_scan.ha.addr_type);
            if (gap_res != ERROR_CODE_SUCCESS) {
                curr_scan.auth_downgrade_in_progress = false;
                LOG_ERROR("gap_connect failed with error: 0x%02x", (unsigned int)gap_res);
                scan_state = ScanState::Scan;
                return;
            }
            return;
        }
        uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
        LOG_INFO("Received disconnection event.");
        // Expected disconnection, reenable scanning
        if (scan_state == ScanState::Disconnecting) {
            LOG_INFO("Expected disconnection");
        } else {
            LOG_ERROR("Disconnected with reason: 0x%02x", static_cast<unsigned int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        auto ha = ha_mgr.get_by_conn_handle(c);
        if (ha) {
            LOG_INFO("%s: Disconnected with %hu available credits.", ha.side_str, ha.avail_credits);
            ha_mgr.remove_by_conn_handle(c);
        }
        if (ha_mgr.hearing_aids.size() > 0) {
            led_mgr.set_led_pattern(one_connected);
        } else {
            led_mgr.set_led_pattern(none_connected);
        }
        scan_state = ScanState::Scan;
        curr_scan.reset();
        break;
    }
    default:
        // LOG_INFO("Unhandled HCI event: %hu", (uint16_t)hci_ev_type);
        break;
    }
}

/* Handler for security manager events */
static void sm_event_handler (uint8_t packet_type, 
                              [[maybe_unused]] uint16_t channel, 
                              uint8_t *packet, 
                              [[maybe_unused]] uint16_t size) {
    ASHA_ASSERT_PACKET_TYPE(HCI_EVENT_PACKET);
    auto ev_type = hci_event_packet_get_type(packet);
    if (scan_state == ScanState::IdentityResolving) {
        switch(ev_type) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
        {
            // If the full set is paired, only handle advertising
            // reports from devices where identity resolving succeeds
            if (runtime_settings.full_set_paired) {
                scan_state = ScanState::Scan;
            } else if (curr_scan.report.is_hearing_aid) {
                scan_state = ScanState::Connecting;
                LOG_INFO("HA discovered with addr %s. Connecting...", bd_addr_to_str(curr_scan.report.address));
                bd_addr_copy(curr_scan.ha.addr, curr_scan.report.address);
                auto gap_res = gap_connect(curr_scan.report.address, static_cast<bd_addr_type_t>(curr_scan.report.address_type));
                if (gap_res != ERROR_CODE_SUCCESS) {
                    LOG_ERROR("gap_connect failed with error: 0x%02x", (unsigned int)gap_res);
                    scan_state = ScanState::Scan;
                    return;
                }
            } else {
                LOG_SCAN("Ad Report for addr %s is not HA", bd_addr_to_str(curr_scan.report.address));
                scan_state = ScanState::Scan;
            }
            break;
        }
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            LOG_INFO("Identity resolving succeeded");
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.ha.addr);
            if (ha_mgr.get_by_addr(curr_scan.ha.addr)) {
                LOG_INFO("Device already connected.");
                scan_state = ScanState::Scan;
                return;
            }
            scan_state = ScanState::Connecting;
            curr_scan.ha.addr_type = static_cast<bd_addr_type_t>(sm_event_identity_resolving_succeeded_get_addr_type(packet));
            LOG_INFO("Connecting to address %s", bd_addr_to_str(curr_scan.ha.addr));
            auto gap_res = gap_connect(curr_scan.ha.addr, curr_scan.ha.addr_type);
            if (gap_res != ERROR_CODE_SUCCESS) {
                LOG_ERROR("gap_connect failed with error: 0x%02x", (unsigned int)gap_res);
                scan_state = ScanState::Scan;
                return;
            }
            break;
        }
        default: break;
        }
    } else {
        if (scan_state != ScanState::Pairing) return;
        switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            LOG_INFO("Just Works requested");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_PAIRING_STARTED:
            LOG_INFO("Pairing started");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)){
            case ERROR_CODE_SUCCESS:
                LOG_INFO("Pairing complete, success");
                // scan_state = ScanState::ServiceDiscovery;
                // discover_services();
                curr_scan.auth_downgrade_in_progress = false;
                scan_state = ScanState::DataLen;
                set_data_length();
                break;
            case ERROR_CODE_CONNECTION_TIMEOUT:
                LOG_ERROR("Pairing failed, timeout");
                scan_state = ScanState::Scan;
                break;
            case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                LOG_ERROR("Pairing failed, disconnected");
                scan_state = ScanState::Scan;
                break;
            case ERROR_CODE_AUTHENTICATION_FAILURE:
            {
                uint8_t reason = sm_event_pairing_complete_get_reason(packet);
                if (reason == SM_REASON_AUTHENTHICATION_REQUIREMENTS && !curr_scan.auth_downgrade_in_progress) {
                    // Try and downgrade from LE Secure connections
                    sm_set_authentication_requirements(authreq_bonding);
                    curr_scan.auth_downgrade_in_progress = true;
                } else {
                    curr_scan.auth_downgrade_in_progress = false;
                    LOG_ERROR("Pairing failed, auth failure with reason: 0x%02x", static_cast<int>(reason));
                }
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
                break;
            }
            default:
                break;
            }
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            LOG_INFO("Reencryption started");
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            switch (sm_event_reencryption_complete_get_status(packet)) {
            case ERROR_CODE_SUCCESS:
                LOG_INFO("Reencryption complete");
                // scan_state = ScanState::ServiceDiscovery;
                // discover_services();
                curr_scan.auth_downgrade_in_progress = false;
                scan_state = ScanState::DataLen;
                set_data_length();
                break;
            case ERROR_CODE_PIN_OR_KEY_MISSING:
            {   
                LOG_ERROR("Reencryption failed with ERROR_CODE_PIN_OR_KEY_MISSING");
                if (!curr_scan.auth_downgrade_in_progress) {
                    curr_scan.auth_downgrade_in_progress = true;
                    sm_set_authentication_requirements(authreq_bonding);
                } else {
                    curr_scan.auth_downgrade_in_progress = false;
                    bd_addr_t addr;
                    sm_event_reencryption_complete_get_address(packet, addr);
                    delete_paired_device(addr);
                    runtime_settings.set_full_set_paired(false);
                }
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
                break;
            }
            default:
                break;
            }
        default:
            break;
        }
    }
}

static void l2cap_cbm_event_handler (uint8_t packet_type, 
                                     [[maybe_unused]] uint16_t channel, 
                                     uint8_t *packet, 
                                     [[maybe_unused]] uint16_t size)
{
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case L2CAP_EVENT_CBM_CHANNEL_OPENED:
    {
        HA& ha = ha_mgr.get_by_conn_handle(l2cap_event_cbm_channel_opened_get_handle(packet));
        uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
        if (ha) {
            ha.on_l2cap_channel_created(status);
        }
        break;
    }
    case L2CAP_EVENT_PACKET_SENT:
    {
        LOG_AUDIO("L2CAP_EVENT_PACKET_SENT");
        HA& ha = ha_mgr.get_by_cid(l2cap_event_packet_sent_get_local_cid(packet));
        ha.on_audio_packet_sent();
        break;
    }
    }
}

/* Handler for reading GATT service and characteristic values
   and subscribing to the AudioStatusPoint characteristic notification 
   during the connection process.
   This function can be read in sequence for the order of events */
static void scan_gatt_event_handler ([[maybe_unused]] uint8_t packet_type, 
                                     [[maybe_unused]] uint16_t channel, 
                                     uint8_t *packet, 
                                     [[maybe_unused]] uint16_t size)
{
    switch (scan_state) {
    case ScanState::ServiceDiscovery:
    {
        switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            gatt_client_service_t service;
            gatt_event_service_query_result_get_service(packet, &service);
            LOG_SCAN("Service Query: Service ID: 0x%04hx : UUID: %s", service.uuid16, uuid128_to_str(service.uuid128));
            if (service.uuid16 == AshaUUID::service16 || uuid_eq(service.uuid128, AshaUUID::service)) {
                memcpy(&curr_scan.ha.asha_service.service, &service, sizeof(service));
                curr_scan.service_found = true;
                LOG_INFO("ASHA service found");
            } else if (service.uuid16 == GapUUID::service16) {
                LOG_INFO("GAP service found");
                memcpy(&curr_scan.ha.gap_service.service, &service, sizeof(service));
            } else if (service.uuid16 == DisUUID::service16) {
                LOG_INFO("DIS service found");
                memcpy(&curr_scan.ha.dis_service.service, &service, sizeof(service));
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            if (gatt_event_query_complete_get_att_status(packet) != ATT_ERROR_SUCCESS) {
                LOG_ERROR("Service discovery ATT error: 0x%02x", static_cast<unsigned int>(gatt_event_query_complete_get_att_status(packet)));
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
                return;
            }
            // Older hearing aids may support MFI but not ASHA
            if (!curr_scan.service_found) {
                LOG_INFO("ASHA service not found. Continuing scanning");
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
                break;
            }
            scan_state = ScanState::Finalizing;
            curr_scan.ha.state = HA::State::ServicesDiscovered;
            finalise_curr_discovery();
            break;
        default:
            break;
        };
    }
    default:
        break;
    }
}

static void connected_gatt_event_handler([[maybe_unused]] uint8_t packet_type, 
                                         [[maybe_unused]] uint16_t channel, 
                                         uint8_t *packet, 
                                         [[maybe_unused]] uint16_t size)
{
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_characteristic_query_result_get_handle(packet));
        if (ha) {
            ha.on_char_discovered(packet);
        }
        break;
    }
    case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_characteristic_value_query_result_get_handle(packet));
        if (ha) {
            ha.on_read_char_value(packet);
        }
        break;
    }
    case GATT_EVENT_NOTIFICATION:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_notification_get_handle(packet));
        if (ha) {
            uint16_t val_handle = gatt_event_notification_get_value_handle(packet);
            if (val_handle == ha.asha_service.asp.value_handle) {
                int8_t status_val = (int8_t)gatt_event_notification_get_value(packet)[0];
                ha.on_asp_notification(status_val);
            }
        }
        break;
    }
    case GATT_EVENT_QUERY_COMPLETE:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_query_complete_get_handle(packet));
        if (ha) {
            ha.on_gatt_event_query_complete(packet);
        }
        break;
    }
    default: 
        break;
    }
}

/* Once the Read Only Properties and PSM characteristics have been read,
   and the AudioStatusPoint notification subscribed to, determine what Device 
   to assign it to, and whether to continue searching for another aid. */
static void finalise_curr_discovery()
{
    // This shouldn't be the case, but best be sure.
    if (ha_mgr.set_complete()) {
        LOG_INFO("Already have complete set. Not adding current device.");
        scan_state = ScanState::Complete;
        return;
    }
    curr_scan.ha.l2cap_packet_handler = &l2cap_cbm_event_handler;
    curr_scan.ha.gatt_packet_handler = &connected_gatt_event_handler;
    auto& ha = ha_mgr.add(curr_scan.ha);
    curr_scan.reset();
    if (!ha) {
        LOG_ERROR("Error adding this device.");
        scan_state = ScanState::Scan;
        return;
    }
    led_mgr.set_led_pattern(one_connected);
    scan_state = ScanState::Scan;
}

static void delete_paired_devices()
{
    LOG_INFO("Removing paired devices");
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            LOG_INFO("Removing: %s", bd_addr_to_str(addr));
            le_device_db_remove(i);
        }
    }
}

static void delete_paired_device(const bd_addr_t address)
{
    LOG_INFO("Removing paired device with address %s", bd_addr_to_str(address));
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN && bd_addr_cmp(addr, address) == 0) {
            LOG_INFO("Found. Removing");
            le_device_db_remove(i);
        }
    }
}

} // namespace asha