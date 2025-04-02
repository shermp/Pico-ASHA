#include <ArduinoJson.h>
#include <btstack.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio_uart.h>
#include <pico/stdio_usb.h>

#include <hardware/watchdog.h>

#include "asha_bt.hpp"
#include "asha_comms.hpp"
#include "asha_usb_serial.hpp"
#include "asha_uuid.hpp"
#include "hearing_aid.hpp"
#include "util.hpp"

namespace asha
{
/* Connection parameters for ASHA
   Note, connection interval is in units of 1.25ms */
constexpr uint16_t asha_conn_interval = 20 / 1.25f;

constexpr uint16_t asha_conn_latency  = 10;

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


static constexpr size_t json_resp_str_size = 4096;

static etl::string<json_resp_str_size> response_json = {};

static void hci_event_handler(PACKET_HANDLER_PARAMS);

constexpr btstack_time_t ha_process_interval_ms = 2;
static btstack_timer_source_t process_timer = {};
static void process_timer_handler(btstack_timer_source_t * timer);

constexpr btstack_time_t ha_audio_interval_ms = 1;
static btstack_timer_source_t audio_timer = {};
static void audio_timer_handler(btstack_timer_source_t * timer);

static void handle_stdin_line_worker(async_context_t *context, async_when_pending_worker_t *worker);
static void delete_paired_devices();
static void add_bonded_to_fal();

static void delete_paired_devices()
{
    using namespace comm;
    //LOG_INFO("Removing paired devices");
    EventPacket ev_pkt(EventType::DeletePair);

    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            //LOG_INFO("Removing: %s", bd_addr_to_str(addr));
            le_device_db_remove(i);
            bd_addr_copy(ev_pkt.data.addr, addr);
            add_event_to_buffer(unset_conn_id, ev_pkt);
        }
    }
}

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
    runtime_settings.get_settings();

    stdio_set_driver_enabled(&stdio_uart, runtime_settings.serial_uart_enabled);

    async_context_t *ctx = cyw43_arch_async_context();

    stdin_pending_worker.do_work = handle_stdin_line_worker;
    async_context_add_when_pending_worker(ctx, &stdin_pending_worker);
    usb_ser_ctx = ctx;

    led_mgr.set_ctx(ctx);

    if (runtime_settings.hci_dump_enabled) {
        // Allow time for USB serial to connect before proceeding
        while (!stdio_usb_connected()) {
            sleep_ms(250);
        }
        sleep_ms(250);
    }

    if (!runtime_settings) {
        //LOG_ERROR("Runtime settings not initialised");
        add_event_to_buffer(unset_conn_id, EventPacket(EventType::PicoASHAInit, StatusType::PAStatus, PAError::PARuntimeSettingsErr));
    }
    //LOG_INFO("BT ASHA starting.");
    
    /* Start init BTStack systems */
    // if (runtime_settings.hci_dump_enabled) {
    //     hci_dump_init(hci_dump_embedded_stdout_get_instance());
    // }
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

static void handle_stdin_line_worker([[maybe_unused]] async_context_t *context, 
                                     [[maybe_unused]] async_when_pending_worker_t *worker)
{
    JsonDocument cmd_doc;
    JsonDocument resp_doc;
    auto err = deserializeJson(cmd_doc, complete_std_line);
    if (err != DeserializationError::Ok) {
        // Dump the log if the user started monitoring USB serial late
        if (err == DeserializationError::EmptyInput) {
            // if (logging_ctx) {
            //     async_context_set_work_pending(logging_ctx, &logging_pending_worker);
            // }
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
        resp_doc["num_conn"] = HearingAid::num_connected();
        resp_doc["full_set"] = HearingAid::full_set_connected();
        JsonObject settings = resp_doc["settings"].to<JsonObject>();
        settings["log_level"] = log_level_to_str(runtime_settings.log_level);
        settings["hci_dump_enabled"] = runtime_settings.hci_dump_enabled;
        settings["uart_enabled"] = runtime_settings.serial_uart_enabled;
        settings["full_set_paired"] = runtime_settings.full_set_paired;

    } else if (cmd_is(SerCmd::ConnDevices)) {
        JsonArray devices = resp_doc["devices"].to<JsonArray>();
        for (auto ha : HearingAid::connected_has()) {
            if (!ha) { continue; }
            JsonObject dev = devices.add<JsonObject>();
            dev["addr"] = bd_addr_to_str(ha->addr);
            dev["name"] = ha->device_name;
            dev["side"] = ha->side_str;
            dev["mono"] = ha->rop.mode == Mode::Mono;
            dev["streaming"] = ha->audio_state != HearingAid::AudioState::AudioUnset && ha->audio_state != HearingAid::AudioState::Stop;
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
            //LOG_INFO("Enabling Watchdog");
            watchdog_enable(250, true);
        }
        resp_doc["success"] = true;
    }
    else {
        resp_doc["cmd"] = "unknown";
    }
    size_t len = serializeJson(resp_doc, response_json.data(), response_json.capacity() - 1);
    response_json.uninitialized_resize(len);
    printf("%s\r\n", response_json.c_str());
}

static void __not_in_flash_func(process_timer_handler)(btstack_timer_source_t* timer)
{
    btstack_run_loop_set_timer(timer, ha_process_interval_ms);
    btstack_run_loop_add_timer(timer);

    HearingAid::process();
    comm::try_send_events();
}

static void __not_in_flash_func(audio_timer_handler)(btstack_timer_source_t* timer)
{
    btstack_run_loop_set_timer(timer, ha_audio_interval_ms);
    btstack_run_loop_add_timer(timer);

    HearingAid::process_audio();
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
                if (!runtime_settings.full_set_paired) {
                    delete_paired_devices();
                }
                // Set the process timer handler
                btstack_run_loop_set_timer_handler(&process_timer, &process_timer_handler);
                btstack_run_loop_set_timer(&process_timer, ha_process_interval_ms);
                btstack_run_loop_add_timer(&process_timer);

                // Set the audio timer handler
                btstack_run_loop_set_timer_handler(&audio_timer, &audio_timer_handler);
                btstack_run_loop_set_timer(&audio_timer, ha_audio_interval_ms);
                btstack_run_loop_add_timer(&audio_timer);

                HearingAid::start_scan();
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
                if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                    //LOG_SCAN("ASHA 128 bit UUID service discovered");
                    is_hearing_aid = true;
                } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                    //LOG_SCAN("MFI UUID discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        default:
            break;
        }
    }
}

} // namespace asha