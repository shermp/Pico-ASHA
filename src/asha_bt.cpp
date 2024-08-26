#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include "asha_logging.h"
#include "asha_uuid.hpp"
#include "asha_bt.hpp"

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

static btstack_packet_callback_registration_t hci_event_cb_reg;
static btstack_packet_callback_registration_t sm_event_cb_reg;

static void hci_event_handler           (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_event_handler            (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_cbm_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void scan_gatt_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void connected_gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handle_bt_audio_pending_worker(async_context_t *context, async_when_pending_worker_t *worker);
static void set_data_length();
static void discover_services();
static void finalise_curr_discovery();
static bool device_db_empty();
static void delete_paired_devices();
static void delete_paired_device(const bd_addr_t addr);

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
    bd_addr_t address;
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
                    LOG_INFO("ASHA 16 bit service discovered");
                    is_hearing_aid = true;
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += sizeof(AshaUUID::service)) {
                reverse_128(adv_data + i, tmp_uuid128);
                if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                    LOG_INFO("ASHA 128 bit UUID service discovered");
                    is_hearing_aid = true;
                } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                    LOG_INFO("MFI UUID discovered");
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
}

static void handle_bt_audio_pending_worker(async_context_t *context, async_when_pending_worker_t *worker)
{
    // No need to do anything if no hearing aids are connected!
    using enum HA::State;
    if (ha_mgr.hearing_aids.empty()) return;
    
    uint32_t write_index = audio_buff.get_write_index();
    AudioBuffer::Volume vol = audio_buff.get_volume();
    for (auto& ha : ha_mgr.hearing_aids) {
        ha.avail_credits = l2cap_cbm_available_credits(ha.cid);
        if (ha.state == L2Connected) {
            /* Ensure sufficient credits are available to (re)start
               audio streaming */
            if (ha.avail_credits >= 8) {
                ha.write_acp_start();
            }
        } else if (ha.is_streaming_audio()) {
            audio_buff.encode_audio = true;

            if (write_index == 0) continue;

            ha.set_write_index(write_index);
            ha.set_volume(vol);
            ha.send_audio_packet();
        }
    }
}

extern "C" void bt_main()
{
#ifdef ASHA_USB_SERIAL
    // Allow time for USB serial to connect before proceeding
    sleep_ms(5000);
#endif

    LOG_INFO("BT ASHA starting.");
    if (cyw43_arch_init()) {
        LOG_ERROR("failed to initialise cyw43_arch");
        return;
    }

    /* Start init BTStack systems */
#ifdef ASHA_HCI_DUMP
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif
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
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);
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

    bt_audio_pending_worker.do_work = handle_bt_audio_pending_worker;
    async_context_t *ctx = cyw43_arch_async_context();
    async_context_add_when_pending_worker(ctx, &bt_audio_pending_worker);
    bt_async_ctx = ctx;

    /* Start BTStack */
    LOG_INFO("HCI power on.");
    hci_power_control(HCI_POWER_ON);

    int64_t timeout = 10'000'000LL;
    if (device_db_empty()) {
        LOG_INFO("Device DB is empty, allowing a 60s timeout");
        timeout = 60'000'000LL;
    }

    /* Allow hearing aids a timeout to complete
       connection before starting to stream audio.
       A longer timeout is allowed when the device
       db is empty */
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t curr_time;
    int64_t diff;
    do {
        sleep_us(500);
        curr_time = get_absolute_time();
        diff = absolute_time_diff_us(start_time, curr_time);
    } while (!ha_mgr.set_complete() && diff < timeout);

    // Flush the first audio packet
    /* Main audio streaming loop */
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
    HA cached = ha_mgr.get_from_cache(curr_scan.ha.addr);
    if (cached) {
        LOG_INFO("Hearing aid found in cache. Skipping discovery");
        cached.conn_handle = curr_scan.ha.conn_handle;
        curr_scan.ha = cached;
        scan_state = ScanState::Finalizing;
        finalise_curr_discovery();
    } else { 
        LOG_INFO("Device paired. Discovering ASHA service");
        auto res = gatt_client_discover_primary_services(&scan_gatt_event_handler, curr_scan.ha.conn_handle);
        //auto res = gatt_client_discover_primary_services_by_uuid128(&scan_gatt_event_handler, curr_scan.ha.conn_handle, AshaUUID::service);
        if (res != ERROR_CODE_SUCCESS) {
            LOG_ERROR("Could not register service query: %d", static_cast<int>(res));
            scan_state = ScanState::Disconnecting;
            gap_disconnect(curr_scan.ha.conn_handle);
        }
    }
}

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    ASHA_ASSERT_PACKET_TYPE(HCI_EVENT_PACKET);
    bd_addr_t local_addr;

    uint8_t hci_ev_type = hci_event_packet_get_type(packet);

    switch(hci_ev_type) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            // Set 2M PHY
            gap_set_connection_phys(2);
            // Set connection parameters including connection interval
            // by default. Values taken from Android
            gap_set_connection_parameters(0x0030, 0x0030, asha_conn_interval, asha_conn_interval, asha_conn_latency, 100, 12, 12);
            gap_local_bd_addr(local_addr);
            LOG_INFO("BTstack up and running on %s", bd_addr_to_str(local_addr));
#ifdef ASHA_DELETE_PAIRINGS
            delete_paired_devices();
#endif
            start_scan();
        } else {
            scan_state = ScanState::Stop;
        }
        break;
    /* Every time a GAP advertising report is received, handle it here */
    case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
        LOG_SCAN("Got extended advertising report.");
        // Fallthrough
    case GAP_EVENT_ADVERTISING_REPORT:
        if (scan_state != ScanState::Scan) return;
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
                LOG_ERROR("Unexpected connection params. Got Connection Interval: %hu  Connection latency: %hu", ci, cl);
            }
            //curr_scan.ha.supervision_timeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
            scan_state = ScanState::Pairing;
            LOG_INFO("Device connected. Attempt pairing");
            sm_request_pairing(curr_scan.ha.conn_handle);
            break;
        }
        }
    case HCI_EVENT_LE_META:
        switch(hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE:
        {
            if (scan_state != ScanState::DataLen) return;
            auto rx_octets = hci_subevent_le_data_length_change_get_max_rx_octets(packet);
            auto rx_time = hci_subevent_le_data_length_change_get_max_rx_time(packet);
            auto tx_octets = hci_subevent_le_data_length_change_get_max_tx_octets(packet);
            auto tx_time = hci_subevent_le_data_length_change_get_max_tx_time(packet);
            LOG_INFO("Date length set to: RX Octets: %hu, RX Time: %hu us, TX Octets: %hu, TX Time: %hu us",
                     rx_octets, rx_time, tx_octets, tx_time);
            scan_state =ScanState::ServiceDiscovery;
            discover_services();
            break;
        }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
    {
        uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
        LOG_INFO("Received disconnection event.");
        // Expected disconnection, reenable scanning
        if (scan_state == ScanState::Disconnecting) {
            LOG_INFO("Expected disconnection");
        } else {
            LOG_ERROR("Disconnected with reason: %d", static_cast<int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        auto ha = ha_mgr.get_by_conn_handle(c);
        if (ha) {
            LOG_INFO("%s device disconnected with %hu available credits.", (ha.side() == HA::Side::Left) ? "Left" : "Right", ha.avail_credits);
            ha_mgr.remove_by_conn_handle(c);
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
static void sm_event_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    ASHA_ASSERT_PACKET_TYPE(HCI_EVENT_PACKET);
    auto ev_type = hci_event_packet_get_type(packet);
    if (scan_state == ScanState::IdentityResolving) {
        switch(ev_type) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            if (curr_scan.report.is_hearing_aid) {
                scan_state = ScanState::Connecting;
                LOG_INFO("Hearing aid discovered with addr %s. Connecting...", bd_addr_to_str(curr_scan.report.address));
                bd_addr_copy(curr_scan.ha.addr, curr_scan.report.address);
                gap_connect(curr_scan.report.address, static_cast<bd_addr_type_t>(curr_scan.report.address_type));
            } else {
                LOG_SCAN("Ad Report for addr %s is not hearing aid", bd_addr_to_str(curr_scan.report.address));
                scan_state = ScanState::Scan;
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            LOG_INFO("Identity resolving succeeded");
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.ha.addr);
            if (ha_mgr.get_by_addr(curr_scan.ha.addr)) {
                LOG_INFO("Device already connected.");
                return;
            }
            scan_state = ScanState::Connecting;
            auto addr_type = sm_event_identity_resolving_succeeded_get_addr_type(packet);
            LOG_INFO("Connecting to address %s", bd_addr_to_str(curr_scan.ha.addr));
            gap_connect(curr_scan.ha.addr, static_cast<bd_addr_type_t>(addr_type));
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
                LOG_ERROR("Pairing failed, authentication failure with reason: %d",
                        static_cast<int>(sm_event_pairing_complete_get_reason(packet)));
                // Try and upgrade to LE Secure connections
                sm_set_authentication_requirements(SM_AUTHREQ_BONDING | SM_AUTHREQ_SECURE_CONNECTION);
                scan_state = ScanState::Scan;
                break;
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
                scan_state = ScanState::DataLen;
                set_data_length();
                break;
            case ERROR_CODE_PIN_OR_KEY_MISSING:
            {   
                LOG_ERROR("Reencryption failed with ERROR_CODE_PIN_OR_KEY_MISSING");
                bd_addr_t addr;
                sm_event_reencryption_complete_get_address(packet, addr);
                delete_paired_device(addr);
                scan_state = ScanState::Scan;
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

static void l2cap_cbm_event_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
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
static void scan_gatt_event_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
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
                memcpy(&curr_scan.ha.service, &service, sizeof(service));
                curr_scan.service_found = true;
                LOG_INFO("ASHA service found");
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
        {
            // Older hearing aids may support MFI but not ASHA
            if (!curr_scan.service_found) {
                LOG_INFO("ASHA service not found. Continuing scanning");
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
                break;
            }
            scan_state = ScanState::CharDiscovery;
            // Service found. Discover characteristics
            auto res = gatt_client_discover_characteristics_for_service(
                    &scan_gatt_event_handler,
                    curr_scan.ha.conn_handle,
                    &curr_scan.ha.service
            );
            if (res != ERROR_CODE_SUCCESS) {
                LOG_ERROR("Could not register characteristics query: %d", static_cast<int>(res));
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
            }
            break;
        }
        }
        break;
    }
    case ScanState::CharDiscovery:
    {
        gatt_client_characteristic_t characteristic;
        switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            if (uuid_eq(characteristic.uuid128, AshaUUID::readOnlyProps)) {
                LOG_INFO("Got ROP Characteristic");
                curr_scan.ha.chars.rop = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioControlPoint)) {
                LOG_INFO("Got ACP Characteristic");
                curr_scan.ha.chars.acp = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioStatus)) {
                LOG_INFO("Got AUS Characteristic");
                curr_scan.ha.chars.asp = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::volume)) {
                LOG_INFO("Got VOL Characteristic");
                curr_scan.ha.chars.vol = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::psm)) {
                LOG_INFO("Got PSM Characteristic");
                curr_scan.ha.chars.psm = characteristic;
            }
            LOG_INFO("Characteristic handles: Start: 0x%04hx  Value: 0x%04hx  End: 0x%04hx",
                     characteristic.start_handle, characteristic.value_handle, characteristic.end_handle);
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("ASHA characteristic discovery complete");
            // Start reading the Read Only Properties characteristic
            scan_state = ScanState::ReadROP;
            gatt_client_read_value_of_characteristic(
                &scan_gatt_event_handler, 
                curr_scan.ha.conn_handle, 
                &curr_scan.ha.chars.rop
            );
            break;
        }
        break;
    }
    case ScanState::ReadROP:
    {
        switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
            LOG_INFO("Getting ReadOnlyProperties value");
            curr_scan.ha.rop.read(gatt_event_characteristic_value_query_result_get_value(packet));
            //curr_scan.device.read_only_props.dump_values();
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("Completed value read of ReadOnlyProperties");
            /* Next get the PSM value */
            scan_state = ScanState::ReadPSM;
            gatt_client_read_value_of_characteristic(
                &scan_gatt_event_handler,
                curr_scan.ha.conn_handle,
                &curr_scan.ha.chars.psm
            );
            break;
        }
        break;
    }
    case ScanState::ReadPSM:
    {
        switch (hci_event_packet_get_type(packet)) {
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
            LOG_INFO("Getting PSM value");
            curr_scan.ha.psm = gatt_event_characteristic_value_query_result_get_value(packet)[0];
            LOG_INFO("PSM: %d", static_cast<int>(curr_scan.ha.psm));
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("Completed value read of PSM");
            curr_scan.ha.rop.print_values();
            scan_state = ScanState::Finalizing;
            finalise_curr_discovery();
            break;
        }
        break;
    }
    }
}

static void connected_gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    switch (hci_event_packet_get_type(packet)) {
    case GATT_EVENT_NOTIFICATION:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_notification_get_handle(packet));
        if (ha) {
            int8_t status_val = (int8_t)gatt_event_notification_get_value(packet)[0];
            ha.on_asp_notification(status_val);
        }
        break;
    }
    case GATT_EVENT_QUERY_COMPLETE:
    {
        HA& ha = ha_mgr.get_by_conn_handle(gatt_event_query_complete_get_handle(packet));
        if (ha) {
            ha.on_gatt_event_query_complete(gatt_event_query_complete_get_att_status(packet));
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
    curr_scan.ha.state = HA::State::GATTConnected;
    curr_scan.ha.l2cap_packet_handler = &l2cap_cbm_event_handler;
    curr_scan.ha.gatt_packet_handler = &connected_gatt_event_handler;
    auto& ha = ha_mgr.add(curr_scan.ha);
    if (!ha) {
        LOG_ERROR("Error adding this device.");
        scan_state = ScanState::Scan;
        return;
    }
    ha.on_gatt_connected();
    if (ha_mgr.set_complete()) {
        LOG_INFO("Connected to all aid(s) in set.");
        scan_state = ScanState::Complete;
    } else {
        scan_state = ScanState::Scan;
    }
}

static bool device_db_empty()
{
    int addr_type;
    bd_addr_t addr;
    sm_key_t sm_key;
    int max_db = le_device_db_max_count();
    for (int i = 0; i < max_db; ++i) {
        le_device_db_info(i, &addr_type, addr, sm_key);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            return false;
        }
    }
    return true;
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