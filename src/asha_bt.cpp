#include "pico/cyw43_arch.h"
#include "pico/time.h"
#ifdef ASHA_HCI_DUMP
    #include "hci_dump_embedded_stdout.h"
#endif

#include "asha_logging.h"
#include "asha_uuid.hpp"
#include "asha_bt.hpp"

#include "util.hpp"

namespace asha {

#define ASHA_ASSERT_PACKET_TYPE(pt) if (packet_type != (pt)) return

static btstack_packet_callback_registration_t hci_event_cb_reg;
static btstack_packet_callback_registration_t sm_event_cb_reg;

static void hci_event_handler           (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_event_handler            (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void l2cap_cbm_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void scan_gatt_event_handler     (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void connected_gatt_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void finalise_curr_discovery();
static bool device_db_empty();
static void delete_paired_devices();

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
                    LOG_INFO("ASHA 16 bit service discovered\n");
                    is_hearing_aid = true;
                }
            }
            break;
        case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
        case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            for (i = 0; i < size; i += sizeof(AshaUUID::service)) {
                reverse_128(adv_data + i, tmp_uuid128);
                if (uuid_eq(tmp_uuid128, AshaUUID::service)) {
                    LOG_INFO("ASHA 128 bit UUID service discovered\n");
                    is_hearing_aid = true;
                } else if (uuid_eq(tmp_uuid128, mfiUUID)) {
                    LOG_INFO("MFI UUID discovered\n");
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

extern "C" void bt_main()
{
    LOG_INFO("BT ASHA starting.\n");
    if (cyw43_arch_init()) {
        LOG_ERROR("failed to initialise cyw43_arch\n");
        return;
    }

    /* Start init BTStack systems */
#ifdef ASHA_HCI_DUMP
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
#endif
    /* L2CAP init required for basic btstack functionality */
    LOG_INFO("L2CAP Init.\n");
    l2cap_init();
    l2cap_set_max_le_mtu(buff_size_sdu);

    /* Init the security manager. This takes care of pairing
       with hearing aids, and also allows reconnecting to already
       paired hearing aids */
    LOG_INFO("SM Init.\n");
    sm_init();
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_YES_NO);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING);

    /* Init GATT client. Required to read/write GATT characteristics */
    LOG_INFO("GATT Client Init.\n");
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
    LOG_INFO("HCI power on.\n");
    hci_power_control(HCI_POWER_ON);

    int64_t timeout = 10'000'000LL;
    if (device_db_empty()) {
        LOG_INFO("Device DB is empty, allowing a 60s timeout\n");
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

    uint8_t seq_num = 0u;
    uint32_t audio_write_index = 0u;
    uint32_t pre_buff = 8u;
    uint32_t min_write_index = pre_buff;
    //uint32_t audio_read_index = 0u;
    using enum HA::State;
    /* Main audio streaming loop */
    while (1) {
        // No need to do anything if no hearing aids are connected!
        if (ha_mgr.hearing_aids.size() == 0) continue;

        // Wait for USB audio streaming to start
        if (!pcm_streaming.Load()) continue;

        AudioBuffer::Volume vol = audio_buff.get_volume();
        audio_write_index = audio_buff.get_write_index();
        
        // If required, update connection params
        // and create l2cap channel
        for (auto& ha : ha_mgr.hearing_aids) {
            ha.set_volume(vol);
            switch (ha.state) {
            case GATTConnected:
                ha.update_conn_params();
                break;
            case UpdateConnParamsCompleted:
                ha.create_l2cap_channel();
                break;
            case L2ConnCompleted:
                ha.write_acp_cmd(HA::ACPOpCode::Start);
                break;
            case ASPStartOk:
                encode_audio = true;
                ha.audio_index = audio_write_index > 0 ? audio_write_index - 1 : 0;
                //ha.audio_index = 0;
                ha.state = AudioPacketReady;
                // fallthrough
            case AudioPacketReady:
                //LOG_INFO("Audio index: %hu\n", ha.audio_index);
                if (ha.audio_index < audio_write_index) {
                    ha.send_audio_packet(audio_buff.get_g_buff(ha.audio_index));
                }
                break;
            default:
                break;
            }
        }
    }
}

/* Start scanning for suitable peripherals */
static void start_scan()
{
    LOG_INFO("Start scanning.\n");
    scan_state = ScanState::Scan;
    gap_set_scan_params(1, 0x0030, 0x0030, 0);
    gap_start_scan();
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
            // Set defaults, except min_ce_length & max_ce_length
            gap_set_connection_parameters(0x0030, 0x0030, 8, 24, 4, 72, 12, 12);
            gap_local_bd_addr(local_addr);
            LOG_INFO("BTstack up and running on %s\n", bd_addr_to_str(local_addr));
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
        LOG_SCAN("Got Ad Report from %s\n", bd_addr_to_str(curr_scan.report.address));
        LOG_SCAN("RSSI: %d\n", (int)((int8_t)curr_scan.report.rssi));
        scan_state = ScanState::IdentityResolving;
        sm_address_resolution_lookup(curr_scan.report.address_type, curr_scan.report.address);
        break;

    case HCI_EVENT_LE_META:
        switch(hci_event_le_meta_get_subevent_code(packet)) {
        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
        {
            if (scan_state != ScanState::Connecting) return;
            curr_scan.ha.conn_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            curr_scan.ha.supervision_timeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
            scan_state = ScanState::ServiceDiscovery;
            LOG_INFO("Device connected. Discovering ASHA service\n");
            auto res = gatt_client_discover_primary_services(&scan_gatt_event_handler, curr_scan.ha.conn_handle);
            //auto res = gatt_client_discover_primary_services_by_uuid128(&scan_gatt_event_handler, curr_scan.ha.conn_handle, AshaUUID::service);
            if (res != ERROR_CODE_SUCCESS) {
                LOG_ERROR("Could not register service query: %d\n", static_cast<int>(res));
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
            }
            break;
        }
        case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
        {
            HA& ha = ha_mgr.get_by_conn_handle(hci_subevent_le_connection_complete_get_connection_handle(packet));
            if (ha) {
                ha.on_conn_param_updated();
            }
            break;
        }
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
    {
        uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
        LOG_INFO("Received disconnection event.\n");
        // Expected disconnection, reenable scanning
        if (scan_state == ScanState::Disconnecting) {
            LOG_INFO("Expected disconnection\n");
        } else {
            LOG_ERROR("Disconnected with reason: %d\n", static_cast<int>(reason));
        }
        auto c = hci_event_disconnection_complete_get_connection_handle(packet);
        auto ha = ha_mgr.get_by_conn_handle(c);
        if (ha) {
            LOG_INFO("%s device disconnected.\n", (ha.side() == HA::Side::Left) ? "Left" : "Right");
            ha_mgr.remove_by_conn_handle(c);
        }
        scan_state = ScanState::Scan;
        curr_scan.reset();
        break;
    }
    default:
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
                LOG_INFO("Hearing aid discovered with addr %s. Connecting...\n", bd_addr_to_str(curr_scan.report.address));
                bd_addr_copy(curr_scan.ha.addr, curr_scan.report.address);
                gap_connect(curr_scan.report.address, static_cast<bd_addr_type_t>(curr_scan.report.address_type));
            } else {
                LOG_SCAN("Ad Report for addr %s is not hearing aid\n", bd_addr_to_str(curr_scan.report.address));
                scan_state = ScanState::Scan;
            }
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        {
            LOG_INFO("Identity resolving succeeded\n");
            sm_event_identity_resolving_succeeded_get_address(packet, curr_scan.ha.addr);
            if (ha_mgr.get_by_addr(curr_scan.ha.addr)) {
                LOG_INFO("Device already connected.\n");
                return;
            }
            scan_state = ScanState::Connecting;
            auto addr_type = sm_event_identity_resolving_succeeded_get_addr_type(packet);
            LOG_INFO("Connecting to address %s\n", bd_addr_to_str(curr_scan.ha.addr));
            gap_connect(curr_scan.ha.addr, static_cast<bd_addr_type_t>(addr_type));
            break;
        }
        default: break;
        }
    } else {
        switch (ev_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            LOG_INFO("Just Works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_PAIRING_STARTED:
            LOG_INFO("Pairing started\n");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            switch (sm_event_pairing_complete_get_status(packet)){
            case ERROR_CODE_SUCCESS:
                LOG_INFO("Pairing complete, success\n");
                break;
            case ERROR_CODE_CONNECTION_TIMEOUT:
                LOG_ERROR("Pairing failed, timeout\n");
                break;
            case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                LOG_ERROR("Pairing failed, disconnected\n");
                break;
            case ERROR_CODE_AUTHENTICATION_FAILURE:
                LOG_ERROR("Pairing failed, authentication failure with reason: %d\n",
                        static_cast<int>(sm_event_pairing_complete_get_reason(packet)));
                break;
            default:
                break;
            }
            break;
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
    case L2CAP_EVENT_CAN_SEND_NOW:
    {
        LOG_AUDIO("L2CAP_EVENT_CAN_SEND_NOW\n");
        HA& ha = ha_mgr.get_by_cid(l2cap_event_can_send_now_get_local_cid(packet));
        ha.on_can_send_audio_packet_now();
    }
    case L2CAP_EVENT_PACKET_SENT:
    {
        LOG_AUDIO("L2CAP_EVENT_PACKET_SENT\n");
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
            LOG_SCAN("Service Query: Service ID: 0x%04hx : UUID: %s\n", service.uuid16, uuid128_to_str(service.uuid128));
            if (service.uuid16 == AshaUUID::service16 || uuid_eq(service.uuid128, AshaUUID::service)) {
                memcpy(&curr_scan.ha.service, &service, sizeof(service));
                curr_scan.service_found = true;
                LOG_INFO("ASHA service found\n");
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
        {
            // Older hearing aids may support MFI but not ASHA
            if (!curr_scan.service_found) {
                LOG_INFO("ASHA service not found. Continuing scanning\n");
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
                LOG_ERROR("Could not register characteristics query: %d\n", static_cast<int>(res));
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
                LOG_INFO("Got ROP Characteristic\n");
                curr_scan.ha.chars.rop = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioControlPoint)) {
                LOG_INFO("Got ACP Characteristic\n");
                curr_scan.ha.chars.acp = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::audioStatus)) {
                LOG_INFO("Got AUS Characteristic\n");
                curr_scan.ha.chars.asp = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::volume)) {
                LOG_INFO("Got VOL Characteristic\n");
                curr_scan.ha.chars.vol = characteristic;
            } else if (uuid_eq(characteristic.uuid128, AshaUUID::psm)) {
                LOG_INFO("Got PSM Characteristic\n");
                curr_scan.ha.chars.psm = characteristic;
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("ASHA characteristic discovery complete\n");
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
            LOG_INFO("Getting ReadOnlyProperties value\n");
            curr_scan.ha.rop.read(gatt_event_characteristic_value_query_result_get_value(packet));
            //curr_scan.device.read_only_props.dump_values();
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("Completed value read of ReadOnlyProperties\n");
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
            LOG_INFO("Getting PSM value\n");
            curr_scan.ha.psm = gatt_event_characteristic_value_query_result_get_value(packet)[0];
            LOG_INFO("PSM: %d\n", static_cast<int>(curr_scan.ha.psm));
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            LOG_INFO("Completed value read of PSM\n");
            curr_scan.ha.rop.print_values();
            scan_state = ScanState::SubscribeASPNotification;
            gatt_client_write_client_characteristic_configuration(
                &scan_gatt_event_handler,
                curr_scan.ha.conn_handle,
                &curr_scan.ha.chars.asp,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
            );
            break;
        }
        break;
    }
    case ScanState::SubscribeASPNotification:
    {
        switch(hci_event_packet_get_type(packet)) {
        case GATT_EVENT_QUERY_COMPLETE: 
        {
            auto att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                LOG_ERROR("Enabling AudioStatusPoint notifications failed with error code: 0x%02x\n"
                    "Disconnecting\n", att_status);
                scan_state = ScanState::Disconnecting;
                gap_disconnect(curr_scan.ha.conn_handle);
            }
            LOG_INFO("AudioStatusPoint notification enabled.\n");
            scan_state = ScanState::Finalizing;
            finalise_curr_discovery();
            break;
        }
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
        LOG_INFO("Already have complete set. Not adding current device.\n");
        scan_state = ScanState::Complete;
        return;
    }
    curr_scan.ha.state = HA::State::GATTConnected;
    curr_scan.ha.l2cap_packet_handler = &l2cap_cbm_event_handler;
    curr_scan.ha.gatt_packet_handler = &connected_gatt_event_handler;
    auto& ha = ha_mgr.add(curr_scan.ha);
    if (!ha) {
        LOG_ERROR("Error adding this device.\n");
        scan_state = ScanState::Scan;
        return;
    }
    if (ha_mgr.set_complete()) {
        LOG_INFO("Connected to all aid(s) in set.\n");
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
    LOG_INFO("Removing paired devices\n");
    int addr_type;
    bd_addr_t addr; 
    sm_key_t irk;
    int max_count = le_device_db_max_count();
    for (int i = 0; i < max_count; ++i) {
        le_device_db_info(i, &addr_type, addr, irk);
        if (addr_type != BD_ADDR_TYPE_UNKNOWN) {
            LOG_INFO("Removing: %s\n", bd_addr_to_str(addr));
            le_device_db_remove(i);
        }
    }
}

} // namespace asha