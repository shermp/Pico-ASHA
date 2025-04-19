#pragma once

#include <cstdint>

namespace asha
{

constexpr const char* bt_err_str(uint8_t err)
{
    switch (err) {
        case 0x00u: return "ERROR_CODE_SUCCESS";
        case 0x01u: return "ERROR_CODE_UNKNOWN_HCI_COMMAND";
        case 0x02u: return "ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER";
        case 0x03u: return "ERROR_CODE_HARDWARE_FAILURE";
        case 0x04u: return "ERROR_CODE_PAGE_TIMEOUT";
        case 0x05u: return "ERROR_CODE_AUTHENTICATION_FAILURE";
        case 0x06u: return "ERROR_CODE_PIN_OR_KEY_MISSING";
        case 0x07u: return "ERROR_CODE_MEMORY_CAPACITY_EXCEEDED";
        case 0x08u: return "ERROR_CODE_CONNECTION_TIMEOUT";
        case 0x09u: return "ERROR_CODE_CONNECTION_LIMIT_EXCEEDED";
        case 0x0Au: return "ERROR_CODE_SYNCHRONOUS_CONNECTION_LIMIT_TO_A_DEVICE_EXCEEDED";
        case 0x0Bu: return "ERROR_CODE_ACL_CONNECTION_ALREADY_EXISTS";
        case 0x0Cu: return "ERROR_CODE_COMMAND_DISALLOWED";
        case 0x0Du: return "ERROR_CODE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES";
        case 0x0Eu: return "ERROR_CODE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS";
        case 0x0Fu: return "ERROR_CODE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR";
        case 0x10u: return "ERROR_CODE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED";
        case 0x11u: return "ERROR_CODE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE";
        case 0x12u: return "ERROR_CODE_INVALID_HCI_COMMAND_PARAMETERS";
        case 0x13u: return "ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION";
        case 0x14u: return "ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES";
        case 0x15u: return "ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF";
        case 0x16u: return "ERROR_CODE_CONNECTION_TERMINATED_BY_LOCAL_HOST";
        case 0x17u: return "ERROR_CODE_REPEATED_ATTEMPTS";
        case 0x18u: return "ERROR_CODE_PAIRING_NOT_ALLOWED";
        case 0x19u: return "ERROR_CODE_UNKNOWN_LMP_PDU";
        case 0x1Au: return "ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE_UNSUPPORTED_LMP_FEATURE";
        case 0x1Bu: return "ERROR_CODE_SCO_OFFSET_REJECTED";
        case 0x1Cu: return "ERROR_CODE_SCO_INTERVAL_REJECTED";
        case 0x1Du: return "ERROR_CODE_SCO_AIR_MODE_REJECTED";
        case 0x1Eu: return "ERROR_CODE_INVALID_LMP_PARAMETERS_INVALID_LL_PARAMETERS";
        case 0x1Fu: return "ERROR_CODE_UNSPECIFIED_ERROR";
        case 0x20u: return "ERROR_CODE_UNSUPPORTED_LMP_PARAMETER_VALUE_UNSUPPORTED_LL_PARAMETER_VALUE";
        case 0x21u: return "ERROR_CODE_ROLE_CHANGE_NOT_ALLOWED";
        case 0x22u: return "ERROR_CODE_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT";
        case 0x23u: return "ERROR_CODE_LMP_ERROR_TRANSACTION_COLLISION";
        case 0x24u: return "ERROR_CODE_LMP_PDU_NOT_ALLOWED";
        case 0x25u: return "ERROR_CODE_ENCRYPTION_MODE_NOT_ACCEPTABLE";
        case 0x26u: return "ERROR_CODE_LINK_KEY_CANNOT_BE_CHANGED";
        case 0x27u: return "ERROR_CODE_REQUESTED_QOS_NOT_SUPPORTED";
        case 0x28u: return "ERROR_CODE_INSTANT_PASSED";
        case 0x29u: return "ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED";
        case 0x2Au: return "ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION";
        case 0x2Bu: return "ERROR_CODE_RESERVED";
        case 0x2Cu: return "ERROR_CODE_QOS_UNACCEPTABLE_PARAMETER";
        case 0x2Du: return "ERROR_CODE_QOS_REJECTED";
        case 0x2Eu: return "ERROR_CODE_CHANNEL_CLASSIFICATION_NOT_SUPPORTED";
        case 0x2Fu: return "ERROR_CODE_INSUFFICIENT_SECURITY";
        case 0x30u: return "ERROR_CODE_PARAMETER_OUT_OF_MANDATORY_RANGE";
        case 0x32u: return "ERROR_CODE_ROLE_SWITCH_PENDING";
        case 0x34u: return "ERROR_CODE_RESERVED_SLOT_VIOLATION";
        case 0x35u: return "ERROR_CODE_ROLE_SWITCH_FAILED";
        case 0x36u: return "ERROR_CODE_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE";
        case 0x37u: return "ERROR_CODE_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST";
        case 0x38u: return "ERROR_CODE_HOST_BUSY_PAIRING";
        case 0x39u: return "ERROR_CODE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND";
        case 0x3Au: return "ERROR_CODE_CONTROLLER_BUSY";
        case 0x3Bu: return "ERROR_CODE_UNACCEPTABLE_CONNECTION_PARAMETERS";
        case 0x3Cu: return "ERROR_CODE_DIRECTED_ADVERTISING_TIMEOUT";
        case 0x3Du: return "ERROR_CODE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE";
        case 0x3Eu: return "ERROR_CODE_CONNECTION_FAILED_TO_BE_ESTABLISHED";
        case 0x3Fu: return "ERROR_CODE_MAC_CONNECTION_FAILED";
        case 0x40u: return "ERROR_CODE_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING";
        case 0x50: return "BTSTACK_CONNECTION_TO_BTDAEMON_FAILED";
        case 0x51: return "BTSTACK_ACTIVATION_FAILED_SYSTEM_BLUETOOTH";
        case 0x52: return "BTSTACK_ACTIVATION_POWERON_FAILED";
        case 0x53: return "BTSTACK_ACTIVATION_FAILED_UNKNOWN";
        case 0x54: return "BTSTACK_NOT_ACTIVATED";
        case 0x55: return "BTSTACK_BUSY";
        case 0x56: return "BTSTACK_MEMORY_ALLOC_FAILED";
        case 0x57: return "BTSTACK_ACL_BUFFERS_FULL";
        case 0x60: return "L2CAP_COMMAND_REJECT_REASON_COMMAND_NOT_UNDERSTOOD";
        case 0x61: return "L2CAP_COMMAND_REJECT_REASON_SIGNALING_MTU_EXCEEDED";
        case 0x62: return "L2CAP_COMMAND_REJECT_REASON_INVALID_CID_IN_REQUEST";
        case 0x63: return "L2CAP_CONNECTION_RESPONSE_RESULT_SUCCESSFUL";
        case 0x64: return "L2CAP_CONNECTION_RESPONSE_RESULT_PENDING";
        case 0x65: return "L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_PSM";
        case 0x66: return "L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_SECURITY";
        case 0x67: return "L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_RESOURCES";
        case 0x68: return "L2CAP_CONNECTION_RESPONSE_RESULT_ERTM_NOT_SUPPORTED";
        case 0x69: return "L2CAP_CONNECTION_RESPONSE_RESULT_RTX_TIMEOUT";
        case 0x6A: return "L2CAP_CONNECTION_BASEBAND_DISCONNECT";
        case 0x6B: return "L2CAP_SERVICE_ALREADY_REGISTERED";
        case 0x6C: return "L2CAP_DATA_LEN_EXCEEDS_REMOTE_MTU";
        case 0x6D: return "L2CAP_SERVICE_DOES_NOT_EXIST";
        case 0x6E: return "L2CAP_LOCAL_CID_DOES_NOT_EXIST";
        case 0x6F: return "L2CAP_CONNECTION_RESPONSE_UNKNOWN_ERROR";
        case 0x70: return "RFCOMM_MULTIPLEXER_STOPPED";
        case 0x71: return "RFCOMM_CHANNEL_ALREADY_REGISTERED";
        case 0x72: return "RFCOMM_NO_OUTGOING_CREDITS";
        case 0x73: return "RFCOMM_AGGREGATE_FLOW_OFF";
        case 0x74: return "RFCOMM_DATA_LEN_EXCEEDS_MTU";
        case 0x7F: return "HFP_REMOTE_REJECTS_AUDIO_CONNECTION";
        case 0x80: return "SDP_HANDLE_ALREADY_REGISTERED";
        case 0x81: return "SDP_QUERY_INCOMPLETE";
        case 0x82: return "SDP_SERVICE_NOT_FOUND";
        case 0x83: return "SDP_HANDLE_INVALID";
        case 0x84: return "SDP_QUERY_BUSY";
        case 0x90: return "ATT_HANDLE_VALUE_INDICATION_IN_PROGRESS";
        case 0x91: return "ATT_HANDLE_VALUE_INDICATION_TIMEOUT";
        case 0x92: return "ATT_HANDLE_VALUE_INDICATION_DISCONNECT";
        case 0x93: return "GATT_CLIENT_NOT_CONNECTED";
        case 0x94: return "GATT_CLIENT_BUSY";
        case 0x95: return "GATT_CLIENT_IN_WRONG_STATE";
        case 0x96: return "GATT_CLIENT_DIFFERENT_CONTEXT_FOR_ADDRESS_ALREADY_EXISTS";
        case 0x97: return "GATT_CLIENT_VALUE_TOO_LONG";
        case 0x98: return "GATT_CLIENT_CHARACTERISTIC_NOTIFICATION_NOT_SUPPORTED";
        case 0x99: return "GATT_CLIENT_CHARACTERISTIC_INDICATION_NOT_SUPPORTED";
        case 0xA0: return "BNEP_SERVICE_ALREADY_REGISTERED";
        case 0xA1: return "BNEP_CHANNEL_NOT_CONNECTED";
        case 0xA2: return "BNEP_DATA_LEN_EXCEEDS_MTU";
        case 0xB0: return "OBEX_UNKNOWN_ERROR";
        case 0xB1: return "OBEX_CONNECT_FAILED";
        case 0xB2: return "OBEX_DISCONNECTED";
        case 0xB3: return "OBEX_NOT_FOUND";
        case 0xB4: return "OBEX_NOT_ACCEPTABLE";
        case 0xB5: return "OBEX_ABORTED";
        case 0xD0: return "MESH_ERROR_APPKEY_INDEX_INVALID";
        default: return "unknown";
    }
}

constexpr const char* att_err_str(uint8_t err)
{
    switch (err) {
        case 0x00u: return "ATT_ERROR_SUCCESS";
        case 0x01u: return "ATT_ERROR_INVALID_HANDLE";
        case 0x02u: return "ATT_ERROR_READ_NOT_PERMITTED";
        case 0x03u: return "ATT_ERROR_WRITE_NOT_PERMITTED";
        case 0x04u: return "ATT_ERROR_INVALID_PDU";
        case 0x05u: return "ATT_ERROR_INSUFFICIENT_AUTHENTICATION";
        case 0x06u: return "ATT_ERROR_REQUEST_NOT_SUPPORTED";
        case 0x07u: return "ATT_ERROR_INVALID_OFFSET";
        case 0x08u: return "ATT_ERROR_INSUFFICIENT_AUTHORIZATION";
        case 0x09u: return "ATT_ERROR_PREPARE_QUEUE_FULL";
        case 0x0au: return "ATT_ERROR_ATTRIBUTE_NOT_FOUND";
        case 0x0bu: return "ATT_ERROR_ATTRIBUTE_NOT_LONG";
        case 0x0cu: return "ATT_ERROR_INSUFFICIENT_ENCRYPTION_KEY_SIZE";
        case 0x0du: return "ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH";
        case 0x0eu: return "ATT_ERROR_UNLIKELY_ERROR";
        case 0x0fu: return "ATT_ERROR_INSUFFICIENT_ENCRYPTION";
        case 0x10u: return "ATT_ERROR_UNSUPPORTED_GROUP_TYPE";
        case 0x11u: return "ATT_ERROR_INSUFFICIENT_RESOURCES";
        case 0x13u: return "ATT_ERROR_VALUE_NOT_ALLOWED";
        case 0x1fu: return "ATT_ERROR_HCI_DISCONNECT_RECEIVED";
        case 0x70u: return "ATT_ERROR_BONDING_INFORMATION_MISSING";
        case 0x7eu: return "ATT_ERROR_DATA_MISMATCH";
        case 0x7Fu: return "ATT_ERROR_TIMEOUT";
        case 0xFCu: return "ATT_ERROR_WRITE_REQUEST_REJECTED";
        case 0xFDu: return "ATT_ERROR_CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_IMPROPERLY_CONFIGURED";
        case 0xFEu: return "ATT_ERROR_PROCEDURE_ALREADY_IN_PROGRESS";
        case 0xFFu: return "ATT_ERROR_OUT_OF_RANGE";
        default: return "unknown";
    }
}

constexpr const char* l2cap_err_str(uint8_t err) {
    switch (err) {
        case 0x00: return "L2CAP_CBM_CONNECTION_RESULT_SUCCESS";
        case 0x02: return "L2CAP_CBM_CONNECTION_RESULT_SPSM_NOT_SUPPORTED";
        case 0x04: return "L2CAP_CBM_CONNECTION_RESULT_NO_RESOURCES_AVAILABLE";
        case 0x05: return "L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_AUTHENTICATION";
        case 0x06: return "L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_AUTHORIZATION";
        case 0x07: return "L2CAP_CBM_CONNECTION_RESULT_ENCYRPTION_KEY_SIZE_TOO_SHORT";
        case 0x08: return "L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_ENCRYPTION";
        case 0x09: return "L2CAP_CBM_CONNECTION_RESULT_INVALID_SOURCE_CID";
        case 0x0A: return "L2CAP_CBM_CONNECTION_RESULT_SOURCE_CID_ALREADY_ALLOCATED";
        case 0x0B: return "L2CAP_CBM_CONNECTION_RESULT_UNACCEPTABLE_PARAMETERS";
        default: return "unknown";
    }
}

constexpr const char* sm_reason_str(uint8_t err)
{
    switch (err) {
        case 0x00: return "SM_REASON_RESERVED";
        case 0x01: return "SM_REASON_PASSKEY_ENTRY_FAILED";
        case 0x02: return "SM_REASON_OOB_NOT_AVAILABLE";
        case 0x03: return "SM_REASON_AUTHENTHICATION_REQUIREMENTS";
        case 0x04: return "SM_REASON_CONFIRM_VALUE_FAILED";
        case 0x05: return "SM_REASON_PAIRING_NOT_SUPPORTED";
        case 0x06: return "SM_REASON_ENCRYPTION_KEY_SIZE";
        case 0x07: return "SM_REASON_COMMAND_NOT_SUPPORTED";
        case 0x08: return "SM_REASON_UNSPECIFIED_REASON";
        case 0x09: return "SM_REASON_REPEATED_ATTEMPTS";
        case 0x0a: return "SM_REASON_INVALID_PARAMETERS";
        case 0x0b: return "SM_REASON_DHKEY_CHECK_FAILED";
        case 0x0c: return "SM_REASON_NUMERIC_COMPARISON_FAILED";
        case 0x0d: return "SM_REASON_BR_EDR_PAIRING_IN_PROGRESS";
        case 0x0e: return "SM_REASON_CROSS_TRANSPORT_KEY_DERIVATION_NOT_ALLOWED";
        case 0x0f: return "SM_REASON_KEY_REJECTED";
        default: return "unknown";
    }
}

}