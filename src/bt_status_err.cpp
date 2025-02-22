#include <bluetooth.h>

#include "bt_status_err.hpp"

#define ERR_CASE(err) case err: return #err

namespace asha
{

const char* bt_err_str(uint8_t err)
{
    switch (err) {
        ERR_CASE(ERROR_CODE_SUCCESS);
        ERR_CASE(ERROR_CODE_UNKNOWN_HCI_COMMAND);
        ERR_CASE(ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER);
        ERR_CASE(ERROR_CODE_HARDWARE_FAILURE);
        ERR_CASE(ERROR_CODE_PAGE_TIMEOUT);
        ERR_CASE(ERROR_CODE_AUTHENTICATION_FAILURE);
        ERR_CASE(ERROR_CODE_PIN_OR_KEY_MISSING);
        ERR_CASE(ERROR_CODE_MEMORY_CAPACITY_EXCEEDED);
        ERR_CASE(ERROR_CODE_CONNECTION_TIMEOUT);
        ERR_CASE(ERROR_CODE_CONNECTION_LIMIT_EXCEEDED);
        ERR_CASE(ERROR_CODE_SYNCHRONOUS_CONNECTION_LIMIT_TO_A_DEVICE_EXCEEDED);
        ERR_CASE(ERROR_CODE_ACL_CONNECTION_ALREADY_EXISTS);
        ERR_CASE(ERROR_CODE_COMMAND_DISALLOWED);
        ERR_CASE(ERROR_CODE_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES);
        ERR_CASE(ERROR_CODE_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS);
        ERR_CASE(ERROR_CODE_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR);
        ERR_CASE(ERROR_CODE_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED);
        ERR_CASE(ERROR_CODE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE);
        ERR_CASE(ERROR_CODE_INVALID_HCI_COMMAND_PARAMETERS);
        ERR_CASE(ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION);
        ERR_CASE(ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES);
        ERR_CASE(ERROR_CODE_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF);
        ERR_CASE(ERROR_CODE_CONNECTION_TERMINATED_BY_LOCAL_HOST);
        ERR_CASE(ERROR_CODE_REPEATED_ATTEMPTS);
        ERR_CASE(ERROR_CODE_PAIRING_NOT_ALLOWED);
        ERR_CASE(ERROR_CODE_UNKNOWN_LMP_PDU);
        ERR_CASE(ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE_UNSUPPORTED_LMP_FEATURE);
        ERR_CASE(ERROR_CODE_SCO_OFFSET_REJECTED);
        ERR_CASE(ERROR_CODE_SCO_INTERVAL_REJECTED);
        ERR_CASE(ERROR_CODE_SCO_AIR_MODE_REJECTED);
        ERR_CASE(ERROR_CODE_INVALID_LMP_PARAMETERS_INVALID_LL_PARAMETERS);
        ERR_CASE(ERROR_CODE_UNSPECIFIED_ERROR);
        ERR_CASE(ERROR_CODE_UNSUPPORTED_LMP_PARAMETER_VALUE_UNSUPPORTED_LL_PARAMETER_VALUE);
        ERR_CASE(ERROR_CODE_ROLE_CHANGE_NOT_ALLOWED);
        ERR_CASE(ERROR_CODE_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT);
        ERR_CASE(ERROR_CODE_LMP_ERROR_TRANSACTION_COLLISION);
        ERR_CASE(ERROR_CODE_LMP_PDU_NOT_ALLOWED);
        ERR_CASE(ERROR_CODE_ENCRYPTION_MODE_NOT_ACCEPTABLE);
        ERR_CASE(ERROR_CODE_LINK_KEY_CANNOT_BE_CHANGED);
        ERR_CASE(ERROR_CODE_REQUESTED_QOS_NOT_SUPPORTED);
        ERR_CASE(ERROR_CODE_INSTANT_PASSED);
        ERR_CASE(ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED);
        ERR_CASE(ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION);
        ERR_CASE(ERROR_CODE_RESERVED);
        ERR_CASE(ERROR_CODE_QOS_UNACCEPTABLE_PARAMETER);
        ERR_CASE(ERROR_CODE_QOS_REJECTED);
        ERR_CASE(ERROR_CODE_CHANNEL_CLASSIFICATION_NOT_SUPPORTED);
        ERR_CASE(ERROR_CODE_INSUFFICIENT_SECURITY);
        ERR_CASE(ERROR_CODE_PARAMETER_OUT_OF_MANDATORY_RANGE);
        ERR_CASE(ERROR_CODE_ROLE_SWITCH_PENDING);
        ERR_CASE(ERROR_CODE_RESERVED_SLOT_VIOLATION);
        ERR_CASE(ERROR_CODE_ROLE_SWITCH_FAILED);
        ERR_CASE(ERROR_CODE_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE);
        ERR_CASE(ERROR_CODE_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST);
        ERR_CASE(ERROR_CODE_HOST_BUSY_PAIRING);
        ERR_CASE(ERROR_CODE_CONNECTION_REJECTED_DUE_TO_NO_SUITABLE_CHANNEL_FOUND);
        ERR_CASE(ERROR_CODE_CONTROLLER_BUSY);
        ERR_CASE(ERROR_CODE_UNACCEPTABLE_CONNECTION_PARAMETERS);
        ERR_CASE(ERROR_CODE_DIRECTED_ADVERTISING_TIMEOUT);
        ERR_CASE(ERROR_CODE_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE);
        ERR_CASE(ERROR_CODE_CONNECTION_FAILED_TO_BE_ESTABLISHED);
        ERR_CASE(ERROR_CODE_MAC_CONNECTION_FAILED);
        ERR_CASE(ERROR_CODE_COARSE_CLOCK_ADJUSTMENT_REJECTED_BUT_WILL_TRY_TO_ADJUST_USING_CLOCK_DRAGGING);
        ERR_CASE(BTSTACK_CONNECTION_TO_BTDAEMON_FAILED);
        ERR_CASE(BTSTACK_ACTIVATION_FAILED_SYSTEM_BLUETOOTH);
        ERR_CASE(BTSTACK_ACTIVATION_POWERON_FAILED);
        ERR_CASE(BTSTACK_ACTIVATION_FAILED_UNKNOWN);
        ERR_CASE(BTSTACK_NOT_ACTIVATED);
        ERR_CASE(BTSTACK_BUSY);
        ERR_CASE(BTSTACK_MEMORY_ALLOC_FAILED);
        ERR_CASE(BTSTACK_ACL_BUFFERS_FULL);
        ERR_CASE(L2CAP_COMMAND_REJECT_REASON_COMMAND_NOT_UNDERSTOOD);
        ERR_CASE(L2CAP_COMMAND_REJECT_REASON_SIGNALING_MTU_EXCEEDED);
        ERR_CASE(L2CAP_COMMAND_REJECT_REASON_INVALID_CID_IN_REQUEST);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_SUCCESSFUL);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_PENDING);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_PSM);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_SECURITY);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_REFUSED_RESOURCES);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_ERTM_NOT_SUPPORTED);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_RESULT_RTX_TIMEOUT);
        ERR_CASE(L2CAP_CONNECTION_BASEBAND_DISCONNECT);
        ERR_CASE(L2CAP_SERVICE_ALREADY_REGISTERED);
        ERR_CASE(L2CAP_DATA_LEN_EXCEEDS_REMOTE_MTU);
        ERR_CASE(L2CAP_SERVICE_DOES_NOT_EXIST);
        ERR_CASE(L2CAP_LOCAL_CID_DOES_NOT_EXIST);
        ERR_CASE(L2CAP_CONNECTION_RESPONSE_UNKNOWN_ERROR);
        ERR_CASE(RFCOMM_MULTIPLEXER_STOPPED);
        ERR_CASE(RFCOMM_CHANNEL_ALREADY_REGISTERED);
        ERR_CASE(RFCOMM_NO_OUTGOING_CREDITS);
        ERR_CASE(RFCOMM_AGGREGATE_FLOW_OFF);
        ERR_CASE(RFCOMM_DATA_LEN_EXCEEDS_MTU);
        ERR_CASE(HFP_REMOTE_REJECTS_AUDIO_CONNECTION);
        ERR_CASE(SDP_HANDLE_ALREADY_REGISTERED);
        ERR_CASE(SDP_QUERY_INCOMPLETE);
        ERR_CASE(SDP_SERVICE_NOT_FOUND);
        ERR_CASE(SDP_HANDLE_INVALID);
        ERR_CASE(SDP_QUERY_BUSY);
        ERR_CASE(ATT_HANDLE_VALUE_INDICATION_IN_PROGRESS);
        ERR_CASE(ATT_HANDLE_VALUE_INDICATION_TIMEOUT);
        ERR_CASE(ATT_HANDLE_VALUE_INDICATION_DISCONNECT);
        ERR_CASE(GATT_CLIENT_NOT_CONNECTED);
        ERR_CASE(GATT_CLIENT_BUSY);
        ERR_CASE(GATT_CLIENT_IN_WRONG_STATE);
        ERR_CASE(GATT_CLIENT_DIFFERENT_CONTEXT_FOR_ADDRESS_ALREADY_EXISTS);
        ERR_CASE(GATT_CLIENT_VALUE_TOO_LONG);
        ERR_CASE(GATT_CLIENT_CHARACTERISTIC_NOTIFICATION_NOT_SUPPORTED);
        ERR_CASE(GATT_CLIENT_CHARACTERISTIC_INDICATION_NOT_SUPPORTED);
        ERR_CASE(BNEP_SERVICE_ALREADY_REGISTERED);
        ERR_CASE(BNEP_CHANNEL_NOT_CONNECTED);
        ERR_CASE(BNEP_DATA_LEN_EXCEEDS_MTU);
        ERR_CASE(OBEX_UNKNOWN_ERROR);
        ERR_CASE(OBEX_CONNECT_FAILED);
        ERR_CASE(OBEX_DISCONNECTED);
        ERR_CASE(OBEX_NOT_FOUND);
        ERR_CASE(OBEX_NOT_ACCEPTABLE);
        ERR_CASE(OBEX_ABORTED);
        ERR_CASE(MESH_ERROR_APPKEY_INDEX_INVALID);
        default: return "unknown";
    }
}

const char* att_err_str(uint8_t err)
{
    switch (err) {
        ERR_CASE(ATT_ERROR_SUCCESS);
        ERR_CASE(ATT_ERROR_INVALID_HANDLE);
        ERR_CASE(ATT_ERROR_READ_NOT_PERMITTED);
        ERR_CASE(ATT_ERROR_WRITE_NOT_PERMITTED);
        ERR_CASE(ATT_ERROR_INVALID_PDU);
        ERR_CASE(ATT_ERROR_INSUFFICIENT_AUTHENTICATION);
        ERR_CASE(ATT_ERROR_REQUEST_NOT_SUPPORTED);
        ERR_CASE(ATT_ERROR_INVALID_OFFSET);
        ERR_CASE(ATT_ERROR_INSUFFICIENT_AUTHORIZATION);
        ERR_CASE(ATT_ERROR_PREPARE_QUEUE_FULL);
        ERR_CASE(ATT_ERROR_ATTRIBUTE_NOT_FOUND);
        ERR_CASE(ATT_ERROR_ATTRIBUTE_NOT_LONG);
        ERR_CASE(ATT_ERROR_INSUFFICIENT_ENCRYPTION_KEY_SIZE);
        ERR_CASE(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);
        ERR_CASE(ATT_ERROR_UNLIKELY_ERROR);
        ERR_CASE(ATT_ERROR_INSUFFICIENT_ENCRYPTION);
        ERR_CASE(ATT_ERROR_UNSUPPORTED_GROUP_TYPE);
        ERR_CASE(ATT_ERROR_INSUFFICIENT_RESOURCES);
        ERR_CASE(ATT_ERROR_VALUE_NOT_ALLOWED);
        ERR_CASE(ATT_ERROR_HCI_DISCONNECT_RECEIVED);
        ERR_CASE(ATT_ERROR_BONDING_INFORMATION_MISSING);
        ERR_CASE(ATT_ERROR_DATA_MISMATCH);
        ERR_CASE(ATT_ERROR_TIMEOUT);
        ERR_CASE(ATT_ERROR_WRITE_REQUEST_REJECTED);
        ERR_CASE(ATT_ERROR_CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTOR_IMPROPERLY_CONFIGURED);
        ERR_CASE(ATT_ERROR_PROCEDURE_ALREADY_IN_PROGRESS);
        ERR_CASE(ATT_ERROR_OUT_OF_RANGE);
        default: return "unknown";
    }
}

const char* l2cap_err_str(uint8_t err) {
    switch (err) {
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_SUCCESS);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_SPSM_NOT_SUPPORTED);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_NO_RESOURCES_AVAILABLE);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_AUTHENTICATION);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_AUTHORIZATION);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_ENCYRPTION_KEY_SIZE_TOO_SHORT);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_INSUFFICIENT_ENCRYPTION);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_INVALID_SOURCE_CID);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_SOURCE_CID_ALREADY_ALLOCATED);
        ERR_CASE(L2CAP_CBM_CONNECTION_RESULT_UNACCEPTABLE_PARAMETERS);
        default: return "unknown";
    }
}

const char* sm_reason_str(uint8_t err)
{
    switch (err) {
        ERR_CASE(SM_REASON_RESERVED);
        ERR_CASE(SM_REASON_PASSKEY_ENTRY_FAILED);
        ERR_CASE(SM_REASON_OOB_NOT_AVAILABLE);
        ERR_CASE(SM_REASON_AUTHENTHICATION_REQUIREMENTS);
        ERR_CASE(SM_REASON_CONFIRM_VALUE_FAILED);
        ERR_CASE(SM_REASON_PAIRING_NOT_SUPPORTED);
        ERR_CASE(SM_REASON_ENCRYPTION_KEY_SIZE);
        ERR_CASE(SM_REASON_COMMAND_NOT_SUPPORTED);
        ERR_CASE(SM_REASON_UNSPECIFIED_REASON);
        ERR_CASE(SM_REASON_REPEATED_ATTEMPTS);
        ERR_CASE(SM_REASON_INVALID_PARAMETERS);
        ERR_CASE(SM_REASON_DHKEY_CHECK_FAILED);
        ERR_CASE(SM_REASON_NUMERIC_COMPARISON_FAILED);
        ERR_CASE(SM_REASON_BR_EDR_PAIRING_IN_PROGRESS);
        ERR_CASE(SM_REASON_CROSS_TRANSPORT_KEY_DERIVATION_NOT_ALLOWED);
        ERR_CASE(SM_REASON_KEY_REJECTED);
        default: return "unknown";
    }
}

} // namespace asha