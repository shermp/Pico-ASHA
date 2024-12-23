#pragma once

#include <cstdint>

namespace asha
{

const char* bt_err_str(uint8_t err);
const char* att_err_str(uint8_t err);
const char* l2cap_err_str(uint8_t err);
const char* sm_reason_str(uint8_t err);

} // namespace asha