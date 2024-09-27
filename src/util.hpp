#pragma once

#include <btstack.h>

namespace asha
{

template <typename T, typename... Args>
bool is_any_of(T t, Args... args)
{
    return ((t == args) || ...);
}

void str_to_bd_addr(const char* addr_str, bd_addr_t addr);
bool str_eq(const char* str1, const char* str2);

bool is_valid(gatt_client_service_t* service);
bool is_valid(gatt_client_characteristic_t* characteristic);

} // namespace asha