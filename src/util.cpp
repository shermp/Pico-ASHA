#include <stdio.h>
#include <string.h>

#include "util.hpp"

namespace asha
{
bool str_eq(const char* str1, const char* str2) {
    return (strcmp(str1, str2) == 0);
}

bool is_valid(gatt_client_service_t* service)
{
    return service && service->end_group_handle != 0x0000u;
}

bool is_valid(gatt_client_characteristic_t* characteristic)
{
    return characteristic && characteristic->end_handle != 0x0000u;
}
} // namespace asha