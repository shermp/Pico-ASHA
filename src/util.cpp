#include "stdio.h"
#include "string.h"

#include "util.hpp"

namespace asha
{
void str_to_bd_addr(const char* addr_str, bd_addr_t addr)
{
    sscanf(addr_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
           &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
}

bool str_eq(const char* str1, const char* str2) {
    return (strcmp(str1, str2) == 0);
}
} // namespace asha