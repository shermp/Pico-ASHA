#include "stdio.h"
#include "string.h"

#include "util.hpp"

namespace asha
{
bool str_eq(const char* str1, const char* str2) {
    return (strcmp(str1, str2) == 0);
}
} // namespace asha