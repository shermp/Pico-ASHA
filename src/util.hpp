#pragma once

namespace asha
{

template <typename T, typename... Args>
bool is_any_of(T t, Args... args)
{
    return ((t == args) || ...);
}

} // namespace asha