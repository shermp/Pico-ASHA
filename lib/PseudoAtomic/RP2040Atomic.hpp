/**
 * @copyright 2023
 * @author TSprech
 * @date 2023/03/27 11:47:36
 * @brief Library which emulates the STL atomic variables using Pico hardware synchronization.
 * @license Apache 2.0
 */

#ifndef RP2040ATOMIC_HPP
#define RP2040ATOMIC_HPP

#include <cstdint>
#include <concepts>
#include "pico/sync.h"

namespace patom {
  namespace internal {
    template<typename T> // Same requirements as std::atomic
    concept atomic_t = std::is_trivially_copyable_v<T> && std::is_copy_constructible_v<T> &&
                       std::is_move_constructible_v<T> && std::is_copy_assignable_v<T> && std::is_move_assignable_v<T>;

    /* Note a single critical section is used as pseudo atomic was designed to have atomic access between cores.
     * Using within an RTOS (or other threaded environment) has not been considered, nor how a shared spinlock across all atomic would affect that. */
    inline critical_section_t ct{}; // The critical section shared by all atomic variables
  }  // namespace internal

  /**
   * @brief Base class for creating an atomic variable.
   * @tparam T The type of the atomic variable, must satisfy atomic_t requirements.
   * @todo Check for C++ version and only use concepts if C++20 or greater is used.
   */
  template<internal::atomic_t T>
  class PseudoAtomic {
  public:
    /**
     * @brief Assignment operator which atomically updates the atomic's value.
     * @param t The new value for the atomic variable.
     * @returns A reference to this.
     */
    auto operator=(T t) -> PseudoAtomic<T>& {
      critical_section_enter_blocking(&ct_); // Lock from reading
      t_ = t; // Assign
      critical_section_exit(&ct_); // Unlock for reading or writing
      return *this; // Default for operator= overload
    }

    /**
     * @breif Atomically fetches the value of the atomic.
     * @returns The current value of the atomic variable.
     */
    auto Load() -> T {
      critical_section_enter_blocking(&ct_); // Lock from writing
      auto t = t_; // Get the internal variable's value
      critical_section_exit(&ct_); // Unlock for reading or writing
      return t; // Return a copy of the current value
    }

    PseudoAtomic() = default;                       // Required to still allow object to be created
    // Copy and move construction / assignment is disabled as access must happen explicitly and be guarded by critical_sections
    PseudoAtomic(const PseudoAtomic<T>&) = delete;  // Remove copy construction
    PseudoAtomic(PseudoAtomic<T>&&) = delete;       // Remove move construction
    PseudoAtomic<T>& operator=(const PseudoAtomic<T>&) = delete;  // Remove copy assignment
    PseudoAtomic<T>& operator=(PseudoAtomic<T>&&) = delete;       // Remove move assignment

   private:
    inline static critical_section_t& ct_ = internal::ct;  /**< Critical section used for protecting reading and swapping */
    volatile T t_; /**< Internal value of the atomic */
  };

  /**
   * @brief Claims a spinlock for the internal critical_section lock.
   * @warning This function MUST be called before using any of the pseudo atomic functionality.
   */
  inline auto PseudoAtomicInit() -> void {
    critical_section_init(&internal::ct);  // Initialize the internal critical_section spin lock
  }

  namespace types {
    // Standard type definitions given by std::atomic and thus replicated by this library
    using patomic_bool = PseudoAtomic<bool>;
    using patomic_char = PseudoAtomic<char>;
    using patomic_schar = PseudoAtomic<signed char>;
    using patomic_uchar = PseudoAtomic<unsigned char>;
    using patomic_short = PseudoAtomic<short>;
    using patomic_ushort = PseudoAtomic<unsigned short>;
    using patomic_int = PseudoAtomic<int>;
    using patomic_uint = PseudoAtomic<unsigned int>;
    using patomic_long = PseudoAtomic<long>;
    using patomic_ulong = PseudoAtomic<unsigned long>;
    using patomic_llong = PseudoAtomic<long long>;
    using patomic_ullong = PseudoAtomic<unsigned long long>;
    using patomic_char8_t = PseudoAtomic<char8_t>;
    using patomic_char16_t = PseudoAtomic<char16_t>;
    using patomic_char32_t = PseudoAtomic<char32_t>;
    using patomic_wchar_t = PseudoAtomic<wchar_t>;
    using patomic_int8_t = PseudoAtomic<std::int8_t>;
    using patomic_uint8_t = PseudoAtomic<std::uint8_t>;
    using patomic_int16_t = PseudoAtomic<std::int16_t>;
    using patomic_uint16_t = PseudoAtomic<std::uint16_t>;
    using patomic_int32_t = PseudoAtomic<std::int32_t>;
    using patomic_uint32_t = PseudoAtomic<std::uint32_t>;
    using patomic_int64_t = PseudoAtomic<std::int64_t>;
    using patomic_uint64_t = PseudoAtomic<std::uint64_t>;
    using patomic_int_least8_t = PseudoAtomic<std::int_least8_t>;
    using patomic_uint_least8_t = PseudoAtomic<std::uint_least8_t>;
    using patomic_int_least16_t = PseudoAtomic<std::int_least16_t>;
    using patomic_uint_least16_t = PseudoAtomic<std::uint_least16_t>;
    using patomic_int_least32_t = PseudoAtomic<std::int_least32_t>;
    using patomic_uint_least32_t = PseudoAtomic<std::uint_least32_t>;
    using patomic_int_least64_t = PseudoAtomic<std::int_least64_t>;
    using patomic_uint_least64_t = PseudoAtomic<std::uint_least64_t>;
    using patomic_int_fast8_t = PseudoAtomic<std::int_fast8_t>;
    using patomic_uint_fast8_t = PseudoAtomic<std::uint_fast8_t>;
    using patomic_int_fast16_t = PseudoAtomic<std::int_fast16_t>;
    using patomic_uint_fast16_t = PseudoAtomic<std::uint_fast16_t>;
    using patomic_int_fast32_t = PseudoAtomic<std::int_fast32_t>;
    using patomic_uint_fast32_t = PseudoAtomic<std::uint_fast32_t>;
    using patomic_int_fast64_t = PseudoAtomic<std::int_fast64_t>;
    using patomic_uint_fast64_t = PseudoAtomic<std::uint_fast64_t>;
    using patomic_intptr_t = PseudoAtomic<std::intptr_t>;
    using patomic_uintptr_t = PseudoAtomic<std::uintptr_t>;
    using patomic_size_t = PseudoAtomic<std::size_t>;
    using patomic_ptrdiff_t = PseudoAtomic<std::ptrdiff_t>;
    using patomic_intmax_t = PseudoAtomic<std::intmax_t>;
    using patomic_uintmax_t = PseudoAtomic<std::uintmax_t>;
  } // namespace types
} // namespace patom

#endif  // RP2040ATOMIC_HPP
