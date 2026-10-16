#pragma once

#include <atomic>

#include "usb_common.hpp"

namespace asha
{

enum class StartupStatus { Pending, Ready, Failed };
enum class StartupError { None, CYW43, RuntimeSettings, Timeout };

// Core 1 publishes exactly once. Payloads remain immutable after publication.
class StartupResult {
public:
    void publish_ready(USBSettings const& settings)
    {
        settings_ = settings;
        status_.store(StartupStatus::Ready, std::memory_order_release);
    }

    void publish_failure(StartupError error)
    {
        error_ = error;
        status_.store(StartupStatus::Failed, std::memory_order_release);
    }

    // Core 0 never accesses either payload until it observes its publication.
    [[nodiscard]] StartupStatus read(USBSettings& settings, StartupError& error) const
    {
        auto const status = status_.load(std::memory_order_acquire);
        if (status == StartupStatus::Ready) settings = settings_;
        if (status == StartupStatus::Failed) error = error_;
        return status;
    }

private:
    std::atomic<StartupStatus> status_{StartupStatus::Pending};
    USBSettings settings_{};
    StartupError error_{StartupError::None};
};

extern StartupResult bt_startup;

} // namespace asha
