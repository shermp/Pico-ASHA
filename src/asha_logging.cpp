#include "pico/stdio_usb.h"

#include "asha_logging.h"

namespace asha
{

void handle_logging_pending_worker([[maybe_unused]] async_context_t *context, [[maybe_unused]] async_when_pending_worker_t *worker)
{
    while(!log_buffer.empty() && stdio_usb_connected()) {
        printf("%s", log_buffer.front().c_str());
        log_buffer.pop();
    }
}

} //namespace asha