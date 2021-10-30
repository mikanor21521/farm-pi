#include "driver.hpp"

#include <memory>
#include <string>

#include "pigpio.h"

#include "spdlog/spdlog.h"

namespace alcor {

Driver::Driver(const std::shared_ptr<spdlog::logger>& logger) {
    std::string logger_name = "driver";
    logger_ = logger->clone(logger_name);
}

void Driver::initialize() {
    int ret = 0;
    ret = gpioInitialise();
    if (ret < 0) {
        logger_->error("pigpio initialization failed.");
        driver::throwDriverException(
            "pigpio initialization failed. status code: {}", ret);
    }
    logger_->info("pigpio initialization done.");
}

void Driver::finalize() {
    gpioTerminate();
    logger_->info("pigpio finalization done.");
}

}  // namespace alcor
