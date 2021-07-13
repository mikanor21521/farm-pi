#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "pigpio/pigpio.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"

namespace driver {

const std::string logger_name = "driver";
const std::string logfile_name = "log/driver.log";

void initialize(const std::string& file_name) {
    // initializing logger
    try {
        // create console sink
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::info);
        console_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        // create daily file sink
        auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
            file_name, 0, 0);
        file_sink->set_level(spdlog::level::trace);
        file_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        // create sinks list
        spdlog::sinks_init_list sinks = {console_sink, file_sink};
        // create logger
        auto logger = std::make_shared<spdlog::logger>(logger_name, sinks);
        logger->set_level(spdlog::level::trace);
        spdlog::register_logger(logger);
        spdlog::get(logger_name)->debug("logger initialization done.");
    } catch (const spdlog::spdlog_ex& ex) {
        spdlog::get(logger_name)
            ->error("logger initialization failed. {}", ex.what());
        throw driver_ex("logger initialization failed. " +
                        std::string(ex.what()));
    }
    // initializing pigpio
    std::int16_t ret = 0;
    ret = gpioInitialise();
    if (ret < 0) {
        spdlog::get(logger_name)
            ->error("pigpio initialization failed. status code: {}", ret);
        throw driver_ex("pigpio initialization failed.", ret);
    }
    spdlog::get(logger_name)->debug("pigpio initialization done.");
}

void finalize() {
    gpioTerminate();
    spdlog::get(logger_name)->debug("pigpio finalization done.");
    spdlog::get(logger_name)->debug("logger finalization done.");
    spdlog::drop(logger_name);
}

driver_ex::driver_ex(std::string msg) : msg_(std::move(msg)) {}

driver_ex::driver_ex(const std::string& msg, std::int16_t error_num)
    : msg_(msg + " status code: " + std::to_string(error_num)) {}

const char* driver_ex::what() const noexcept {
    return msg_.c_str();
}

}  // namespace driver
