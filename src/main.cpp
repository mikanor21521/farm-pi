#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "pigpio/pigpio.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

int main() {
    std::string logger_name = "albireo-pi";
    std::string log_file_name = "log/main.log";
    try {
        // create console sink
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::info);
        console_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        // create daily file sink
        auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
            log_file_name, 0, 0);
        file_sink->set_level(spdlog::level::trace);
        file_sink->set_pattern("[%Y/%m/%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        // create sinks list
        spdlog::sinks_init_list sinks_list = {console_sink, file_sink};
        // create logger
        auto logger = std::make_shared<spdlog::logger>(logger_name, sinks_list);
        logger->set_level(spdlog::level::trace);
        logger->debug("logger initialization done.");
        spdlog::register_logger(logger);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "log initialization failed. " << ex.what() << std::endl;
    }
    std::int16_t ret = 0;
    const uint8_t gpio_num = 4;
    const uint8_t gpio_mode = PI_OUTPUT;
    auto logger = spdlog::get(logger_name);
    ret = gpioInitialise();
    if (ret < 0) {
        logger->error("pigpio initialization failed.");
        logger->error("error code: {}", ret);
    } else {
        logger->debug("pigpio initialization done.");
        logger->trace("gpio number: {}", gpio_num);
        logger->trace("gpio mode: {}", gpio_mode);
        gpioSetMode(gpio_num, gpio_mode);
        logger->debug("gpio initialization done.");
        for (std::uint8_t i = 0; i < 10; i++) {
            if (i % 2 == 0) {
                gpioWrite(gpio_num, PI_HIGH);
                logger->info("gpio turn on.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else {
                gpioWrite(gpio_num, PI_LOW);
                logger->info("gpio turn off.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        gpioWrite(gpio_num, PI_LOW);
        gpioSetMode(gpio_num, PI_INPUT);
        logger->debug("gpio finalization done.");
        gpioTerminate();
        logger->debug("pigpio finalization done.");
    }
    spdlog::drop_all();
    return 0;
}
