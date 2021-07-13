#include <bitset>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pigpio/pigpio.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"
#include "driver/gpio.hpp"
#include "driver/spi.hpp"

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
    auto logger = spdlog::get(logger_name);
    const uint8_t gpio_num = 4;
    const driver::gpio::mode gpio_mode = driver::gpio::mode::output;
    driver::initialize();
    driver::gpio::gpio gpio(gpio_num, gpio_mode);
    gpio.initialize();
    for (int i = 0; i < 10; i++) {
        if (i % 2 == 0) {
            logger->info("gpio turn on.");
            gpio.set_level(driver::gpio::level::high);
        } else {
            logger->info("gpio turn off.");
            gpio.set_level(driver::gpio::level::low);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    gpio.finalize();
    const std::uint8_t spi_slave = 0;
    const std::uint32_t spi_clock = 100000;
    const std::uint8_t spi_mode = 0;
    const driver::spi::active_high spi_active_high =
        driver::spi::active_high::disable;
    driver::spi::spi spi(spi_slave, spi_clock, spi_mode, spi_active_high);
    spi.initialize();
    std::vector<std::bitset<8>> conf = {0x68, 0x00};
    auto data = spi.transfer(conf);
    std::uint16_t volt = data.at(0).to_ulong() << 8 | data.at(1).to_ulong();
    logger->info("volt: {}", volt);
    spi.finalize();
    driver::finalize();
    spdlog::drop_all();
    return 0;
}
