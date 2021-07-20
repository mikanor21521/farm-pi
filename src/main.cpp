#include <bitset>
#include <chrono>
#include <cmath>
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

#include "driver/device/bme280.hpp"
#include "driver/device/mcp3002.hpp"
#include "driver/device/tsl2561.hpp"
#include "driver/driver.hpp"
#include "driver/gpio.hpp"
#include "driver/i2c.hpp"
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
    // mcp3002
    const std::uint8_t mcp3002_spi_slave = 0;
    const std::uint32_t mcp3002_spi_clock = 100000;
    const std::uint8_t mcp3002_spi_mode = 0;
    const driver::spi::active_high mcp3002_spi_active_high =
        driver::spi::active_high::disable;
    const double mcp3002_vase_voltage = 3.3;
    driver::spi::spi mcp3002_spi(mcp3002_spi_slave, mcp3002_spi_clock,
                                 mcp3002_spi_mode, mcp3002_spi_active_high);
    driver::device::mcp3002::mcp3002 mcp3002(mcp3002_spi, mcp3002_vase_voltage);
    mcp3002.initialize();
    double voltage =
        mcp3002.read_voltage(driver::device::mcp3002::channel::single_0);
    logger->info("mcp3002 voltage: {}", voltage);
    mcp3002.finalize();
    // bme280
    const std::uint8_t bme280_spi_slave = 1;
    const std::uint32_t bme280_spi_clock = 100000;
    const std::uint8_t bme280_spi_mode = 0;
    const driver::spi::active_high bme280_spi_active_high =
        driver::spi::active_high::disable;
    const driver::device::bme280::mode bme280_mode =
        driver::device::bme280::mode::normal;
    const driver::device::bme280::oversampling_temperature bme280_osr_temp =
        driver::device::bme280::oversampling_temperature::x16;
    const driver::device::bme280::oversampling_pressure bme280_osr_pres =
        driver::device::bme280::oversampling_pressure::x16;
    const driver::device::bme280::oversampling_humidity bme280_osr_hum =
        driver::device::bme280::oversampling_humidity::x16;
    const driver::device::bme280::standby_time bme280_standby =
        driver::device::bme280::standby_time::ms1000;
    const driver::device::bme280::filter bme280_filter =
        driver::device::bme280::filter::x16;
    driver::spi::spi bme280_spi(bme280_spi_slave, bme280_spi_clock,
                                bme280_spi_mode, bme280_spi_active_high);
    driver::device::bme280::bme280 bme280(
        bme280_spi, bme280_mode, bme280_osr_temp, bme280_osr_pres,
        bme280_osr_hum, bme280_standby, bme280_filter);
    bme280.initialize();
    double temperature = bme280.read_temperature();
    double pressure = bme280.read_pressure();
    double humidity = bme280.read_humidity();
    logger->info("bme280 temperature: {}", temperature);
    logger->info("bme280 pressure: {}", pressure);
    logger->info("bme280 humidity: {}", humidity);
    bme280.finalize();
    // tsl2561
    const std::uint8_t tsl2561_i2c_bus = 1;
    const std::uint8_t tsl2561_i2c_address =
        driver::device::tsl2561::address_float;
    const driver::device::tsl2561::gain tsl2561_gain =
        driver::device::tsl2561::gain::x16;
    const driver::device::tsl2561::integral tsl2561_integral =
        driver::device::tsl2561::integral::ms402;
    driver::i2c::i2c tsl2561_i2c(tsl2561_i2c_bus, tsl2561_i2c_address);
    driver::device::tsl2561::tsl2561 tsl2561(tsl2561_i2c, tsl2561_gain,
                                             tsl2561_integral);
    tsl2561.initialize();
    double illuminance = tsl2561.read_illuminance();
    logger->info("tsl2561 illuminance: {}", illuminance);
    tsl2561.finalize();
    driver::finalize();
    spdlog::drop_all();
    return 0;
}
