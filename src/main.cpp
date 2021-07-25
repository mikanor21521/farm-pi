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
#include "mqtt/client.h"

#include "driver/device/bme280.hpp"
#include "driver/device/mcp3002.hpp"
#include "driver/device/tsl2561.hpp"
#include "driver/driver.hpp"
#include "driver/gpio.hpp"
#include "driver/i2c.hpp"
#include "driver/spi.hpp"

namespace {

const std::string logger_name = "albireo-pi";
const std::string log_file_name = "log/main.log";
const std::string farm_id = "uranum";
const std::string url = "tcp://localhost:1883";

namespace k5000 {

namespace mcp3002 {

namespace spi {

constexpr std::uint8_t ss = 23;
constexpr std::uint8_t miso = 27;
constexpr std::uint8_t mosi = 17;
constexpr std::uint8_t sck = 22;
constexpr std::uint32_t clock_speed = 100000;
constexpr std::uint8_t mode = 0;
constexpr driver::spi::active_high active_high =
    driver::spi::active_high::disable;

}  // namespace spi

constexpr double base_voltage = 3.3;

}  // namespace mcp3002

namespace bme280 {

namespace spi {

constexpr std::uint8_t slave = 0;
constexpr std::uint32_t clock_speed = 100000;
constexpr std::uint8_t mode = 0;
constexpr driver::spi::active_high active_high =
    driver::spi::active_high::disable;

}  // namespace spi

constexpr driver::device::bme280::mode mode =
    driver::device::bme280::mode::normal;
constexpr driver::device::bme280::oversampling_temperature osr_temp =
    driver::device::bme280::oversampling_temperature::x16;
constexpr driver::device::bme280::oversampling_pressure osr_pres =
    driver::device::bme280::oversampling_pressure::x16;
constexpr driver::device::bme280::oversampling_humidity osr_hum =
    driver::device::bme280::oversampling_humidity::x16;
constexpr driver::device::bme280::standby_time standby =
    driver::device::bme280::standby_time::ms1000;
constexpr driver::device::bme280::filter filter =
    driver::device::bme280::filter::x16;

}  // namespace bme280

namespace tsl2561 {

namespace i2c {

constexpr std::uint8_t bus_number = 1;
constexpr std::uint8_t address = 0x39;

}  // namespace i2c

constexpr driver::device::tsl2561::gain gain =
    driver::device::tsl2561::gain::x16;
constexpr driver::device::tsl2561::integral integral =
    driver::device::tsl2561::integral::ms402;

}  // namespace tsl2561

}  // namespace k5000

namespace k3000 {

namespace mcp3002 {

namespace spi {

constexpr std::uint8_t ss = 24;
constexpr std::uint8_t miso = 27;
constexpr std::uint8_t mosi = 17;
constexpr std::uint8_t sck = 22;
constexpr std::uint32_t clock_speed = 100000;
constexpr std::uint8_t mode = 0;
constexpr driver::spi::active_high active_high =
    driver::spi::active_high::disable;

}  // namespace spi

constexpr double base_voltage = 3.3;

}  // namespace mcp3002

namespace bme280 {

namespace spi {

constexpr std::uint8_t slave = 1;
constexpr std::uint32_t clock_speed = 100000;
constexpr std::uint8_t mode = 0;
constexpr driver::spi::active_high active_high =
    driver::spi::active_high::disable;

}  // namespace spi

constexpr driver::device::bme280::mode mode =
    driver::device::bme280::mode::normal;
constexpr driver::device::bme280::oversampling_temperature osr_temp =
    driver::device::bme280::oversampling_temperature::x16;
constexpr driver::device::bme280::oversampling_pressure osr_pres =
    driver::device::bme280::oversampling_pressure::x16;
constexpr driver::device::bme280::oversampling_humidity osr_hum =
    driver::device::bme280::oversampling_humidity::x16;
constexpr driver::device::bme280::standby_time standby =
    driver::device::bme280::standby_time::ms1000;
constexpr driver::device::bme280::filter filter =
    driver::device::bme280::filter::x16;

}  // namespace bme280

namespace tsl2561 {

namespace i2c {

constexpr std::uint8_t bus_number = 1;
constexpr std::uint8_t address = 0x29;

}  // namespace i2c

constexpr driver::device::tsl2561::gain gain =
    driver::device::tsl2561::gain::x16;
constexpr driver::device::tsl2561::integral integral =
    driver::device::tsl2561::integral::ms402;

}  // namespace tsl2561

}  // namespace k3000

}  // namespace

void send_to_database(const std::string& topic, double data);

[[noreturn]] void data();

int main() {
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
    driver::initialize();
    std::thread data_th(data);
    data_th.join();
    driver::finalize();
    spdlog::drop_all();
    return 0;
}

void send_to_database(const std::string& topic, double data) {
    mqtt::client cli(url, farm_id);
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    try {
        cli.connect(connOpts);
        auto msg = mqtt::make_message("/farms" + farm_id + "/" + topic, std::to_string(data));
        msg->set_qos(1);
        cli.publish(msg);
        cli.disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << " ["
            << exc.get_reason_code() << "]" << std::endl;
    }
}

[[noreturn]] void data() {
    auto logger = spdlog::get(logger_name);
    logger->info("Data start.");
    // 5000k
    // water_temperature and led_temperature
    auto* k5000_mcp3002_spi = new driver::spi::bit_banging_spi(
        k5000::mcp3002::spi::ss, k5000::mcp3002::spi::miso,
        k5000::mcp3002::spi::mosi, k5000::mcp3002::spi::sck,
        k5000::mcp3002::spi::clock_speed, k5000::mcp3002::spi::mode,
        k5000::mcp3002::spi::active_high);
    driver::device::mcp3002::mcp3002 k5000_mcp3002(
        k5000_mcp3002_spi, k5000::mcp3002::base_voltage);
    k5000_mcp3002.initialize();
    // temperature, pressure and humidity
    auto* k5000_bme280_spi = new driver::spi::main_spi(
        k5000::bme280::spi::slave, k5000::bme280::spi::clock_speed,
        k5000::bme280::spi::mode, k5000::bme280::spi::active_high);
    driver::device::bme280::bme280 k5000_bme280(
        k5000_bme280_spi, k5000::bme280::mode, k5000::bme280::osr_temp,
        k5000::bme280::osr_pres, k5000::bme280::osr_hum, k5000::bme280::standby,
        k5000::bme280::filter);
    k5000_bme280.initialize();
    // illuminance
    driver::i2c::i2c k5000_tsl2561_i2c(k5000::tsl2561::i2c::bus_number,
                                       k5000::tsl2561::i2c::address);
    driver::device::tsl2561::tsl2561 k5000_tsl2561(
        k5000_tsl2561_i2c, k5000::tsl2561::gain, k5000::tsl2561::integral);
    k5000_tsl2561.initialize();
    // 3000k
    // water_temperature and led_temperature
    auto* k3000_mcp3002_spi = new driver::spi::bit_banging_spi(
        k3000::mcp3002::spi::ss, k3000::mcp3002::spi::miso,
        k3000::mcp3002::spi::mosi, k3000::mcp3002::spi::sck,
        k3000::mcp3002::spi::clock_speed, k3000::mcp3002::spi::mode,
        k3000::mcp3002::spi::active_high);
    driver::device::mcp3002::mcp3002 k3000_mcp3002(
        k3000_mcp3002_spi, k3000::mcp3002::base_voltage);
    k3000_mcp3002.initialize();
    // temperature, pressure and humidity
    auto* k3000_bme280_spi = new driver::spi::main_spi(
        k3000::bme280::spi::slave, k3000::bme280::spi::clock_speed,
        k3000::bme280::spi::mode, k3000::bme280::spi::active_high);
    driver::device::bme280::bme280 k3000_bme280(
        k3000_bme280_spi, k3000::bme280::mode, k3000::bme280::osr_temp,
        k3000::bme280::osr_pres, k3000::bme280::osr_hum, k3000::bme280::standby,
        k3000::bme280::filter);
    k3000_bme280.initialize();
    // illuminance
    driver::i2c::i2c k3000_tsl2561_i2c(k3000::tsl2561::i2c::bus_number,
                                       k3000::tsl2561::i2c::address);
    driver::device::tsl2561::tsl2561 k3000_tsl2561(
        k3000_tsl2561_i2c, k3000::tsl2561::gain, k3000::tsl2561::integral);
    k3000_tsl2561.initialize();
    while (true) {
        send_to_database("k5000/water_temperature" , k5000_mcp3002.read_voltage(driver::device::mcp3002::channel::single_0));
        send_to_database("k5000/led_temperature"   , k5000_mcp3002.read_voltage(driver::device::mcp3002::channel::single_1));
        send_to_database("k5000/temperature"       , k5000_bme280.read_temperature());
        send_to_database("k5000/pressure"          , k5000_bme280.read_pressure());
        send_to_database("k5000/humidity"          , k5000_bme280.read_humidity());
        send_to_database("k5000/illuminance"       , k5000_tsl2561.read_illuminance());
        send_to_database("k3000/water_temperature" , k3000_mcp3002.read_voltage(driver::device::mcp3002::channel::single_0));
        send_to_database("k3000/led_temperature"   , k3000_mcp3002.read_voltage(driver::device::mcp3002::channel::single_1));
        send_to_database("k3000/temperature"       , k3000_bme280.read_temperature());
        send_to_database("k3000/pressure"          , k3000_bme280.read_pressure());
        send_to_database("k3000/humidity"          , k3000_bme280.read_humidity());
        send_to_database("k3000/illuminance"       , k3000_tsl2561.read_illuminance());
        std::this_thread::sleep_for(std::chrono::minutes(1));
    }
}
