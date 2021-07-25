#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver/device/bme280.hpp"
#include "driver/driver.hpp"
#include "driver/spi.hpp"

namespace driver::device::bme280 {

namespace {

namespace address {

constexpr std::uint8_t ctrl_meas = 0xF4;
constexpr std::uint8_t ctrl_hum = 0xF2;
constexpr std::uint8_t config = 0xF5;
constexpr std::uint8_t chip_id = 0xD0;
constexpr std::uint8_t status = 0xF3;
constexpr std::uint8_t reset = 0xE0;

namespace calib {

constexpr std::uint8_t temperature = 0x88;
constexpr std::uint8_t pressure = 0x8E;
constexpr std::uint8_t humidity_1 = 0xA1;
constexpr std::uint8_t humidity = 0xE1;

}  // namespace calib

namespace data {

constexpr std::uint8_t temperature = 0xFA;
constexpr std::uint8_t pressure = 0xF7;
constexpr std::uint8_t humidity = 0xFD;

}  // namespace data

}  // namespace address

namespace limit {

namespace temperature {

constexpr double max = 85.0;
constexpr double min = -40.0;

}  // namespace temperature

namespace pressure {

constexpr double max = 1100.0;
constexpr double min = 300.0;

}  // namespace pressure

namespace humidity {

constexpr double max = 100.0;
constexpr double min = 0.0;

}  // namespace humidity

}  // namespace limit

constexpr std::uint8_t chip_id = 0x60;

}  // namespace

bme280::bme280(spi::spi* spi_device, mode mode,
               oversampling_temperature osr_temp,
               oversampling_pressure osr_pres, oversampling_humidity osr_hum,
               standby_time standby, filter filter) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("bme280 mode: {}", static_cast<std::uint8_t>(mode));
    if (mode != mode::sleep && mode != mode::forced && mode != mode::normal) {
        logger->error("bme280 initialization failed. mode is invalid.");
        throw driver_ex("bme280 initialization failed. mode is invalid.");
    }
    mode_ = mode;
    logger->trace("bme280 temperature oversampling: {}", osr_temp);
    if (osr_temp != oversampling_temperature::skip &&
        osr_temp != oversampling_temperature::x1 &&
        osr_temp != oversampling_temperature::x2 &&
        osr_temp != oversampling_temperature::x4 &&
        osr_temp != oversampling_temperature::x8 &&
        osr_temp != oversampling_temperature::x16) {
        logger->error(
            "bme280 initialization failed. temperature oversampling is "
            "invalid.");
        throw driver_ex(
            "bme280 initialization failed. temperature oversampling is "
            "invalid.");
    }
    osr_temp_ = osr_temp;
    logger->trace("bme280 pressure oversampling: {}", osr_pres);
    if (osr_pres != oversampling_pressure::skip &&
        osr_pres != oversampling_pressure::x1 &&
        osr_pres != oversampling_pressure::x2 &&
        osr_pres != oversampling_pressure::x4 &&
        osr_pres != oversampling_pressure::x8 &&
        osr_pres != oversampling_pressure::x16) {
        logger->error(
            "bme280 initialization failed. pressure oversampling is "
            "invalid.");
        throw driver_ex(
            "bme280 initialization failed. pressure oversampling is "
            "invalid.");
    }
    osr_pres_ = osr_pres;
    logger->trace("bme280 humidity oversampling: {}", osr_hum);
    if (osr_hum != oversampling_humidity::skip &&
        osr_hum != oversampling_humidity::x1 &&
        osr_hum != oversampling_humidity::x2 &&
        osr_hum != oversampling_humidity::x4 &&
        osr_hum != oversampling_humidity::x8 &&
        osr_hum != oversampling_humidity::x16) {
        logger->error(
            "bme280 initialization failed. humidity oversampling is "
            "invalid.");
        throw driver_ex(
            "bme280 initialization failed. humidity oversampling is "
            "invalid.");
    }
    osr_hum_ = osr_hum;
    logger->trace("bme280 standby time: {}", standby);
    if (standby != standby_time::ms0_5 && standby != standby_time::ms62_5 &&
        standby != standby_time::ms125 && standby != standby_time::ms250 &&
        standby != standby_time::ms500 && standby != standby_time::ms1000 &&
        standby != standby_time::ms10 && standby != standby_time::ms20) {
        logger->error("bme280 initialization failed. standby time is invalid.");
        throw driver_ex(
            "bme280 initialization failed. standby time is invalid.");
    }
    standby_ = standby;
    logger->trace("bme280 filter coeff: {}", filter);
    if (filter != filter::off && filter != filter::x2 && filter != filter::x4 &&
        filter != filter::x8 && filter != filter::x16) {
        logger->error("bme280 initialization failed. filter coeff is invalid.");
        throw driver_ex(
            "bme280 initialization failed. filter coeff is invalid.");
    }
    filter_ = filter;
    spidev_ = spi_device;
    t_fine_ = 0;
}

void bme280::initialize() {
    logger->debug("bme280 initialization start.");
    spidev_->initialize();
    logger->debug("bme280 spi device initialization done.");
    std::uint8_t chip = 0;
    chip = get_registor(address::chip_id);
    logger->debug("bme280 chip id: {}", chip);
    if (chip != chip_id) {
        logger->error("bme280 initialization failed. chip id is invalid.");
        throw driver_ex("bme280 initialization failed. chip id is invalid.");
    }
    reset();
    set_settings();
    logger->debug("bme280 set settings done.");
    logger->debug("bme280 initialization done.");
}

void bme280::finalize() {
    logger->debug("bme280 finalization start.");
    spidev_->finalize();
    logger->debug("bme280 spi device finalization done.");
    logger->debug("bme280 finalization done.");
}

double bme280::read_temperature() {
    logger->debug("bme280 read temperature start.");
    std::uint16_t calib_t1 = 0;
    std::int16_t calib_t2 = 0;
    std::int16_t calib_t3 = 0;
    std::vector<std::uint8_t> reg_data(6, 0);
    for (int i = 0; i < 6; i++) {
        reg_data[i] = get_registor(address::calib::temperature + i);
    }
    calib_t1 = static_cast<std::uint16_t>(reg_data[0] | reg_data[1] << 8);
    calib_t2 = static_cast<std::int16_t>(reg_data[2] | reg_data[3] << 8);
    calib_t3 = static_cast<std::int16_t>(reg_data[4] | reg_data[5] << 8);
    reg_data.resize(3);
    for (int i = 0; i < 3; i++) {
        reg_data[i] = get_registor(address::data::temperature + i);
    }
    std::uint32_t temp_reg = 0;
    temp_reg = reg_data[0] << 12 | reg_data[1] << 4 | (reg_data[2] & 0xF0) >> 4;
    double var1 = 0.0;
    double var2 = 0.0;
    double temperature = 0.0;
    var1 = temp_reg / 16384.0 - calib_t1 / 1024.0;
    var1 = var1 * calib_t2;
    var2 = temp_reg / 131072.0 - calib_t1 / 8192.0;
    var2 = std::pow(var2, 2) * calib_t3;
    t_fine_ = var1 + var2;
    temperature = (var1 + var2) / 5120.0;
    if (temperature > limit::temperature::max) {
        temperature = limit::temperature::max;
    } else if (temperature < limit::temperature::min) {
        temperature = limit::temperature::min;
    }
    logger->debug("bme280 read temperature done.");
    return temperature;
}

double bme280::read_pressure() {
    logger->debug("bme280 read pressure start.");
    std::uint16_t calib_p1 = 0;
    std::int16_t calib_p2 = 0;
    std::int16_t calib_p3 = 0;
    std::int16_t calib_p4 = 0;
    std::int16_t calib_p5 = 0;
    std::int16_t calib_p6 = 0;
    std::int16_t calib_p7 = 0;
    std::int16_t calib_p8 = 0;
    std::int16_t calib_p9 = 0;
    std::vector<std::uint8_t> reg_data(18, 0);
    for (int i = 0; i < 18; i++) {
        reg_data[i] = get_registor(address::calib::pressure + i);
    }
    calib_p1 = static_cast<std::uint16_t>(reg_data[0] | reg_data[1] << 8);
    calib_p2 = static_cast<std::int16_t>(reg_data[2] | reg_data[3] << 8);
    calib_p3 = static_cast<std::int16_t>(reg_data[4] | reg_data[5] << 8);
    calib_p4 = static_cast<std::int16_t>(reg_data[6] | reg_data[7] << 8);
    calib_p5 = static_cast<std::int16_t>(reg_data[8] | reg_data[9] << 8);
    calib_p6 = static_cast<std::int16_t>(reg_data[10] | reg_data[11] << 8);
    calib_p7 = static_cast<std::int16_t>(reg_data[12] | reg_data[13] << 8);
    calib_p8 = static_cast<std::int16_t>(reg_data[14] | reg_data[15] << 8);
    calib_p9 = static_cast<std::int16_t>(reg_data[16] | reg_data[17] << 8);
    reg_data.resize(3);
    for (int i = 0; i < 3; i++) {
        reg_data[i] = get_registor(address::data::pressure + i);
    }
    std::uint32_t pres_reg = 0;
    pres_reg = reg_data[0] << 12 | reg_data[1] << 4 | (reg_data[2] & 0xF0) >> 4;
    double var1 = 0.0;
    double var2 = 0.0;
    double var3 = 0.0;
    double pressure = 0.0;
    var1 = t_fine_ / 2.0 - 64000.0;
    var2 = var1 * var1 * calib_p6 / 32768.0;
    var2 = var2 + var1 * calib_p5 * 2.0;
    var2 = var2 / 4.0 + calib_p4 * 65536.0;
    var3 = calib_p3 * var1 * var1 / 524288.0;
    var1 = (var3 + calib_p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * calib_p1;
    if (var1 > 0.0) {
        pressure = 1048576.0 - pres_reg;
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
        var1 = calib_p9 * pressure * pressure / 2147483648.0;
        var2 = pressure * calib_p8 / 32768.0;
        pressure = pressure + (var1 + var2 + calib_p7) / 16.0;
        pressure /= 100.0;
        if (pressure > limit::pressure::max) {
            pressure = limit::pressure::max;
        } else if (pressure < limit::pressure::min) {
            pressure = limit::pressure::min;
        }
    } else {
        pressure = limit::pressure::min;
    }
    logger->debug("bme280 read pressure done.");
    return pressure;
}
double bme280::read_humidity() {
    logger->debug("bme280 read humidity start.");
    std::uint8_t calib_h1 = 0;
    std::int16_t calib_h2 = 0;
    std::uint8_t calib_h3 = 0;
    std::int16_t calib_h4 = 0;
    std::int16_t calib_h5 = 0;
    std::int8_t calib_h6 = 0;
    std::vector<std::uint8_t> reg_data(8, 0);
    reg_data[0] = get_registor(address::calib::humidity_1);
    for (int i = 0; i < 7; i++) {
        reg_data[i + 1] = get_registor(address::calib::humidity + i);
    }
    calib_h1 = static_cast<std::uint8_t>(reg_data[0]);
    calib_h2 = static_cast<std::int16_t>(reg_data[1] | reg_data[2] << 8);
    calib_h3 = static_cast<std::uint8_t>(reg_data[3]);
    calib_h4 =
        static_cast<std::int16_t>(reg_data[4] << 4 | (reg_data[5] & 0x0F));
    calib_h5 =
        static_cast<std::int16_t>((reg_data[5] & 0xF0) >> 4 | reg_data[6] << 4);
    calib_h6 = static_cast<std::int8_t>(reg_data[7]);
    reg_data.resize(2);
    for (int i = 0; i < 2; i++) {
        reg_data[i] = get_registor(address::data::humidity + i);
    }
    std::uint16_t hum_reg = reg_data[0] << 8 | reg_data[1];
    double var1 = 0.0;
    double var2 = 0.0;
    double var3 = 0.0;
    double var4 = 0.0;
    double var5 = 0.0;
    double var6 = 0.0;
    double humidity = 0.0;
    var1 = t_fine_ - 76800.0;
    var2 = calib_h4 * 64.0 + calib_h5 / 16384.0 * var1;
    var3 = hum_reg - var2;
    var4 = calib_h2 / 65536.0;
    var5 = (1.0 + calib_h3 / 67108864.0 * var1);
    var6 = 1.0 + calib_h6 / 67108864.0 * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - calib_h1 * var6 / 524288.0);
    if (humidity > limit::humidity::max) {
        humidity = limit::humidity::max;
    } else if (humidity < limit::humidity::min) {
        humidity = limit::humidity::min;
    }
    logger->debug("bme280 read humidity done.");
    return humidity;
}

void bme280::set_registor(std::uint8_t addr, std::uint8_t data) {
    std::vector<std::bitset<8>> tx_buf(2, 0);
    tx_buf[0] = 0x7F & addr;
    tx_buf[1] = data;
    spidev_->transfer(tx_buf);
}

std::uint8_t bme280::get_registor(std::uint8_t addr) {
    std::vector<std::bitset<8>> tx_buf(2, 0);
    std::vector<std::bitset<8>> rx_buf(2, 0);
    tx_buf[0] = 0x80 | addr;
    rx_buf = spidev_->transfer(tx_buf);
    return static_cast<std::uint8_t>(rx_buf[1].to_ulong());
}

void bme280::reset() {
    logger->debug("bme280 resetting.");
    set_registor(address::reset, 0xB6);
    std::uint8_t status = 0;
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        status = get_registor(address::status);
    } while ((status & 0x01) != 0);
    logger->debug("bme280 reset done.");
}

void bme280::set_settings() {
    logger->debug("bme280 settings start.");
    std::uint8_t config_reg = 0;
    std::uint8_t ctrl_meas_reg = 0;
    std::uint8_t ctrl_hum_reg = 0;
    config_reg = static_cast<std::uint8_t>(standby_) |
                 static_cast<std::uint8_t>(filter_);
    ctrl_meas_reg = static_cast<std::uint8_t>(osr_temp_) |
                    static_cast<std::uint8_t>(osr_pres_) |
                    static_cast<std::uint8_t>(mode_);
    ctrl_hum_reg = static_cast<std::uint8_t>(osr_hum_);
    set_registor(address::config, config_reg);
    set_registor(address::ctrl_meas, ctrl_meas_reg);
    set_registor(address::ctrl_hum, ctrl_hum_reg);
    logger->debug("bme280 settings done.");
}

}  // namespace driver::device::bme280
