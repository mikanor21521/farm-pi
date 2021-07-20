#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver/device/tsl2561.hpp"
#include "driver/driver.hpp"
#include "driver/i2c.hpp"

namespace driver::device::tsl2561 {

const std::uint8_t address_low = 0x29;
const std::uint8_t address_float = 0x39;
const std::uint8_t address_high = 0x49;

namespace {

namespace address {

constexpr std::uint8_t control = 0x80;
constexpr std::uint8_t timing = 0x81;
constexpr std::uint8_t chip_id = 0x8A;
constexpr std::uint8_t channel0_data = 0x8C;
constexpr std::uint8_t channel1_data = 0x8E;

}  // namespace address

namespace command {

constexpr std::uint8_t power_off = 0x00;
constexpr std::uint8_t power_on = 0x03;

}  // namespace command

constexpr std::uint8_t chip_id = 0x10;

}  // namespace

tsl2561::tsl2561(const i2c::i2c& i2c_device, gain gain, integral integral) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("tsl2561 gain: {}", gain);
    if (gain != gain::x1 && gain != gain::x16) {
        logger->error("tsl2561 initialization failed. gain is invalid.");
        throw driver_ex("tsl2561 initialization failed. gain is invalid.");
    }
    gain_ = gain;
    logger->trace("tsl2561 integral: {}", integral);
    if (integral != integral::ms13 && integral != integral::ms101 &&
        integral != integral::ms402) {
        logger->error("tsl2561 initialization failed. integral is invalid.");
        throw driver_ex("tsl2561 initialization failed. integral is invalid.");
    }
    integral_ = integral;
    i2cdev_ = new i2c::i2c(i2c_device);
}

tsl2561::~tsl2561() noexcept {
    delete i2cdev_;
}

void tsl2561::initialize() const {
    logger->debug("tsl2561 initialization start.");
    i2cdev_->initialize();
    logger->debug("tsl2561 i2c device initialization done.");
    auto data = get_reg(address::chip_id, 1);
    logger->trace("tsl2561 chip id: {}", data[0]);
    if ((data[0] & chip_id) == 0) {
        logger->error("tsl2561 initialization failed. chip id is invalid.");
        throw driver_ex("tsl2561 initialization failed. chip id is invalid.");
    }
    power_on();
    set_reg(address::timing, static_cast<std::uint8_t>(gain_) |
                                 static_cast<std::uint8_t>(integral_));
    power_off();
    logger->debug("tsl2561 initialization done.");
}

void tsl2561::finalize() const {
    logger->debug("tsl2561 finalization start.");
    i2cdev_->finalize();
    logger->debug("tsl2561 i2c device finalization done.");
    logger->debug("tsl2561 finalization done.");
}

double tsl2561::read_illuminance() const {
    logger->debug("tsl2561 read illuminance start.");
    power_on();
    switch (integral_) {
        case integral::ms13: {
            std::this_thread::sleep_for(std::chrono::milliseconds(14));
        } break;
        case integral::ms101: {
            std::this_thread::sleep_for(std::chrono::milliseconds(102));
        } break;
        case integral::ms402: {
            std::this_thread::sleep_for(std::chrono::milliseconds(403));
        } break;
    }
    std::vector<std::uint8_t> data(2, 0);
    data = get_reg(address::channel0_data, 2);
    double channel0 = data[0] | data[1] << 8;
    data = get_reg(address::channel1_data, 2);
    double channel1 = data[0] | data[1] << 8;
    double illuminance = 0.0;
    if (channel0 != 0) {
        double result = channel1 / channel0;
#ifdef TSL2561_PACKAGE_CS
        if (result <= 0.52) {
            illuminance =
                0.0315 * channel0 - 0.0593 * channel0 * pow(result, 1.4);
        } else if (result <= 0.65) {
            illuminance = 0.0229 * channel0 - 0.0291 * channel1;
        } else if (result <= 0.80) {
            illuminance = 0.0157 * channel0 - 0.0180 * channel1;
        } else if (result <= 1.30) {
            illuminance = 0.00338 * channel0 - 0.00260 * channel1;
        } else {
            illuminance = 0.0;
        }
#elif TSL2561_PACKAGE_FN
        if (result <= 0.50) {
            illuminance =
                0.0304 * channel0 - 0.062 * channel0 * pow(result, 1.4);
        } else if (result <= 0.61) {
            illuminance = 0.0224 * channel0 - 0.031 * channel1;
        } else if (result <= 0.80) {
            illuminance = 0.0128 * channel0 - 0.0153 * channel1;
        } else if (result <= 1.30) {
            illuminance = 0.00146 * channel0 - 0.00112 * channel1;
        } else {
            illuminance = 0.0;
        }
#else
#error Choose the CS package or the FN package.
#endif
    } else {
        illuminance = 0.0;
    }
    if (gain_ == gain::x1) {
        illuminance *= 16;
    }
    switch (integral_) {
        case integral::ms13: {
            illuminance *= 402.0 / 13.7;
        } break;
        case integral::ms101: {
            illuminance *= 402.0 / 101.0;
        } break;
        case integral::ms402:
            break;
    }
    logger->debug("tsl2561 read illuminance done.");
    return illuminance;
}

void tsl2561::power_on() const {
    set_reg(address::control, command::power_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void tsl2561::power_off() const {
    set_reg(address::control, command::power_off);
}

void tsl2561::set_reg(std::uint8_t address, std::uint8_t data) const {
    std::vector<std::bitset<8>> tx_buf(2, 0);
    tx_buf[0] = address;
    tx_buf[1] = data;
    i2cdev_->write(tx_buf);
}

[[nodiscard]] std::vector<std::uint8_t> tsl2561::get_reg(
    std::uint8_t address, std::uintmax_t length) const {
    std::vector<std::bitset<8>> tx_buf(1, 0);
    std::vector<std::bitset<8>> rx_buf(length, 0);
    tx_buf[0] = address;
    i2cdev_->write(tx_buf);
    rx_buf = i2cdev_->read(length);
    std::vector<std::uint8_t> data(length, 0);
    for (std::uintmax_t i = 0; i < length; i++) {
        data[i] = rx_buf[i].to_ulong();
    }
    return data;
}

}  // namespace driver::device::tsl2561
