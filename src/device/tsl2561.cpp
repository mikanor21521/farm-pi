#include "device/tsl2561.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "driver.hpp"

namespace {

constexpr std::uint8_t register_control = 0x80;
constexpr std::uint8_t register_timing = 0x81;
constexpr std::uint8_t register_chip_id = 0x8A;
constexpr std::uint8_t register_channel0 = 0x8C;
constexpr std::uint8_t register_channel1 = 0x8E;

constexpr std::uint8_t gain_x1 = 0x00;
constexpr std::uint8_t gain_x16 = 0x10;

constexpr std::uint8_t integral_ms13 = 0x00;
constexpr std::uint8_t integral_ms101 = 0x01;
constexpr std::uint8_t integral_ms402 = 0x02;

constexpr std::uint8_t power_off = 0x00;
constexpr std::uint8_t power_on = 0x03;

constexpr std::uint8_t chip_id = 0x10;

}  // namespace

std::shared_ptr<Tsl2561> createTsl2561(
    const std::shared_ptr<spdlog::logger> &logger,
    const std::shared_ptr<driver::I2c> &i2c, Tsl2561::Gain gain,
    Tsl2561::Integral integral, Tsl2561::Package package) {
    static int count = 0;
    std::string logger_name = "tsl2561";
    count++;
    return std::make_shared<Tsl2561>(logger, i2c, gain, integral, package);
}

Tsl2561::Tsl2561(std::shared_ptr<spdlog::logger> logger,
                 std::shared_ptr<driver::I2c> i2c, Gain gain, Integral integral,
                 Package package)
    : logger_(std::move(logger)),
      i2c_(std::move(i2c)),
      gain_(gain),
      integral_(integral),
      package_(package),
      channel0_count_(0),
      channel1_count_(0) {
    assert(gain == Gain::x1 || gain == Gain::x16);
    assert(integral == Integral::ms13 || integral == Integral::ms101 ||
           integral == Integral::ms402);
    assert(package == Package::CS || package == Package::FN);
}

void Tsl2561::initialize() {
    logger_->info("tsl2561 initialization start.");
    i2c_->initialize();
    if (!isChipIdValid()) {
        logger_->error("invalid chip id");
        driver::throwDriverException("invalid chip id");
    }
    setSettings();
    logger_->info("tsl2561 initialization done.");
}

void Tsl2561::finalize() {
    logger_->info("tsl2561 finalization start.");
    i2c_->finalize();
    logger_->info("tsl2561 finalization done.");
}

void Tsl2561::measure() {
    std::lock_guard<std::shared_mutex> lock(mtx_);
    powerOn();
    switch (integral_) {
        case Integral::ms13:
            std::this_thread::sleep_for(std::chrono::milliseconds(14));
            break;
        case Integral::ms101:
            std::this_thread::sleep_for(std::chrono::milliseconds(102));
            break;
        case Integral::ms402:
            std::this_thread::sleep_for(std::chrono::milliseconds(403));
            break;
        case Integral::custom:
            return;
    }
    auto data_ch0 = getReg(register_channel0, 2);
    auto data_ch1 = getReg(register_channel1, 2);
    powerOff();
    channel0_count_ = data_ch0[0] | data_ch0[1] << 8;
    channel1_count_ = data_ch1[0] | data_ch1[1] << 8;
}

std::uint16_t Tsl2561::getChannel0Count() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return channel0_count_;
}

std::uint16_t Tsl2561::getChannel1Count() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return channel1_count_;
}

double Tsl2561::getIlluminance() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    double illuminance = 0.0;
    if (channel0_count_ > 0) {
        double result = static_cast<double>(channel1_count_) /
                        static_cast<double>(channel0_count_);
        switch (package_) {
            case Package::CS:
                if (result <= 0.52) {
                    illuminance = 0.0315 * channel0_count_ -
                                  0.0593 * channel0_count_ * pow(result, 1.4);
                } else if (result <= 0.65) {
                    illuminance =
                        0.0229 * channel0_count_ - 0.0291 * channel1_count_;
                } else if (result <= 0.80) {
                    illuminance =
                        0.0157 * channel0_count_ - 0.0180 * channel1_count_;
                } else if (result <= 1.30) {
                    illuminance =
                        0.00338 * channel0_count_ - 0.00260 * channel1_count_;
                } else {
                    illuminance = 0.0;
                }
                break;
            case Package::FN:
                if (result <= 0.50) {
                    illuminance = 0.0304 * channel0_count_ -
                                  0.062 * channel0_count_ * pow(result, 1.4);
                } else if (result <= 0.61) {
                    illuminance =
                        0.0224 * channel0_count_ - 0.031 * channel1_count_;
                } else if (result <= 0.80) {
                    illuminance =
                        0.0128 * channel0_count_ - 0.0153 * channel1_count_;
                } else if (result <= 1.30) {
                    illuminance =
                        0.00146 * channel0_count_ - 0.00112 * channel1_count_;
                } else {
                    illuminance = 0.0;
                }
                break;
        }
    } else {
        illuminance = 0.0;
    }
    if (gain_ == Gain::x1) {
        illuminance *= 16;
    }
    if (integral_ == Integral::ms13) {
        illuminance *= 402.0 / 13.7;
    } else if (integral_ == Integral::ms101) {
        illuminance *= 402.0 / 101.0;
    }
    return illuminance;
}

void Tsl2561::powerOn() const {
    setReg(register_control, power_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void Tsl2561::powerOff() const {
    setReg(register_control, power_off);
}

void Tsl2561::setSettings() const {
    std::uint8_t settings = 0x00;
    switch (gain_) {
        case Gain::x1:
            settings |= gain_x1;
            break;
        case Gain::x16:
            settings |= gain_x16;
            break;
    }
    switch (integral_) {
        case Integral::ms13:
            settings |= integral_ms13;
            break;
        case Integral::ms101:
            settings |= integral_ms101;
            break;
        case Integral::ms402:
            settings |= integral_ms402;
            break;
        case Integral::custom:
            return;
    }
    setReg(register_timing, settings);
}

bool Tsl2561::isChipIdValid() const {
    auto data = getReg(register_chip_id, 1);
    return data[0] == chip_id;
}

void Tsl2561::setReg(std::uint8_t address, std::uint8_t data) const {
    i2c_->write({address, data});
}

std::vector<std::uint8_t> Tsl2561::getReg(std::uint8_t address,
                                          std::uintmax_t length) const {
    i2c_->write({address});
    return i2c_->read(length);
}
