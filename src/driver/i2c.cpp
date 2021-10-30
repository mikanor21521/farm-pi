#include "driver.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pigpio.h"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

namespace alcor {

std::unique_ptr<driver::I2c> Driver::createI2c(std::uint8_t bus_number,
                                               std::uint8_t address) {
    std::string logger_name = "i2c";
    auto logger = logger_->clone(fmt::format("{}-{}", logger_name, i2c_count_));
    i2c_count_++;
    auto i2c = std::make_unique<driver::I2c>(logger, bus_number, address);
    return i2c;
}

namespace driver {

I2c::I2c(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t bus_number,
         std::uint8_t address) noexcept
    : bus_number_(bus_number), address_(address), i2cdev_(0x00) {
    assert(bus_number == 0 || bus_number == 1);
    assert(address <= 0x7f);
    logger_ = logger;
    logger_->debug("i2c bus number: {}", bus_number);
    logger_->debug("i2c address: {:#04x}", address);
}

void I2c::initialize() {
    logger_->info("i2c initialization start.");
    int ret = 0;
    std::uint8_t flag = 0;
    ret = i2cOpen(bus_number_, address_, flag);
    if (ret < 0) {
        logger_->error("i2c initialization failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("i2c initialization failed. status code: {}", ret);
    }
    i2cdev_ = ret;
    logger_->info("i2c initialization done.");
}

void I2c::finalize() const {
    logger_->info("i2c finalization start.");
    int ret = 0;
    ret = i2cClose(i2cdev_);
    if (ret < 0) {
        logger_->error("i2c finalization failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("i2c finalization failed. status code: {}", ret);
    }
    logger_->info("i2c finalization done.");
}

void I2c::write(std::vector<std::uint8_t> data) const {
    logger_->info("i2c data write start.");
    logger_->info("i2c data written is [{:#04x}]", fmt::join(data, ", "));
    int ret = 0;
    std::uintmax_t length = data.size();
    std::vector<char> buffer(length);
    for (std::uintmax_t i = 0; i < length; i++) {
        buffer.at(i) = static_cast<char>(data.at(i));
    }
    ret = i2cWriteDevice(i2cdev_, buffer.data(), length);
    if (ret < 0) {
        logger_->error("i2c data write failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("i2c data write failed. status code: {}", ret);
    }
    logger_->info("i2c data write done.");
}

std::vector<std::uint8_t> I2c::read(std::uintmax_t length) const {
    logger_->info("i2c data read start.");
    int ret = 0;
    std::vector<char> buffer(length);
    ret = i2cReadDevice(i2cdev_, buffer.data(), length);
    if (ret < 0) {
        logger_->error("i2c data read failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("i2c data read failed. status code: {}", ret);
    }
    std::vector<std::uint8_t> data(length);
    for (std::uintmax_t i = 0; i < length; i++) {
        data.at(i) = static_cast<std::uint8_t>(buffer.at(i));
    }
    logger_->info("i2c data read is [{:#04x}]", fmt::join(data, ", "));
    logger_->info("i2c data read done.");
    return data;
}

}  // namespace driver

}  // namespace alcor
