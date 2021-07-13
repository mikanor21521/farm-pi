#include <bitset>
#include <cstdint>
#include <iostream>
#include <vector>

#include "pigpio/pigpio.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"
#include "driver/i2c.hpp"

namespace driver::i2c {

i2c::i2c(std::uint8_t bus_number, std::uint8_t address) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("i2c bus number: {}", bus_number);
    bus_ = bus_number;
    logger->trace("i2c address: {}", address);
    if (address > 0x7F) {
        logger->error("i2c initialization failed. address not 0x00-0x7F.");
        throw driver_ex("i2c initialization failed. address not 0x00-0x7F.");
    }
    addr_ = address;
    i2cdev_ = 0x00;
}

void i2c::initialize() {
    logger->debug("i2c initialization start.");
    std::int8_t ret = 0;
    ret = i2cOpen(bus_, addr_, 0x00);
    if (ret < 0) {
        logger->error("i2c initialization failed. return code: {}", ret);
        throw driver_ex("i2c initialization failed.", ret);
    }
    i2cdev_ = ret;
    logger->debug("i2c initialization done.");
}

void i2c::finalize() const {
    logger->debug("i2c finalization start.");
    std::int8_t ret = 0;
    ret = i2cClose(i2cdev_);
    if (ret < 0) {
        logger->error("i2c finalization failed. return code {}", ret);
        throw driver_ex("i2c finalization failed.", ret);
    }
    logger->debug("i2c finalization done.");
}

void i2c::write(std::vector<std::bitset<8>> data) {
    logger->debug("i2c write start.");
    std::int8_t ret = 0;
    std::uintmax_t length = data.size();
    char* buffer = new char[length];
    for (std::uintmax_t i = 0; i < length; i++) {
        buffer[i] = static_cast<char>(data.at(i).to_ulong());
    }
    ret = i2cWriteDevice(i2cdev_, buffer, length);
    if (ret < 0) {
        logger->error("i2c write failed. return code: {}", ret);
        throw driver_ex("i2c write failed.", ret);
    }
    delete[] buffer;
    logger->debug("i2c write done.");
}

std::vector<std::bitset<8>> i2c::read(std::uintmax_t length) {
    logger->debug("i2c read start.");
    std::int8_t ret = 0;
    char* buffer = new char[length];
    ret = i2cReadDevice(i2cdev_, buffer, length);
    if (ret < 0) {
        logger->error("i2c read failed. return code: {}", ret);
        throw driver_ex("i2c read failed.", ret);
    }
    std::vector<std::bitset<8>> data;
    for (uintmax_t i = 0; i < length; i++) {
        data.emplace_back(buffer[i]);
    }
    delete[] buffer;
    logger->debug("i2c read done.");
    return data;
}
}  // namespace driver::i2c
