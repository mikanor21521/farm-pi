#include <bitset>
#include <cstdint>
#include <iostream>
#include <vector>

#include "pigpio/pigpio.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"
#include "driver/spi.hpp"

namespace driver::spi {

spi::spi(std::uint8_t slave_number, std::uint32_t clock_speed,
         std::uint8_t mode_number, active_high active_high) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("spi slave number: {}", slave_number);
    if (slave_number > 1) {
        logger->error(
            "spi initialization failed. main spi slave number not 0-1.");
        throw driver_ex(
            "spi initialization failed. main spi slave number not 0-1.");
    }
    slave_ = slave_number;
    logger->trace("spi clock speed: {}", clock_speed);
    if (clock_speed < 32000 || clock_speed > 125000000) {
        logger->error(
            "spi initialization failed. spi clock speed not 32K-125M.");
        throw driver_ex(
            "spi initialization failed. spi clock speed not 32K-125M.");
    }
    clock_ = clock_speed;
    logger->trace("spi mode number: {}", mode_number);
    if (mode_number > 4) {
        logger->error("spi initialization failed. spi mode number not 0-4");
        throw driver_ex("spi initialization failed. spi mode number not 0-4");
    }
    mode_ = mode_number;
    logger->trace("spi active high: {}",
                  static_cast<std::uint8_t>(active_high));
    if (active_high != active_high::enable &&
        active_high != active_high::disable) {
        logger->error(
            "spi initialization failed. spi active high not enable/disable.");
        throw driver_ex(
            "spi initialization failed. spi active high not enable/disable.");
    }
    active_ = active_high;
    spidev_ = 0x00;
}

void spi::initialize() {
    logger->debug("spi initialization start.");
    std::int8_t ret = 0;
    std::bitset<22> spi_flag = 0;
    spi_flag |= mode_;
    spi_flag |= static_cast<uint8_t>(active_) << slave_;
    ret = spiOpen(slave_, clock_, spi_flag.to_ulong());
    if (ret < 0) {
        logger->error("spi initialization failed. return code: {}", spidev_);
        throw driver_ex("spi initialization failed.", spidev_);
    }
    spidev_ = ret;
    logger->debug("spi initialization done.");
}

void spi::finalize() const {
    logger->debug("spi finalization start.");
    std::int16_t ret = 0;
    ret = spiClose(spidev_);
    if (ret < 0) {
        logger->error("spi finalization failed. return code {}", ret);
        throw driver_ex("spi finalization failed.", ret);
    }
    logger->debug("spi finalization done.");
}

std::vector<std::bitset<8>> spi::transfer(std::vector<std::bitset<8>> data) {
    logger->debug("spi transfer start.");
    std::int16_t ret = 0;
    std::uintmax_t length = data.size();
    char* tx_buffer = new char[length];
    char* rx_buffer = new char[length];
    for (std::uintmax_t i = 0; i < length; i++) {
        tx_buffer[i] = static_cast<std::uint8_t>(data.at(i).to_ulong());
        rx_buffer[i] = 0x00;
    }
    ret = spiXfer(spidev_, tx_buffer, rx_buffer, length);
    if (ret < 0) {
        logger->error("spi transfer failed. return code: {}", ret);
        throw driver_ex("spi transfer failed.", ret);
    }
    for (std::uintmax_t i = 0; i < length; i++) {
        data.at(i) = rx_buffer[i];
    }
    delete[] tx_buffer;
    delete[] rx_buffer;
    logger->debug("spi transfer done.");
    return data;
}

}  // namespace driver::spi
