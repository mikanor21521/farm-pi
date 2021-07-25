#include <bitset>
#include <cstdint>
#include <iostream>
#include <vector>

#include "pigpio/pigpio.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"
#include "driver/spi.hpp"

namespace driver::spi {

spi::spi() {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
}

main_spi::main_spi(std::uint8_t slave_number, std::uint32_t clock_speed,
                   std::uint8_t mode_number, active_high active_high)
    : spi() {
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

void main_spi::initialize() {
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

void main_spi::finalize() const {
    logger->debug("spi finalization start.");
    std::int16_t ret = 0;
    ret = spiClose(spidev_);
    if (ret < 0) {
        logger->error("spi finalization failed. return code {}", ret);
        throw driver_ex("spi finalization failed.", ret);
    }
    logger->debug("spi finalization done.");
}

std::vector<std::bitset<8>> main_spi::transfer(
    std::vector<std::bitset<8>> data) {
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

bit_banging_spi::bit_banging_spi(std::uint8_t ss_pin, std::uint8_t miso_pin,
                                 std::uint8_t mosi_pin, std::uint8_t sck_pin,
                                 std::uint32_t clock_speed, std::uint8_t mode,
                                 active_high active_high)
    : spi() {
    logger->trace("slave select pin: {}", ss_pin);
    ss_pin_ = ss_pin;
    logger->trace("master in slave out pin: {}", miso_pin);
    miso_pin_ = miso_pin;
    logger->trace("master out slave in pin: {}", mosi_pin);
    mosi_pin_ = mosi_pin;
    logger->trace("serial clock pin: {}", sck_pin);
    sck_pin_ = sck_pin;
    logger->trace("clock speed: {}", clock_speed);
    clock_speed_ = clock_speed;
    logger->trace("spi mode: {}", mode);
    mode_ = mode;
    logger->trace("active high: {}", static_cast<std::uint8_t>(active_high));
    active_ = active_high;
}

void bit_banging_spi::initialize() {
    logger->debug("spi initialization start.");
    std::int8_t ret = 0;
    std::bitset<22> spi_flag = 0;
    spi_flag |= mode_;
    spi_flag |= static_cast<std::uint8_t>(active_);
    ret = bbSPIOpen(ss_pin_, miso_pin_, mosi_pin_, sck_pin_, clock_speed_,
                    spi_flag.to_ulong());
    if (ret < 0) {
        logger->error("spi initialization failed. return code: {}", ret);
        throw driver_ex("spi initialization failed.", ret);
    }
    logger->debug("spi initialization done.");
}

void bit_banging_spi::finalize() const {
    logger->debug("spi finalization start.");
    std::int8_t ret = 0;
    ret = bbSPIClose(ss_pin_);
    if (ret < 0) {
        logger->error("spi finalization failed. return code: {}", ret);
        throw driver_ex("spi finalization failed.", ret);
    }
    logger->debug("spi finalization done.");
}

std::vector<std::bitset<8>> bit_banging_spi::transfer(
    std::vector<std::bitset<8>> data) {
    logger->debug("spi transfer start.");
    std::int8_t ret = 0;
    std::uintmax_t length = data.size();
    char* tx_buffer = new char[length];
    char* rx_buffer = new char[length];
    for (std::uintmax_t i = 0; i < length; i++) {
        tx_buffer[i] = static_cast<std::uint8_t>(data.at(i).to_ulong());
        rx_buffer[i] = 0x00;
    }
    ret = bbSPIXfer(ss_pin_, tx_buffer, rx_buffer, length);
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
