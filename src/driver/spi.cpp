#include "driver.hpp"

#include <bitset>
#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "pigpio.h"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

std::shared_ptr<driver::Spi> Driver::createSpi(std::uint8_t cs_pin,
                                               std::uint32_t clock_speed,
                                               std::uint8_t cpol,
                                               std::uint8_t cpha,
                                               bool active_high) {
    std::string logger_name = "spi";
    auto logger = logger_->clone(fmt::format("{}-{}", logger_name, spi_count_));
    spi_count_++;
    auto spi = std::make_shared<driver::Spi>(logger, cs_pin, clock_speed, cpol,
                                             cpha, active_high);
    return spi;
}

namespace driver {

std::mutex Spi::mtx_;

Spi::Spi(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t cs_pin,
         std::uint32_t clock_speed, std::uint8_t cpol, std::uint8_t cpha,
         bool active_high) noexcept
    : cs_pin_(cs_pin),
      clock_speed_(clock_speed),
      cpol_(cpol),
      cpha_(cpha),
      active_high_(active_high),
      spidev_(0x00) {
    assert(cs_pin == 0 || cs_pin == 1);
    assert(32000 < clock_speed && clock_speed < 125000000);
    assert(cpol == 0 || cpol == 1);
    assert(cpha == 0 || cpol == 1);
    logger_ = logger;
    logger_->debug("spi cs pin number: {}", cs_pin);
    logger_->debug("spi clock speed: {}", clock_speed);
    logger_->debug("spi clock polarity: {}", cpol);
    logger_->debug("spi clock phase: {}", cpha);
    logger_->debug("spi active high: {}", active_high);
}

void Spi::initialize() {
    logger_->info("spi initialization start.");
    const int spi_flag_size = 22;
    int ret = 0;
    std::bitset<spi_flag_size> flag;
    if (cpha_ == 1) {
        flag.set(0);
    }
    if (cpol_ == 1) {
        flag.set(1);
    }
    if (active_high_) {
        flag.set(2 + cs_pin_);
    }
    logger_->debug("spi initialize flag: {:#024b}", flag.to_ulong());
    ret = spiOpen(cs_pin_, clock_speed_, flag.to_ulong());
    if (ret < 0) {
        logger_->error("spi initialization failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("spi initialization failed. status code: {}", ret);
    }
    spidev_ = ret;
    logger_->info("spi initialization done.");
}

void Spi::finalize() const {
    logger_->info("spi finalization start.");
    int ret = 0;
    ret = spiClose(spidev_);
    if (ret < 0) {
        logger_->error("spi finalization failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("spi finalization failed. status code: {}", ret);
    }
    logger_->info("spi finalization done.");
}

void Spi::write(std::vector<std::uint8_t> data) const {
    std::lock_guard<std::mutex> lock(mtx_);
    logger_->info("spi data write start.");
    logger_->info("spi data written is [{:#04x}]", fmt::join(data, ", "));
    int ret = 0;
    std::uintmax_t length = data.size();
    std::vector<char> buffer(length);
    for (std::uintmax_t i = 0; i < length; i++) {
        buffer.at(i) = static_cast<char>(data.at(i));
    }
    ret = spiWrite(spidev_, buffer.data(), length);
    if (ret < 0) {
        logger_->error("spi data write failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("spi data write failed. status code: {}", ret);
    }
    logger_->info("spi data write done.");
}

std::vector<std::uint8_t> Spi::read(std::uintmax_t length) const {
    std::lock_guard<std::mutex> lock(mtx_);
    logger_->info("spi data read start.");
    int ret = 0;
    std::vector<char> buffer(length);
    ret = spiRead(spidev_, buffer.data(), length);
    if (ret < 0) {
        logger_->error("spi data read failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("spi data read failed. status code: {}", ret);
    }
    std::vector<std::uint8_t> data(length);
    for (std::uintmax_t i = 0; i < length; i++) {
        data.at(i) = static_cast<std::uint8_t>(buffer.at(i));
    }
    logger_->info("spi data read is [{:#04x}]", fmt::join(data, ", "));
    logger_->info("spi data read done.");
    return data;
}

std::vector<std::uint8_t> Spi::transfer(std::vector<std::uint8_t> data) const {
    std::lock_guard<std::mutex> lock(mtx_);
    logger_->info("spi data transfer start.");
    logger_->info("spi data written is [{:#04x}]", fmt::join(data, ", "));
    int ret = 0;
    std::uintmax_t length = data.size();
    std::vector<char> tx_buf(length);
    std::vector<char> rx_buf(length);
    for (std::uintmax_t i = 0; i < length; i++) {
        tx_buf.at(i) = static_cast<char>(data.at(i));
    }
    ret = spiXfer(spidev_, tx_buf.data(), rx_buf.data(), length);
    if (ret < 0) {
        logger_->error("spi data transfer failed.");
        logger_->debug("status code: {}");
        throwDriverException("spi data transfer failed. status code: {}", ret);
    }
    for (std::uintmax_t i = 0; i < length; i++) {
        data.at(i) = static_cast<std::uint8_t>(rx_buf.at(i));
    }
    logger_->info("spi data read is [{:#04x}]", fmt::join(data, ", "));
    logger_->info("spi data transfer done.");
    return data;
}

}  // namespace driver
