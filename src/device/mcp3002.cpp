#include "device/mcp3002.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "driver.hpp"

namespace {

constexpr std::uint8_t channel_single0 = 0x00;
constexpr std::uint8_t channel_single1 = 0x20;
constexpr std::uint8_t channel_differential0 = 0x40;
constexpr std::uint8_t channel_differential1 = 0x60;

}  // namespace

std::shared_ptr<Mcp3002> createMcp3002(
    const std::shared_ptr<spdlog::logger> &logger,
    const std::shared_ptr<driver::Spi> &spi, double base_voltage) {
    static int count = 0;
    std::string logger_name = "mcp3002";
    count++;
    return std::make_shared<Mcp3002>(
        logger->clone(fmt::format("{}-{}", logger_name, count)), spi,
        base_voltage);
}

Mcp3002::Mcp3002(std::shared_ptr<spdlog::logger> logger,
                 std::shared_ptr<driver::Spi> spi, double base_voltage)
    : logger_(std::move(logger)),
      spi_(std::move(spi)),
      base_voltage_(base_voltage),
      single0_count_(0),
      single1_count_(0),
      differential0_count_(0),
      differential1_count_(0) {
    assert(base_voltage > 0);
}

void Mcp3002::initialize() {
    logger_->info("mcp3002 initialization start.");
    spi_->initialize();
    logger_->info("mcp3002 initialization done.");
}

void Mcp3002::finalize() {
    logger_->info("mcp3002 finalization start.");
    spi_->finalize();
    logger_->info("mcp3002 finalization done.");
}

void Mcp3002::measure(Channel channel) {
    std::lock_guard<std::shared_mutex> lock(mtx_);
    std::vector<std::uint8_t> buf(2, 0x00);
    buf[0] = 0x90;
    switch (channel) {
        case Channel::single0:
            buf[0] |= channel_single0;
            break;
        case Channel::single1:
            buf[0] |= channel_single1;
            break;
        case Channel::differential0:
            buf[0] |= channel_differential0;
            break;
        case Channel::differential1:
            buf[0] |= channel_differential1;
            break;
    }
    buf = spi_->transfer(buf);
    std::uint16_t adc = (buf[0] & 0x03) << 7 | (buf[1] & 0xFE) >> 1;
    switch (channel) {
        case Channel::single0:
            single0_count_ = adc;
            break;
        case Channel::single1:
            single1_count_ = adc;
            break;
        case Channel::differential0:
            differential0_count_ = adc;
            break;
        case Channel::differential1:
            differential1_count_ = adc;
            break;
    }
}

std::uint16_t Mcp3002::getCount(Channel channel) {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::uint16_t count = 0;
    switch (channel) {
        case Channel::single0:
            count = single0_count_;
            break;
        case Channel::single1:
            count = single1_count_;
            break;
        case Channel::differential0:
            count = differential0_count_;
            break;
        case Channel::differential1:
            count = differential1_count_;
            break;
    }
    return count;
}

double Mcp3002::getVoltage(Channel channel) {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::uint16_t count = 0;
    switch (channel) {
        case Channel::single0:
            count = single0_count_;
            break;
        case Channel::single1:
            count = single1_count_;
            break;
        case Channel::differential0:
            count = differential0_count_;
            break;
        case Channel::differential1:
            count = differential1_count_;
            break;
    }
    return count * base_voltage_ / 1024.0;
}
