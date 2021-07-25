#include <bitset>
#include <cstdint>
#include <iostream>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver/device/mcp3002.hpp"
#include "driver/driver.hpp"
#include "driver/spi.hpp"

namespace driver::device::mcp3002 {

mcp3002::mcp3002(spi::spi* spi_device, double base_voltage) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("mcp3002 base voltage: {}", base_voltage);
    if (base_voltage < 0) {
        logger->error(
            "mcp3002 initialization failed. base voltage under zero.");
        throw driver_ex(
            "mcp3002 initialization failed. base voltage under zero.");
    }
    base_ = base_voltage;
    spidev_ = spi_device;
}

void mcp3002::initialize() const {
    logger->debug("mcp3002 initialization start.");
    spidev_->initialize();
    logger->debug("mcp3002 spi device initialization done.");
    logger->debug("mcp3002 initialization done.");
}

void mcp3002::finalize() const {
    logger->debug("mcp3002 finalization start.");
    spidev_->finalize();
    logger->debug("mcp3002 spi device finalization done.");
    logger->debug("mcp3002 finalization done.");
}

double mcp3002::read_voltage(channel ch) {
    logger->debug("mcp3002 read voltage start.");
    std::vector<std::bitset<8>> tx_buf(2, 0x00);
    tx_buf.at(0) = 0x90 | static_cast<std::uint8_t>(ch) << 5;
    auto rx_buf = spidev_->transfer(tx_buf);
    std::uint16_t adc = ((0x0600 & rx_buf.at(0).to_ulong() << 8) |
                         (0xFE & rx_buf.at(1).to_ulong())) >>
                        1;
    logger->trace("mcp3002 adc: {}", adc);
    double voltage = adc * base_ / 1024.0;
    logger->trace("mcp3002 voltage: {}", voltage);
    logger->debug("mcp3002 read voltage done.");
    return voltage;
}

}  // namespace driver::device::mcp3002
