#ifndef DEVICE_MCP3002_HPP
#define DEVICE_MCP3002_HPP

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver.hpp"

class Mcp3002 {
public:
    enum class Channel {
        single0,
        single1,
        differential0,
        differential1,
    };
    Mcp3002(std::shared_ptr<spdlog::logger> logger,
            std::shared_ptr<driver::Spi> spi, double base_voltage);
    void initialize();
    void finalize();
    void measure(Channel channel);
    [[nodiscard]] std::uint16_t getCount(Channel channel);
    [[nodiscard]] double getVoltage(Channel channel);

private:
    std::shared_mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<driver::Spi> spi_;
    double base_voltage_;
    std::uint16_t single0_count_;
    std::uint16_t single1_count_;
    std::uint16_t differential0_count_;
    std::uint16_t differential1_count_;
};

std::shared_ptr<Mcp3002> createMcp3002(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Spi>& spi, double base_voltage);

#endif
