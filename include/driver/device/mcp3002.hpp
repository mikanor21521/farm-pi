#ifndef MCP3002_HPP
#define MCP3002_HPP

#include <cstdint>
#include <memory>

#include "spdlog/spdlog.h"

#include "driver/spi.hpp"

namespace driver::device::mcp3002 {

enum class channel : std::uint8_t {
    differential_0 = 0x00,
    differential_1 = 0x01,
    single_0 = 0x02,
    single_1 = 0x03,
};

class mcp3002 final {
public:
    mcp3002(const spi::spi& spi_device, double base_voltage);
    ~mcp3002() noexcept;
    void initialize() const;
    void finalize() const;
    double read_voltage(channel ch);

private:
    std::shared_ptr<spdlog::logger> logger;
    spi::spi* spidev_;
    double base_;
};

}  // namespace driver::device::mcp3002

#endif
