#ifndef SPI_HPP
#define SPI_HPP

#include <bitset>
#include <cstdint>
#include <memory>
#include <vector>

#include "spdlog/logger.h"
#include "spdlog/spdlog.h"

namespace driver::spi {

enum class active_high : std::uint8_t {
    disable = 0x00,
    enable = 0x04,
};

class spi final {
public:
    spi(std::uint8_t slave_number, std::uint32_t clock_speed,
        std::uint8_t mode_number, active_high active_high);
    virtual ~spi() noexcept = default;
    void initialize();
    void finalize() const;
    std::vector<std::bitset<8>> transfer(std::vector<std::bitset<8>> data);

private:
    std::shared_ptr<spdlog::logger> logger;
    std::uint8_t spidev_;
    std::uint8_t slave_;
    std::uint32_t clock_;
    std::uint8_t mode_;
    active_high active_;
};

}  // namespace driver::spi

#endif
