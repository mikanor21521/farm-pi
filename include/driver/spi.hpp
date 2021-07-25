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

class spi {
public:
    spi();
    virtual void initialize() = 0;
    virtual void finalize() const = 0;
    virtual std::vector<std::bitset<8>> transfer(
        std::vector<std::bitset<8>> data) = 0;

protected:
    std::shared_ptr<spdlog::logger> logger;
};

class main_spi final : public spi {
public:
    main_spi(std::uint8_t slave_number, std::uint32_t clock_speed,
             std::uint8_t mode_number, active_high active_high);
    virtual ~main_spi() noexcept = default;
    void initialize() override;
    void finalize() const override;
    std::vector<std::bitset<8>> transfer(
        std::vector<std::bitset<8>> data) override;

private:
    std::uint8_t spidev_;
    std::uint8_t slave_;
    std::uint32_t clock_;
    std::uint8_t mode_;
    active_high active_;
};

class bit_banging_spi final : public spi {
public:
    bit_banging_spi(std::uint8_t ss_pin, std::uint8_t miso_pin,
                    std::uint8_t mosi_pin, std::uint8_t sck_pin,
                    std::uint32_t clock_speed, std::uint8_t mode,
                    active_high active_high);
    void initialize() override;
    void finalize() const override;
    std::vector<std::bitset<8>> transfer(
        std::vector<std::bitset<8>> data) override;

private:
    std::uint8_t ss_pin_;
    std::uint8_t miso_pin_;
    std::uint8_t mosi_pin_;
    std::uint8_t sck_pin_;
    std::uint32_t clock_speed_;
    std::uint8_t mode_;
    active_high active_;
};

}  // namespace driver::spi

#endif
