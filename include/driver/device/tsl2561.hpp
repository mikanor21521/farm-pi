#ifndef TSL2561_HPP
#define TSL2561_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver/i2c.hpp"

namespace driver::device::tsl2561 {

extern const std::uint8_t address_low;
extern const std::uint8_t address_float;
extern const std::uint8_t address_high;

enum class gain : std::uint8_t {
    x1 = 0x00,
    x16 = 0x10,
};

enum class integral : std::uint8_t {
    ms13 = 0x00,
    ms101 = 0x01,
    ms402 = 0x02,
};

class tsl2561 final {
public:
    tsl2561(const i2c::i2c& i2c_device, gain gain, integral integral);
    ~tsl2561() noexcept;
    void initialize() const;
    void finalize() const;
    [[nodiscard]] double read_illuminance() const;

private:
    std::shared_ptr<spdlog::logger> logger;
    i2c::i2c* i2cdev_;
    gain gain_;
    integral integral_;

    void power_on() const;
    void power_off() const;
    void set_reg(std::uint8_t address, std::uint8_t data) const;
    [[nodiscard]] std::vector<std::uint8_t> get_reg(
        std::uint8_t address, std::uintmax_t length) const;
};

}  // namespace driver::device::tsl2561

#endif
