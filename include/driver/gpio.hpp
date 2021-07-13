#ifndef GPIO_HPP
#define GPIO_HPP

#include <cstdint>
#include <memory>

#include "spdlog/spdlog.h"

namespace driver::gpio {

enum class mode : std::uint8_t {
    input = 0x00,
    output = 0x01,
};

enum class level : std::uint8_t {
    low = 0x00,
    high = 0x01,
};

class gpio final {
public:
    gpio(std::uint8_t num, mode gpio_mode);
    virtual ~gpio() noexcept = default;
    void initialize() const;
    void finalize() const;
    void set_level(level gpio_level) const;
    [[nodiscard]] level get_level() const;

private:
    std::shared_ptr<spdlog::logger> logger;
    std::uint8_t num_;
    mode mode_;
};

}  // namespace driver::gpio

#endif
