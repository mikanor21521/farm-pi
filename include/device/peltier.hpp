#ifndef DRIVER_PELTIER_HPP
#define DRIVER_PELTIER_HPP

#include <cstdint>
#include <memory>

#include "spdlog/spdlog.h"

#include "driver.hpp"

class Peltier final {
public:
    Peltier(std::shared_ptr<spdlog::logger> logger,
            std::shared_ptr<driver::Pwm> pwm_plus,
            std::shared_ptr<driver::Pwm> pwm_minus, std::uint16_t duty_plus,
            std::uint16_t duty_minus) noexcept;
    void initialize();
    void finalize();
    void turnUp() const;
    void turnDowm() const;
    void turnOff() const;

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<driver::Pwm> pwm_plus_;
    std::shared_ptr<driver::Pwm> pwm_minus_;
    std::uint16_t duty_plus_;
    std::uint16_t duty_minus_;
};

std::shared_ptr<Peltier> createPeltier(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Pwm>& pwm_plus,
    const std::shared_ptr<driver::Pwm>& pwm_minus, std::uint16_t duty_plus,
    std::uint16_t duty_minus);

#endif
