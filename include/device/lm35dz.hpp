#ifndef DRIVER_LM35DZ_HPP
#define DRIVER_LM35DZ_HPP

#include <functional>
#include <memory>

#include "spdlog/spdlog.h"

class Lm35dz final {
public:
    Lm35dz(std::shared_ptr<spdlog::logger> logger,
           std::function<double()> getVoltage) noexcept;
    [[nodiscard]] double getTemperature() const;

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::function<double()> getVoltage_;
};

std::shared_ptr<Lm35dz> createLm35dz(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::function<double()>& getVoltage);

#endif
