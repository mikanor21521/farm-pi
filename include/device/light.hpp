#ifndef DEVICE_LIGHT_HPP
#define DEVICE_LIGHT_HPP

#include <memory>

#include "spdlog/spdlog.h"

#include "driver.hpp"

class Light final {
public:
    explicit Light(std::shared_ptr<spdlog::logger> logger,
                   std::shared_ptr<driver::Gpio> gpio) noexcept;
    void initialize();
    void finalize();
    void turnOn() const;
    void turnOff() const;

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<driver::Gpio> gpio_;
};

std::shared_ptr<Light> createLight(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Gpio>& gpio);

#endif
