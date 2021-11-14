#include "device/light.hpp"

#include <memory>
#include <string>
#include <utility>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "driver.hpp"

std::shared_ptr<Light> createLight(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Gpio>& gpio) {
    static int count = 0;
    std::string logger_name = "light";
    count++;
    return std::make_shared<Light>(
        logger->clone(fmt::format("{}-{}", logger_name, count)), gpio);
}

Light::Light(std::shared_ptr<spdlog::logger> logger,
             std::shared_ptr<driver::Gpio> gpio) noexcept
    : logger_(std::move(logger)), gpio_(std::move_if_noexcept(gpio)) {}

void Light::initialize() {
    logger_->info("light initialization start.");
    gpio_->initialize();
    logger_->info("light initialization done.");
}

void Light::finalize() {
    logger_->info("light finalization start.");
    gpio_->finalize();
    logger_->info("light finalization done.");
}

void Light::turnOn() const {
    gpio_->setLevel(driver::Gpio::Level::low);
}

void Light::turnOff() const {
    gpio_->setLevel(driver::Gpio::Level::high);
}
