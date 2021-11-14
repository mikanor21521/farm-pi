#include "device/lm35dz.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

std::shared_ptr<Lm35dz> createLm35dz(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::function<double()>& getVoltage) {
    static int count = 0;
    std::string logger_name = "lm35dz";
    count++;
    return std::make_shared<Lm35dz>(logger, getVoltage);
}

Lm35dz::Lm35dz(std::shared_ptr<spdlog::logger> logger,
               std::function<double()> getVoltage) noexcept
    : logger_(std::move(logger)), getVoltage_(std::move(getVoltage)) {}

double Lm35dz::getTemperature() const {
    return getVoltage_() * 100.0;
}
