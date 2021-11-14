#include "device/peltier.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "driver.hpp"

std::shared_ptr<Peltier> createPeltier(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Pwm>& pwm_plus,
    const std::shared_ptr<driver::Pwm>& pwm_minus, std::uint16_t duty_plus,
    std::uint16_t duty_minus) {
    static int count = 0;
    std::string logger_name = "peltier";
    count++;
    return std::make_shared<Peltier>(
        logger->clone(fmt::format("{}-{}", logger_name, count)), pwm_plus,
        pwm_minus, duty_plus, duty_minus);
}

Peltier::Peltier(std::shared_ptr<spdlog::logger> logger,
                 std::shared_ptr<driver::Pwm> pwm_plus,
                 std::shared_ptr<driver::Pwm> pwm_minus,
                 std::uint16_t duty_plus, std::uint16_t duty_minus) noexcept
    : logger_(std::move(logger)),
      pwm_plus_(std::move(pwm_plus)),
      pwm_minus_(std::move(pwm_minus)),
      duty_plus_(duty_plus),
      duty_minus_(duty_minus) {}

void Peltier::initialize() {
    logger_->info("peltier element initialization start.");
    pwm_plus_->initialize();
    pwm_minus_->initialize();
    logger_->info("peltier element initialization done.");
}

void Peltier::finalize() {
    logger_->info("peltier element finalization start.");
    pwm_plus_->finalize();
    pwm_minus_->finalize();
    logger_->info("peltier element finalization done.");
}

void Peltier::turnUp() const {
    pwm_plus_->setPWM(duty_plus_);
    pwm_minus_->setPWM(0);
}

void Peltier::turnDowm() const {
    pwm_plus_->setPWM(0);
    pwm_minus_->setPWM(duty_minus_);
}

void Peltier::turnOff() const {
    pwm_plus_->setPWM(0);
    pwm_minus_->setPWM(0);
}
