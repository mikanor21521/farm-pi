#include "driver.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "pigpio.h"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

std::shared_ptr<driver::Pwm> Driver::createPwm(std::uint8_t pin,
                                               std::uint16_t range,
                                               std::uint32_t frequency) {
    std::string logger_name = "pwm";
    auto logger = logger_->clone(fmt::format("{}-{}", logger_name, pwm_count_));
    pwm_count_++;
    auto pwm = std::make_shared<driver::Pwm>(logger, pin, range, frequency);
    return pwm;
}

namespace driver {

std::mutex Pwm::mtx_;

Pwm::Pwm(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t pin,
         std::uint16_t range, std::uint32_t frequency) noexcept
    : pin_(pin), range_(range), frequency_(frequency) {
    assert(pin <= 31);
    assert(25 <= range && range <= 40000);
    logger_ = logger;
    logger_->debug("pwm pin : {}", pin);
    logger_->debug("range : {}", range);
    logger_->debug("frequency : {}", frequency);
}

void Pwm::initialize() const {
    logger_->info("pwm initialization start.");
    int ret = 0;
    ret = gpioSetMode(pin_, PI_OUTPUT);
    if (ret < 0) {
        logger_->error("set pwm mode failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set pwm mode failed. status code: {}", ret);
    }
    ret = gpioSetPWMrange(pin_, range_);
    if (ret < 0) {
        logger_->error("set pwm range failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set pwm range failed. status code: {}", ret);
    }
    ret = gpioSetPWMfrequency(pin_, frequency_);
    if (ret < 0) {
        logger_->error("set pwm frequency failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set pwm range failed. status code: {}", ret);
    }
    setPWM(0);
    logger_->info("pwm initialization done.");
}

void Pwm::finalize() const {
    logger_->info("pwm finalization start.");
    setPWM(0);
    int ret = 0;
    ret = gpioSetMode(pin_, PI_INPUT);
    if (ret < 0) {
        logger_->error("set pwm mode failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set pwm mode failed. status code: {}", ret);
    }
    logger_->info("pwm finalization done.");
}

void Pwm::setPWM(std::uint16_t duty) const {
    std::lock_guard<std::mutex> lock(mtx_);
    logger_->info("set pwm start.");
    int ret = 0;
    ret = gpioPWM(pin_, duty);
    if (ret < 0) {
        logger_->error("set pwm failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set pwm failed. status code: {}", ret);
    }
    logger_->info("set pwm done.");
}

std::uint16_t Pwm::getPWM() const {
    std::lock_guard<std::mutex> lock(mtx_);
    logger_->info("get pwm start.");
    std::int32_t ret = 0;
    ret = gpioGetPWMdutycycle(pin_);
    if (ret < 0) {
        logger_->error("get pwm failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("get pwm failed. status code: {}", ret);
    }
    logger_->info("get pwm done.");
    return static_cast<std::uint16_t>(ret);
}

}  // namespace driver
