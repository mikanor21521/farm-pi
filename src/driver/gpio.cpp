#include "driver.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <string>

#include "pigpio.h"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

namespace alcor {

std::unique_ptr<driver::Gpio> Driver::createGpio(std::uint8_t num,
                                                 driver::Gpio::Mode mode) {
    std::string logger_name = "gpio";
    auto logger =
        logger_->clone(fmt::format("{}-{}", logger_name, gpio_count_));
    gpio_count_++;
    auto gpio = std::make_unique<driver::Gpio>(logger, num, mode);
    return gpio;
}

namespace driver {

Gpio::Gpio(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t num,
           Mode mode) noexcept
    : num_(num), mode_(mode) {
    assert(num < 53);
    assert(mode == Mode::input || mode == Mode::output);
    logger_ = logger;
    logger_->debug("gpio number: {}", num);
    if (mode == Mode::input) {
        logger_->debug("gpio mode: input");
    } else {
        logger_->debug("gpio mode: output");
    }
}

void Gpio::initialize() const {
    logger_->info("gpio initialization start.");
    setMode(mode_);
    logger_->info("gpio initialization done.");
}

void Gpio::finalize() const {
    logger_->info("gpio finalization start.");
    if (mode_ == Mode::output) {
        setLevel(Level::low);
        setMode(Mode::input);
    }
    logger_->info("gpio finalization done.");
}

void Gpio::setLevel(Level level) const {
    assert(mode_ == Mode::output);
    logger_->info("set gpio level start.");
    int ret = 0;
    switch (level) {
        case Level::low:
            ret = gpioWrite(num_, PI_LOW);
            logger_->info("set gpio level to low.");
            break;
        case Level::high:
            ret = gpioWrite(num_, PI_HIGH);
            logger_->info("set gpio level to high.");
            break;
    }
    if (ret < 0) {
        logger_->error("set gpio level failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set gpio level failed. status code: {}", ret);
    }
    logger_->info("set gpio level done.");
}

Gpio::Level Gpio::getLevel() const {
    logger_->info("get gpio level start.");
    int ret = 0;
    Level level = Level::low;
    ret = gpioRead(num_);
    if (ret < 0) {
        logger_->error("get gpio level failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("get gpio level failed. status code: {}", ret);
    }
    switch (ret) {
        case PI_LOW:
            level = Level::low;
            logger_->info("gpio level is low.");
            break;
        case PI_HIGH:
            level = Level::high;
            logger_->info("gpio level is high.");
            break;
        default:
            logger_->error("get gpio level failed.");
            logger_->debug("status : {}", ret);
            throwDriverException("get gpio level failed. status: {}", ret);
    }
    logger_->info("get gpio level done.");
    return level;
}

void Gpio::setMode(Mode mode) const {
    logger_->info("set gpio mode start.");
    int ret = 0;
    switch (mode) {
        case Mode::input:
            ret = gpioSetMode(num_, PI_INPUT);
            logger_->info("set gpio mode to input.");
            break;
        case Mode::output:
            ret = gpioSetMode(num_, PI_OUTPUT);
            logger_->info("set gpio mode to output.");
            break;
    }
    if (ret < 0) {
        logger_->error("set gpio mode failed.");
        logger_->debug("status code: {}", ret);
        throwDriverException("set gpio mode failed. status code: {}", ret);
    }
    logger_->info("set gpio mode done.");
}

}  // namespace driver
}  // namespace alcor
