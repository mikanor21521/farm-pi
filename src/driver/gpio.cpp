#include <cstdint>
#include <iostream>

#include "pigpio/pigpio.h"
#include "spdlog/spdlog.h"

#include "driver/driver.hpp"
#include "driver/gpio.hpp"

namespace driver::gpio {

gpio::gpio(std::uint8_t gpio_num, mode gpio_mode) {
    logger = spdlog::get(logger_name);
    if (!logger) {
        std::cerr << "logger get failed." << std::endl;
        throw driver_ex("logger get failed.");
    }
    logger->trace("gpio num: {}", gpio_num);
    if (gpio_num > 53) {
        logger->error("gpio initialization failed. gpio not 0-53.");
        throw driver_ex("gpio initialisation failed. gpio not 0-53.");
    }
    num_ = gpio_num;
    logger->trace("gpio mode: {}", static_cast<std::uint8_t>(gpio_mode));
    if (gpio_mode != mode::input && gpio_mode != mode::output) {
        logger->error("gpio initialization failed. mode not 0-1.");
        throw driver_ex("gpio initialisation failed. mode not 0-1.");
    }
    mode_ = gpio_mode;
}

void gpio::initialize() const {
    logger->debug("gpio initialization start.");
    std::int16_t ret = 0;
    switch (mode_) {
        case mode::input:
            ret = gpioSetMode(num_, PI_INPUT);
            break;
        case mode::output:
            ret = gpioSetMode(num_, PI_OUTPUT);
            break;
    }
    if (ret < 0) {
        logger->error("gpio initialization failed. status code: {}", ret);
        throw driver_ex("gpio initialisation failed.", ret);
    }
    logger->debug("gpio initialization done.");
}

void gpio::finalize() const {
    logger->debug("gpio finalization start.");
    std::int16_t ret = 0;
    if (mode_ == mode::output) {
        ret = gpioWrite(num_, PI_LOW);
        if (ret < 0) {
            logger->error("gpio finalization failed. status code: {}", ret);
            throw driver_ex("gpio finalization failed.", ret);
        }
        ret = gpioSetMode(num_, PI_INPUT);
        if (ret < 0) {
            logger->error("gpio finalization failed. status code: {}", ret);
            throw driver_ex("gpio finalization failed.", ret);
        }
    }
    logger->debug("gpio finalization done.");
}

void gpio::set_level(level gpio_level) const {
    logger->debug("gpio write start.");
    std::int16_t ret = 0;
    if (mode_ == mode::input) {
        logger->error("gpio write failed. mode not output.");
        throw driver_ex("gpio write failed. mode not output.");
    }
    switch (gpio_level) {
        case level::low:
            ret = gpioWrite(num_, PI_LOW);
            break;
        case level::high:
            ret = gpioWrite(num_, PI_HIGH);
            break;
        default:
            logger->error("gpio write failed. level not 0-1.");
            throw driver_ex("gpio write failed. level not 0-1.");
    }
    if (ret < 0) {
        logger->error("gpio write failed. status code: {}", ret);
        throw driver_ex("gpio write failed.", ret);
    }
    logger->debug("gpio write done. level: {}",
                  static_cast<std::uint8_t>(gpio_level));
}

level gpio::get_level() const {
    logger->debug("gpio read start.");
    std::int16_t ret = 0;
    ret = gpioRead(num_);
    if (ret < 0) {
        logger->error("gpio read failed. status code: {}", ret);
        throw driver_ex("gpio read failed.", ret);
    }
    level level_;
    switch (ret) {
        case PI_LOW:
            level_ = level::low;
            break;
        case PI_HIGH:
            level_ = level::high;
            break;
        default:
            logger->error("gpio read failed. level not 0-1. level: {}", ret);
            throw driver_ex("gpio read failed. level not 0-1.");
    }
    logger->debug("gpio read done. level: {}",
                  static_cast<std::uint8_t>(level_));
    return level_;
}

}  // namespace driver::gpio
