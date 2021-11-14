#ifndef DEVICE_TSL2561_HPP
#define DEVICE_TSL2561_HPP

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver.hpp"

class Tsl2561 {
public:
    enum class Gain {
        x1,
        x16,
    };
    enum class Integral {
        ms13,
        ms101,
        ms402,
        custom,  // not supported
    };
    enum class Package {
        FN,
        CS,
    };
    Tsl2561(std::shared_ptr<spdlog::logger> logger,
            std::shared_ptr<driver::I2c> i2c, Gain gain, Integral integral,
            Package package);
    void initialize();
    void finalize();
    void measure();
    [[nodiscard]] std::uint16_t getChannel0Count();
    [[nodiscard]] std::uint16_t getChannel1Count();
    [[nodiscard]] double getIlluminance();

private:
    void powerOn() const;
    void powerOff() const;
    void setSettings() const;
    [[nodiscard]] bool isChipIdValid() const;
    void setReg(std::uint8_t address, std::uint8_t data) const;
    [[nodiscard]] std::vector<std::uint8_t> getReg(std::uint8_t address,
                                                   std::uintmax_t length) const;
    std::shared_mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<driver::I2c> i2c_;
    Gain gain_;
    Integral integral_;
    Package package_;
    std::uint16_t channel0_count_;
    std::uint16_t channel1_count_;
};

std::shared_ptr<Tsl2561> createTsl2561(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::I2c>& i2c, Tsl2561::Gain gain,
    Tsl2561::Integral integral, Tsl2561::Package package);

#endif
