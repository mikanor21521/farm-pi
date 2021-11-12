#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

namespace driver {

class DriverException : public std::exception {
public:
    explicit DriverException(std::string msg) noexcept : msg_(std::move(msg)) {}
    [[nodiscard]] const char* what() const noexcept override {
        return msg_.c_str();
    }

private:
    std::string msg_;
};

template <typename... Args>
[[noreturn]] void throwDriverException(const std::string& msg, Args... args) {
    throw DriverException(fmt::format(msg, args...));
}

class Gpio final {
public:
    enum class Mode {
        input,
        output,
    };
    enum class Level {
        low,
        high,
    };
    Gpio(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t num,
         Mode mode) noexcept;
    void initialize() const;
    void finalize() const;
    void setLevel(Level level) const;
    [[nodiscard]] Level getLevel() const;

private:
    void setMode(Mode mode) const;
    static std::mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::uint8_t num_;
    Mode mode_;
};

class Pwm final {
public:
    Pwm(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t pin,
        std::uint16_t range, std::uint32_t frequency) noexcept;
    void initialize() const;
    void finalize() const;
    void setPWM(std::uint16_t duty) const;
    [[nodiscard]] std::uint16_t getPWM() const;

private:
    static std::mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::uint8_t pin_;
    std::uint16_t range_;
    std::uint32_t frequency_;
};

class Spi final {
public:
    Spi(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t cs_pin,
        std::uint32_t clock_speed, std::uint8_t cpol, std::uint8_t cpha,
        bool active_high) noexcept;
    void initialize();
    void finalize() const;
    void write(std::vector<std::uint8_t> data) const;
    [[nodiscard]] std::vector<std::uint8_t> read(std::uintmax_t length) const;
    [[nodiscard]] std::vector<std::uint8_t> transfer(
        std::vector<std::uint8_t> data) const;

private:
    static std::mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::uint8_t cs_pin_;
    std::uint32_t clock_speed_;
    std::uint8_t cpol_;
    std::uint8_t cpha_;
    bool active_high_;
    std::uint8_t spidev_;
};

class I2c final {
public:
    I2c(const std::shared_ptr<spdlog::logger>& logger, std::uint8_t bus_number,
        std::uint8_t address) noexcept;
    void initialize();
    void finalize() const;
    void write(std::vector<std::uint8_t> data) const;
    [[nodiscard]] std::vector<std::uint8_t> read(std::uintmax_t length) const;

private:
    static std::mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::uint8_t bus_number_;
    std::uint8_t address_;
    std::uint8_t i2cdev_;
};

}  // namespace driver

class Driver final {
public:
    explicit Driver(const std::shared_ptr<spdlog::logger>& logger);
    void initialize();
    void finalize();
    [[nodiscard]] std::shared_ptr<driver::Gpio> createGpio(
        std::uint8_t num, driver::Gpio::Mode mode);
    [[nodiscard]] std::shared_ptr<driver::Pwm> createPwm(
        std::uint8_t pin, std::uint16_t range, std::uint32_t frequency);
    [[nodiscard]] std::shared_ptr<driver::Spi> createSpi(
        std::uint8_t cs_pin, std::uint32_t clock_speed, std::uint8_t cpol,
        std::uint8_t cpha, bool active_high);
    [[nodiscard]] std::shared_ptr<driver::I2c> createI2c(
        std::uint8_t bus_number, std::uint8_t address);

private:
    std::shared_ptr<spdlog::logger> logger_;
    int gpio_count_ = 0;
    int pwm_count_ = 0;
    int spi_count_ = 0;
    int i2c_count_ = 0;
};

#endif
