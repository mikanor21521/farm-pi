#ifndef DEVICE_BME280_HPP
#define DEVICE_BME280_HPP

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver.hpp"

class Bme280 final {
public:
    enum class Sampling {
        skip,
        x1,
        x2,
        x4,
        x8,
        x16,
    };
    enum class Mode {
        sleep,
        forced,
        normal,
    };
    enum class StandbyDuration {
        ms0_5,
        ms62_5,
        ms125,
        ms250,
        ms500,
        ms1000,
        ms10,
        ms20,
    };
    enum class Filter {
        off,
        x2,
        x4,
        x8,
        x16,
    };
    struct CalibData {
        std::uint16_t dig_t1 = 0;
        std::int16_t dig_t2 = 0;
        std::int16_t dig_t3 = 0;
        std::uint16_t dig_p1 = 0;
        std::int16_t dig_p2 = 0;
        std::int16_t dig_p3 = 0;
        std::int16_t dig_p4 = 0;
        std::int16_t dig_p5 = 0;
        std::int16_t dig_p6 = 0;
        std::int16_t dig_p7 = 0;
        std::int16_t dig_p8 = 0;
        std::int16_t dig_p9 = 0;
        std::uint8_t dig_h1 = 0;
        std::int16_t dig_h2 = 0;
        std::uint8_t dig_h3 = 0;
        std::int16_t dig_h4 = 0;
        std::int16_t dig_h5 = 0;
        std::int8_t dig_h6 = 0;
    };
    Bme280(std::shared_ptr<spdlog::logger> logger,
           std::shared_ptr<driver::Spi> spi, Sampling temp_sampling,
           Sampling pres_sampling, Sampling hum_sampling, Mode mode,
           StandbyDuration standby, Filter filter);
    void initialize();
    void finalize();
    void measure();
    [[nodiscard]] std::uint32_t getTemperatureCount();
    [[nodiscard]] std::uint32_t getPressureCount();
    [[nodiscard]] std::uint32_t getHumidityCount();
    [[nodiscard]] double getTemperature();
    [[nodiscard]] double getPressure();
    [[nodiscard]] double getHumidity();

private:
    void reset() const;
    void readCalibData();
    void setMode(Mode mode) const;
    void setSettings() const;
    [[nodiscard]] double readtfine() const;
    [[nodiscard]] bool isChipIdValid() const;
    [[nodiscard]] bool isStarusMeasuring() const;
    [[nodiscard]] bool isStarusUpdate() const;
    void setReg(std::uint8_t address, std::uint8_t data) const;
    [[nodiscard]] std::vector<std::uint8_t> getReg(std::uint8_t address,
                                                   std::uintmax_t length) const;
    std::shared_mutex mtx_;
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<driver::Spi> spi_;
    Sampling temp_sampling_;
    Sampling pres_sampling_;
    Sampling hum_sampling_;
    Mode mode_;
    StandbyDuration standby_;
    Filter filter_;
    CalibData calib_data_;
    std::uint32_t temperature_count_;
    std::uint32_t pressure_count_;
    std::uint32_t humidity_count_;
};

std::shared_ptr<Bme280> createBme280(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Spi>& spi, Bme280::Sampling temp_sampling,
    Bme280::Sampling pres_sampling, Bme280::Sampling hum_sampling,
    Bme280::Mode mode, Bme280::StandbyDuration standby, Bme280::Filter filter);

#endif
