#ifndef BME280_HPP
#define BME280_HPP

#include <bitset>
#include <cstdint>
#include <memory>
#include <vector>

#include "spdlog/spdlog.h"

#include "driver/spi.hpp"

namespace driver::device::bme280 {

enum class oversampling_temperature : std::uint8_t {
    skip = 0x00,
    x1 = 0x20,
    x2 = 0x40,
    x4 = 0x60,
    x8 = 0x80,
    x16 = 0xA0,
};

enum class oversampling_pressure : std::uint8_t {
    skip = 0x00,
    x1 = 0x04,
    x2 = 0x08,
    x4 = 0x0C,
    x8 = 0x10,
    x16 = 0x14,
};

enum class oversampling_humidity : std::uint8_t {
    skip = 0x00,
    x1 = 0x01,
    x2 = 0x02,
    x4 = 0x03,
    x8 = 0x04,
    x16 = 0x05,
};

enum class mode : std::uint8_t {
    sleep = 0x00,
    forced = 0x01,
    normal = 0x03,
};

enum class standby_time : std::uint8_t {
    ms0_5 = 0x00,
    ms62_5 = 0x20,
    ms125 = 0x40,
    ms250 = 0x60,
    ms500 = 0x80,
    ms1000 = 0xA0,
    ms10 = 0xC0,
    ms20 = 0xE0,
};

enum class filter : std::uint8_t {
    off = 0x00,
    x2 = 0x04,
    x4 = 0x08,
    x8 = 0x0C,
    x16 = 0x10,
};

class bme280 final {
public:
    bme280(spi::spi* spi_device, mode mode, oversampling_temperature osr_temp,
           oversampling_pressure osr_pres, oversampling_humidity osr_hum,
           standby_time standby, filter filter);
    void initialize();
    void finalize();
    double read_temperature();
    double read_pressure();
    double read_humidity();

private:
    std::shared_ptr<spdlog::logger> logger;
    spi::spi* spidev_;
    mode mode_;
    oversampling_temperature osr_temp_;
    oversampling_pressure osr_pres_;
    oversampling_humidity osr_hum_;
    standby_time standby_;
    filter filter_;
    double t_fine_;

    void set_registor(std::uint8_t addr, std::uint8_t data);
    std::uint8_t get_registor(std::uint8_t addr);
    void reset();
    void set_settings();
};

}  // namespace driver::device::bme280

#endif
