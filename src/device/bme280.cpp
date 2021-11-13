#include "device/bme280.hpp"

#include <cassert>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include "driver.hpp"

namespace {

constexpr std::uint8_t register_adc_data = 0xF7;
constexpr std::uint8_t register_config = 0xF5;
constexpr std::uint8_t register_ctrl_meas = 0xF4;
constexpr std::uint8_t register_status = 0xF3;
constexpr std::uint8_t register_ctrl_hum = 0xF2;
constexpr std::uint8_t register_reset = 0xE0;
constexpr std::uint8_t register_id = 0xD0;

constexpr std::uint8_t register_t1 = 0x88;
constexpr std::uint8_t register_t2 = 0x8A;
constexpr std::uint8_t register_t3 = 0x8C;

constexpr std::uint8_t register_p1 = 0x8E;
constexpr std::uint8_t register_p2 = 0x90;
constexpr std::uint8_t register_p3 = 0x92;
constexpr std::uint8_t register_p4 = 0x94;
constexpr std::uint8_t register_p5 = 0x96;
constexpr std::uint8_t register_p6 = 0x98;
constexpr std::uint8_t register_p7 = 0x9A;
constexpr std::uint8_t register_p8 = 0x9C;
constexpr std::uint8_t register_p9 = 0x9E;

constexpr std::uint8_t register_h1 = 0xA1;
constexpr std::uint8_t register_h2 = 0xE1;
constexpr std::uint8_t register_h3 = 0xE3;
constexpr std::uint8_t register_h4 = 0xE4;
constexpr std::uint8_t register_h5 = 0xE5;
constexpr std::uint8_t register_h6 = 0xE7;

constexpr std::uint8_t sampling_temperature_skip = 0x00;
constexpr std::uint8_t sampling_temperature_x1 = 0x20;
constexpr std::uint8_t sampling_temperature_x2 = 0x40;
constexpr std::uint8_t sampling_temperature_x4 = 0x60;
constexpr std::uint8_t sampling_temperature_x8 = 0x80;
constexpr std::uint8_t sampling_temperature_x16 = 0xA0;

constexpr std::uint8_t sampling_pressure_skip = 0x00;
constexpr std::uint8_t sampling_pressure_x1 = 0x04;
constexpr std::uint8_t sampling_pressure_x2 = 0x08;
constexpr std::uint8_t sampling_pressure_x4 = 0x0C;
constexpr std::uint8_t sampling_pressure_x8 = 0x10;
constexpr std::uint8_t sampling_pressure_x16 = 0x14;

constexpr std::uint8_t sampling_humidity_skip = 0x00;
constexpr std::uint8_t sampling_humidity_x1 = 0x01;
constexpr std::uint8_t sampling_humidity_x2 = 0x02;
constexpr std::uint8_t sampling_humidity_x4 = 0x03;
constexpr std::uint8_t sampling_humidity_x8 = 0x04;
constexpr std::uint8_t sampling_humidity_x16 = 0x05;

constexpr std::uint8_t mode_sleep = 0x00;
constexpr std::uint8_t mode_forced = 0x01;
constexpr std::uint8_t mode_normal = 0x03;

constexpr std::uint8_t standby_duration_ms0_5 = 0x00;
constexpr std::uint8_t standby_duration_ms62_5 = 0x20;
constexpr std::uint8_t standby_duration_ms125 = 0x40;
constexpr std::uint8_t standby_duration_ms250 = 0x60;
constexpr std::uint8_t standby_duration_ms500 = 0x80;
constexpr std::uint8_t standby_duration_ms1000 = 0xA0;
constexpr std::uint8_t standby_duration_ms10 = 0xC0;
constexpr std::uint8_t standby_duration_ms20 = 0xE0;

constexpr std::uint8_t filter_off = 0x00;
constexpr std::uint8_t filter_x2 = 0x04;
constexpr std::uint8_t filter_x4 = 0x08;
constexpr std::uint8_t filter_x8 = 0x0C;
constexpr std::uint8_t filter_x16 = 0x10;

constexpr std::uint8_t chip_id = 0x60;

constexpr double limit_temperature_max = 85.0;
constexpr double limit_temperature_min = -40.0;
constexpr double limit_pressure_max = 1100.0;
constexpr double limit_pressure_min = 300.0;
constexpr double limit_humidity_max = 100.0;
constexpr double limit_humidity_min = 0.0;

}  // namespace

std::shared_ptr<Bme280> createBme280(
    const std::shared_ptr<spdlog::logger>& logger,
    const std::shared_ptr<driver::Spi>& spi, Bme280::Sampling temp_sampling,
    Bme280::Sampling pres_sampling, Bme280::Sampling hum_sampling,
    Bme280::Mode mode, Bme280::StandbyDuration standby, Bme280::Filter filter) {
    static int count = 0;
    std::string logger_name = "bme280";
    return std::make_shared<Bme280>(
        logger->clone(fmt::format("{}-{}", logger_name, count)), spi,
        temp_sampling, pres_sampling, hum_sampling, mode, standby, filter);
}

Bme280::Bme280(std::shared_ptr<spdlog::logger> logger,
               std::shared_ptr<driver::Spi> spi, Sampling temp_sampling,
               Sampling pres_sampling, Sampling hum_sampling, Mode mode,
               StandbyDuration standby, Filter filter)
    : logger_(std::move(logger)),
      spi_(std::move(spi)),
      temp_sampling_(temp_sampling),
      pres_sampling_(pres_sampling),
      hum_sampling_(hum_sampling),
      mode_(mode),
      standby_(standby),
      filter_(filter),
      temperature_count_(0),
      pressure_count_(0),
      humidity_count_(0) {
    assert(temp_sampling == Sampling::skip || temp_sampling == Sampling::x1 ||
           temp_sampling == Sampling::x2 || temp_sampling == Sampling::x4 ||
           temp_sampling == Sampling::x8 || temp_sampling == Sampling::x16);
    assert(pres_sampling == Sampling::skip || pres_sampling == Sampling::x1 ||
           pres_sampling == Sampling::x2 || pres_sampling == Sampling::x4 ||
           pres_sampling == Sampling::x8 || pres_sampling == Sampling::x16);
    assert(hum_sampling == Sampling::skip || hum_sampling == Sampling::x1 ||
           hum_sampling == Sampling::x2 || hum_sampling == Sampling::x4 ||
           hum_sampling == Sampling::x8 || hum_sampling == Sampling::x16);
    assert(mode == Mode::sleep || mode == Mode::forced || mode == Mode::normal);
    assert(standby == StandbyDuration::ms0_5 ||
           standby == StandbyDuration::ms62_5 ||
           standby == StandbyDuration::ms125 ||
           standby == StandbyDuration::ms250 ||
           standby == StandbyDuration::ms500 ||
           standby == StandbyDuration::ms1000 ||
           standby == StandbyDuration::ms10 ||
           standby == StandbyDuration::ms20);
    assert(filter == Filter::off || filter == Filter::x2 ||
           filter == Filter::x4 || filter == Filter::x8 ||
           filter == Filter::x16);
}

void Bme280::initialize() {
    logger_->info("bme280 initialization start.");
    spi_->initialize();
    reset();
    if (!isChipIdValid()) {
        logger_->error("invalid chip id");
        driver::throwDriverException("invalid chip id");
    }
    readCalibData();
    setSettings();
    logger_->info("bme280 initialization done.");
}

void Bme280::finalize() {
    logger_->info("bme280 finalization start.");
    reset();
    spi_->finalize();
    logger_->info("bme280 finalization done.");
}

void Bme280::measure() {
    std::lock_guard<std::shared_mutex> lock(mtx_);
    std::uint32_t data_msb = 0;
    std::uint32_t data_lsb = 0;
    std::uint32_t data_xlsb = 0;
    logger_->info("bme280 measurement start.");
    if (mode_ == Mode::forced) {
        setMode(Mode::forced);
    }
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (isStarusMeasuring());
    auto adc_data = getReg(register_adc_data, 8);
    data_msb = adc_data[0] << 12;
    data_lsb = adc_data[1] << 4;
    data_xlsb = (adc_data[2] & 0xF0) >> 4;
    pressure_count_ = data_msb | data_lsb | data_xlsb;
    data_msb = adc_data[3] << 12;
    data_lsb = adc_data[4] << 4;
    data_xlsb = (adc_data[5] & 0xF0) >> 4;
    temperature_count_ = data_msb | data_lsb | data_xlsb;
    data_msb = adc_data[6] << 8;
    data_lsb = adc_data[7];
    humidity_count_ = data_msb | data_lsb;
    logger_->info("bme280 measurement done.");
}

std::uint32_t Bme280::getTemperatureCount() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return temperature_count_;
}

std::uint32_t Bme280::getPressureCount() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return pressure_count_;
}

std::uint32_t Bme280::getHumidityCount() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return humidity_count_;
}

double Bme280::getTemperature() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    double temperature = 0.0;
    temperature = readtfine() / 5120.0;
    if (temperature < limit_temperature_min) {
        temperature = limit_temperature_min;
    } else if (temperature > limit_temperature_max) {
        temperature = limit_temperature_max;
    }
    return temperature;
}

double Bme280::getPressure() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    double var1 = 0.0;
    double var2 = 0.0;
    double var3 = 0.0;
    double pressure = 0.0;
    var1 = readtfine() / 2.0 - 64000.0;
    var2 = var1 * var1 * calib_data_.dig_p6 / 32768.0;
    var2 = var2 + var1 * calib_data_.dig_p5 * 2.0;
    var2 = var2 / 4.0 + calib_data_.dig_p4 * 65536.0;
    var3 = calib_data_.dig_p3 * var1 * var1 / 524288.0;
    var1 = (var3 + calib_data_.dig_p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * calib_data_.dig_p1;
    if (var1 > 0.0) {
        pressure = 1048576.0 - pressure_count_;
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
        var1 = calib_data_.dig_p9 * pressure * pressure / 2147483648.0;
        var2 = pressure * calib_data_.dig_p8 / 32768.0;
        pressure = pressure + (var1 + var2 + calib_data_.dig_p7) / 16.0;
        if (pressure < limit_pressure_min) {
            pressure = limit_pressure_min;
        } else if (pressure > limit_pressure_max) {
            pressure = limit_pressure_max;
        }
    } else {
        pressure = limit_pressure_min;
    }
    return pressure;
}

double Bme280::getHumidity() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    double var1 = 0.0;
    double var2 = 0.0;
    double var3 = 0.0;
    double var4 = 0.0;
    double var5 = 0.0;
    double var6 = 0.0;
    double humidity = 0.0;
    var1 = readtfine() - 76800.0;
    var2 = calib_data_.dig_h4 * 64.0 + calib_data_.dig_h5 / 16384.0 * var1;
    var3 = humidity_count_ - var2;
    var4 = calib_data_.dig_h2 / 65536.0;
    var5 = calib_data_.dig_h3 / 67108864.0 * var1 + 1.0;
    var6 = calib_data_.dig_h6 / 67108864.0 * var1 * var5 + 1.0;
    var6 = var3 * var4 * var5 * var6;
    humidity = var6 * (1.0 - calib_data_.dig_h1 * var6 / 524288.0);
    if (humidity < limit_humidity_min) {
        humidity = limit_humidity_min;
    } else if (humidity > limit_humidity_max) {
        humidity = limit_humidity_max;
    }
    return humidity;
}

void Bme280::reset() const {
    setReg(register_reset, 0xB6);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (isStarusUpdate());
}

void Bme280::readCalibData() {
    auto read8 = [this](std::uint8_t address) -> std::uint8_t {
        auto data = getReg(address, 1);
        return data[0];
    };
    auto reads8 = [this](std::uint8_t address) -> std::int8_t {
        auto data = getReg(address, 1);
        return static_cast<std::int8_t>(data[0]);
    };
    auto read16_le = [this](std::uint8_t address) -> std::uint16_t {
        auto data = getReg(address, 2);
        return data[1] << 8 | data[0];
    };
    auto reads16_le = [this](std::uint8_t address) -> std::int16_t {
        auto data = getReg(address, 2);
        return static_cast<std::int16_t>(data[1] << 8 | data[0]);
    };
    calib_data_.dig_t1 = read16_le(register_t1);
    calib_data_.dig_t2 = reads16_le(register_t2);
    calib_data_.dig_t3 = reads16_le(register_t3);
    calib_data_.dig_p1 = read16_le(register_p1);
    calib_data_.dig_p2 = reads16_le(register_p2);
    calib_data_.dig_p3 = reads16_le(register_p3);
    calib_data_.dig_p4 = reads16_le(register_p4);
    calib_data_.dig_p5 = reads16_le(register_p5);
    calib_data_.dig_p6 = reads16_le(register_p6);
    calib_data_.dig_p7 = reads16_le(register_p7);
    calib_data_.dig_p8 = reads16_le(register_p8);
    calib_data_.dig_p9 = reads16_le(register_p9);
    calib_data_.dig_h1 = read8(register_h1);
    calib_data_.dig_h2 = reads16_le(register_h2);
    calib_data_.dig_h3 = read8(register_h3);
    calib_data_.dig_h4 =
        (reads8(register_h4) << 4) | (read8(register_h4 + 1) & 0x0F);
    calib_data_.dig_h5 =
        (reads8(register_h5 + 1) << 4) | (read8(register_h5) >> 4);
    calib_data_.dig_h6 = reads8(register_h6);
}

void Bme280::setMode(Mode mode) const {
    std::uint8_t reg_data = 0x00;
    auto data = getReg(register_ctrl_meas, 1);
    switch (mode) {
        case Mode::sleep:
            reg_data = (data[0] & ~0x03) | mode_sleep;
            break;
        case Mode::forced:
            reg_data = (data[0] & ~0x03) | mode_forced;
            break;
        case Mode::normal:
            reg_data = (data[0] & ~0x03) | mode_normal;
            break;
    }
    setReg(register_ctrl_meas, reg_data);
}

void Bme280::setSettings() const {
    std::uint8_t reg_config = 0x00;
    std::uint8_t reg_ctrl_meas = 0x00;
    std::uint8_t reg_ctrl_hum = 0x00;
    switch (temp_sampling_) {
        case Sampling::skip:
            reg_ctrl_meas |= sampling_temperature_skip;
            break;
        case Sampling::x1:
            reg_ctrl_meas |= sampling_temperature_x1;
            break;
        case Sampling::x2:
            reg_ctrl_meas |= sampling_temperature_x2;
            break;
        case Sampling::x4:
            reg_ctrl_meas |= sampling_temperature_x4;
            break;
        case Sampling::x8:
            reg_ctrl_meas |= sampling_temperature_x8;
            break;
        case Sampling::x16:
            reg_ctrl_meas |= sampling_temperature_x16;
            break;
    }
    switch (pres_sampling_) {
        case Sampling::skip:
            reg_ctrl_meas |= sampling_pressure_skip;
            break;
        case Sampling::x1:
            reg_ctrl_meas |= sampling_pressure_x1;
            break;
        case Sampling::x2:
            reg_ctrl_meas |= sampling_pressure_x2;
            break;
        case Sampling::x4:
            reg_ctrl_meas |= sampling_pressure_x4;
            break;
        case Sampling::x8:
            reg_ctrl_meas |= sampling_pressure_x8;
            break;
        case Sampling::x16:
            reg_ctrl_meas |= sampling_pressure_x16;
            break;
    }
    switch (hum_sampling_) {
        case Sampling::skip:
            reg_ctrl_hum |= sampling_humidity_skip;
            break;
        case Sampling::x1:
            reg_ctrl_hum |= sampling_humidity_x1;
            break;
        case Sampling::x2:
            reg_ctrl_hum |= sampling_humidity_x2;
            break;
        case Sampling::x4:
            reg_ctrl_hum |= sampling_humidity_x4;
            break;
        case Sampling::x8:
            reg_ctrl_hum |= sampling_humidity_x8;
            break;
        case Sampling::x16:
            reg_ctrl_hum |= sampling_humidity_x16;
            break;
    }
    switch (mode_) {
        case Mode::sleep:
            reg_ctrl_meas |= mode_sleep;
            break;
        case Mode::forced:
            reg_ctrl_meas |= mode_forced;
            break;
        case Mode::normal:
            reg_ctrl_meas |= mode_normal;
            break;
    }
    switch (standby_) {
        case StandbyDuration::ms0_5:
            reg_config |= standby_duration_ms0_5;
            break;
        case StandbyDuration::ms62_5:
            reg_config |= standby_duration_ms62_5;
            break;
        case StandbyDuration::ms125:
            reg_config |= standby_duration_ms125;
            break;
        case StandbyDuration::ms250:
            reg_config |= standby_duration_ms250;
            break;
        case StandbyDuration::ms500:
            reg_config |= standby_duration_ms500;
            break;
        case StandbyDuration::ms1000:
            reg_config |= standby_duration_ms1000;
            break;
        case StandbyDuration::ms10:
            reg_config |= standby_duration_ms10;
            break;
        case StandbyDuration::ms20:
            reg_config |= standby_duration_ms20;
            break;
    }
    switch (filter_) {
        case Filter::off:
            reg_config |= filter_off;
            break;
        case Filter::x2:
            reg_config |= filter_x2;
            break;
        case Filter::x4:
            reg_config |= filter_x4;
            break;
        case Filter::x8:
            reg_config |= filter_x8;
            break;
        case Filter::x16:
            reg_config |= filter_x16;
            break;
    }
    setReg(register_config, reg_config);
    setReg(register_ctrl_meas, reg_ctrl_meas);
    setReg(register_ctrl_hum, reg_ctrl_hum);
}

double Bme280::readtfine() const {
    double var1 = 0.0;
    double var2 = 0.0;
    var1 = temperature_count_ / 16384.0 - calib_data_.dig_t1 / 1024.0;
    var1 = var1 * calib_data_.dig_t2;
    var2 = temperature_count_ / 131072.0 - calib_data_.dig_t1 / 8192.0;
    var2 = var2 * var2 * calib_data_.dig_t3;
    return var1 + var2;
}

bool Bme280::isChipIdValid() const {
    auto data = getReg(register_id, 1);
    return data[0] == chip_id;
}

bool Bme280::isStarusMeasuring() const {
    auto data = getReg(register_status, 1);
    return (data[0] & 0x08) == 0x08;
}

bool Bme280::isStarusUpdate() const {
    auto data = getReg(register_status, 1);
    return (data[0] & 0x01) == 0x01;
}

void Bme280::setReg(std::uint8_t address, std::uint8_t data) const {
    std::vector<std::uint8_t> buf = {static_cast<std::uint8_t>(0x7F & address),
                                     data};
    spi_->write(buf);
}

std::vector<std::uint8_t> Bme280::getReg(std::uint8_t address,
                                         std::uintmax_t length) const {
    std::vector<std::uint8_t> buf(length + 1);
    buf[0] = address | 0x80;
    buf = spi_->transfer(buf);
    buf.erase(buf.begin());
    return buf;
}
