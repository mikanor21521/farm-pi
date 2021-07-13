#ifndef I2C_HPP
#define I2C_HPP

#include <bitset>
#include <cstdint>
#include <memory>
#include <vector>

#include "spdlog/logger.h"
#include "spdlog/spdlog.h"

namespace driver::i2c {

class i2c final {
public:
    i2c(std::uint8_t bus_number, std::uint8_t address);
    virtual ~i2c() noexcept = default;
    void initialize();
    void finalize() const;
    void write(std::vector<std::bitset<8>> data);
    std::vector<std::bitset<8>> read(std::uintmax_t length);

private:
    std::shared_ptr<spdlog::logger> logger;
    std::uint8_t i2cdev_;
    std::uint8_t bus_;
    std::uint8_t addr_;
};

}  // namespace driver::i2c

#endif
