#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <cstdint>
#include <exception>
#include <string>

namespace driver {

extern const std::string logger_name;
extern const std::string logfile_name;

void initialize(const std::string& file_name = logfile_name);
void finalize();

class driver_ex : public std::exception {
public:
    explicit driver_ex(std::string msg);
    driver_ex(const std::string& msg, int16_t error_num);
    [[nodiscard]] const char* what() const noexcept override;

private:
    std::string msg_;
};

}  // namespace driver

#endif
