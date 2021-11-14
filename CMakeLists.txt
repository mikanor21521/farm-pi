cmake_minimum_required(VERSION 3.15)
enable_language(C CXX)
project(albireo-pi)

add_subdirectory(libs)

add_executable(main)
target_sources(
    main
    PRIVATE src/main.cpp
            src/driver/driver.cpp
            src/driver/gpio.cpp
            src/driver/pwm.cpp
            src/driver/spi.cpp
            src/driver/i2c.cpp
            src/device/bme280.cpp
            src/device/mcp3002.cpp
            src/device/tsl2561.cpp)
target_include_directories(main PRIVATE include)
target_link_libraries(
    main
    PRIVATE pigpio
            spdlog
            json
            httplib
            cxxopts
            tomlplusplus
            paho-mqtt-cpp
            taskflow)
target_compile_options(main PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                                    $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_features(main PRIVATE cxx_std_17)
target_compile_definitions(main PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(main PROPERTIES CXX_EXTENSIONS OFF)
