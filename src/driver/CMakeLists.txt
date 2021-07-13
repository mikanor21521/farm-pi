add_library(driver)
target_sources(driver PRIVATE driver.cpp gpio.cpp i2c.cpp spi.cpp)
target_include_directories(driver PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(driver PUBLIC pigpio spdlog)
