add_subdirectory(driver)

add_executable(main)
target_sources(main PRIVATE main.cpp)
target_link_libraries(main PRIVATE driver device::mcp3002 device::bme280 spdlog)
