list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/vendor/pigpio/cmake)

find_package(Threads REQUIRED)
find_package(RT REQUIRED)

add_library(pigpio)
target_sources(pigpio PRIVATE pigpio/pigpio.c pigpio/command.c
                              pigpio/custom.cext)
target_include_directories(pigpio PRIVATE ./pigpio/)
target_include_directories(pigpio INTERFACE ./)
target_link_libraries(pigpio PUBLIC RT::RT Threads::Threads)

add_library(fmt)
target_sources(fmt PRIVATE ./fmt/src/format.cc)
target_include_directories(fmt PUBLIC ./fmt/include/)
target_compile_definitions(fmt PUBLIC FMT_SHARED)

add_library(spdlog)
target_sources(
  spdlog
  PRIVATE ./spdlog/src/spdlog.cpp ./spdlog/src/stdout_sinks.cpp
          ./spdlog/src/color_sinks.cpp ./spdlog/src/file_sinks.cpp
          ./spdlog/src/async.cpp ./spdlog/src/cfg.cpp)
target_include_directories(spdlog PUBLIC ./spdlog/include)
if(BUILD_SHARED_LIBS)
  target_compile_definitions(spdlog PUBLIC SPDLOG_BUILD_SHARED)
endif()
target_compile_definitions(spdlog PUBLIC SPDLOG_COMPILED_LIB
                                         SPDLOG_FMT_EXTERNAL)
target_link_libraries(spdlog PUBLIC Threads::Threads fmt)
