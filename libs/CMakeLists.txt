find_package(Threads REQUIRED)

find_library(RT_LIBRARY rt)

add_library(pigpio)
target_sources(pigpio PRIVATE pigpio/pigpio.c pigpio/command.c
                              pigpio/custom.cext)
target_include_directories(pigpio PUBLIC pigpio)
target_link_libraries(pigpio PUBLIC Threads::Threads ${RT_LIBRARY})
target_compile_options(pigpio PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                      $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(pigpio PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(pigpio PROPERTIES C_EXTENSIONS ON C_STANDARD 11)

add_library(fmt)
target_sources(fmt PRIVATE fmt/src/format.cc)
target_include_directories(fmt PUBLIC fmt/include)
target_compile_options(fmt PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                   $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_features(fmt PRIVATE cxx_std_17)
target_compile_definitions(fmt PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(fmt PROPERTIES CXX_EXTENSIONS OFF)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(
        fmt
        PRIVATE FMT_EXPORT
        INTERFACE FMT_SHARED)
endif()

add_library(spdlog)
target_sources(
    spdlog
    PRIVATE spdlog/src/spdlog.cpp spdlog/src/stdout_sinks.cpp
            spdlog/src/color_sinks.cpp spdlog/src/file_sinks.cpp
            spdlog/src/async.cpp spdlog/src/cfg.cpp)
target_include_directories(spdlog PUBLIC spdlog/include)
target_link_libraries(spdlog PUBLIC Threads::Threads fmt)
target_compile_definitions(spdlog PUBLIC SPDLOG_COMPILED_LIB
                                         SPDLOG_FMT_EXTERNAL)
target_compile_options(spdlog PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                      $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_features(spdlog PRIVATE cxx_std_17)
target_compile_definitions(spdlog PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(spdlog PROPERTIES CXX_EXTENSIONS OFF)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(spdlog PUBLIC SPDLOG_BUILD_SHARED)
endif()

add_library(json INTERFACE)
target_include_directories(json INTERFACE libs/json/include)

add_library(httplib INTERFACE)
target_include_directories(httplib INTERFACE libs/cpp-httplib)

add_library(cxxopts INTERFACE)
target_include_directories(cxxopts INTERFACE libs/cxxopts/include)

add_library(tomlplusplus INTERFACE)
target_include_directories(tomlplusplus INTERFACE libs/tomlplusplus/include)
