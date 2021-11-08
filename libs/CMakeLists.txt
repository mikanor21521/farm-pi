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

file(READ paho.mqtt.c/version.major PAHO_VERSION_MAJOR)
file(READ paho.mqtt.c/version.minor PAHO_VERSION_MINOR)
file(READ paho.mqtt.c/version.patch PAHO_VERSION_PATCH)
set(CLIENT_VERSION
    ${PAHO_VERSION_MAJOR}.${PAHO_VERSION_MINOR}.${PAHO_VERSION_PATCH})
string(TIMESTAMP BUILD_TIMESTAMP UTC)
add_library(paho-mqtt-c)
configure_file(paho.mqtt.c/src/VersionInfo.h.in
               ${CMAKE_BINARY_DIR}/paho.mqtt.c/VersionInfo.h @ONLY)
target_sources(
    paho-mqtt-c
    PRIVATE paho.mqtt.c/src/MQTTTime.c
            paho.mqtt.c/src/MQTTAsync.c
            paho.mqtt.c/src/MQTTAsyncUtils.c
            paho.mqtt.c/src/MQTTProtocolClient.c
            paho.mqtt.c/src/Clients.c
            paho.mqtt.c/src/utf-8.c
            paho.mqtt.c/src/MQTTPacket.c
            paho.mqtt.c/src/MQTTPacketOut.c
            paho.mqtt.c/src/Messages.c
            paho.mqtt.c/src/Tree.c
            paho.mqtt.c/src/Socket.c
            paho.mqtt.c/src/Log.c
            paho.mqtt.c/src/MQTTPersistence.c
            paho.mqtt.c/src/Thread.c
            paho.mqtt.c/src/MQTTProtocolOut.c
            paho.mqtt.c/src/MQTTPersistenceDefault.c
            paho.mqtt.c/src/SocketBuffer.c
            paho.mqtt.c/src/LinkedList.c
            paho.mqtt.c/src/MQTTProperties.c
            paho.mqtt.c/src/MQTTReasonCodes.c
            paho.mqtt.c/src/Base64.c
            paho.mqtt.c/src/SHA1.c
            paho.mqtt.c/src/WebSocket.c
            paho.mqtt.c/src/StackTrace.c
            paho.mqtt.c/src/Heap.c)
target_include_directories(
    paho-mqtt-c
    PUBLIC paho.mqtt.c/src
    PRIVATE ${CMAKE_BINARY_DIR}/paho.mqtt.c)
target_link_libraries(paho-mqtt-c PUBLIC Threads::Threads ${RT_LIBRARY})
target_compile_options(paho-mqtt-c PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                           $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(paho-mqtt-c PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(paho-mqtt-c PROPERTIES C_EXTENSIONS ON C_STANDARD 11)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(paho-mqtt-c PUBLIC PAHO_MQTT_EXPORTS)
else()
    target_compile_definitions(paho-mqtt-c PUBLIC PAHO_MQTT_STATIC)
endif()

add_library(paho-mqtt-cpp)
target_sources(
    paho-mqtt-cpp
    PRIVATE paho.mqtt.cpp/src/async_client.cpp
            paho.mqtt.cpp/src/client.cpp
            paho.mqtt.cpp/src/connect_options.cpp
            paho.mqtt.cpp/src/create_options.cpp
            paho.mqtt.cpp/src/disconnect_options.cpp
            paho.mqtt.cpp/src/iclient_persistence.cpp
            paho.mqtt.cpp/src/message.cpp
            paho.mqtt.cpp/src/properties.cpp
            paho.mqtt.cpp/src/response_options.cpp
            paho.mqtt.cpp/src/ssl_options.cpp
            paho.mqtt.cpp/src/string_collection.cpp
            paho.mqtt.cpp/src/subscribe_options.cpp
            paho.mqtt.cpp/src/token.cpp
            paho.mqtt.cpp/src/topic.cpp
            paho.mqtt.cpp/src/will_options.cpp)
target_include_directories(paho-mqtt-cpp PUBLIC paho.mqtt.cpp/src)
target_link_libraries(paho-mqtt-cpp PUBLIC paho-mqtt-c)
target_compile_options(paho-mqtt-cpp PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                             $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_features(paho-mqtt-cpp PRIVATE cxx_std_17)
target_compile_definitions(paho-mqtt-cpp PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
set_target_properties(paho-mqtt-cpp PROPERTIES CXX_EXTENSIONS OFF)

add_library(json INTERFACE)
target_include_directories(json INTERFACE json/include)

add_library(httplib INTERFACE)
target_include_directories(httplib INTERFACE cpp-httplib)

add_library(cxxopts INTERFACE)
target_include_directories(cxxopts INTERFACE cxxopts/include)

add_library(tomlplusplus INTERFACE)
target_include_directories(tomlplusplus INTERFACE tomlplusplus/include)
