cmake_minimum_required(VERSION 3.16)
project(SoapyESPSDR VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
find_package(SoapySDR CONFIG REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ZLIB REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

SOAPY_SDR_MODULE_UTIL(
    TARGET ESPSDRSupport
    SOURCES src/SoapyESPSDR.cpp
    LIBRARIES CURL::libcurl PkgConfig::JSONCPP ZLIB::ZLIB
)

target_compile_features(ESPSDRSupport PRIVATE cxx_std_17)
target_compile_options(ESPSDRSupport PRIVATE -Wall -Wextra -Wpedantic)

add_executable(espsdr_loss_monitor apps/espsdr_loss_monitor.cpp)
target_compile_features(espsdr_loss_monitor PRIVATE cxx_std_17)
target_link_libraries(espsdr_loss_monitor PRIVATE SoapySDR)
install(TARGETS espsdr_loss_monitor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

option(ESPSDR_BUILD_TESTS "Build the SoapyESPSDR live hardware tests" ON)
if(ESPSDR_BUILD_TESTS)
    add_executable(espsdr_live_rate_test tests/live_rate_test.cpp)
    target_compile_features(espsdr_live_rate_test PRIVATE cxx_std_17)
    target_link_libraries(espsdr_live_rate_test PRIVATE SoapySDR)
    add_executable(espsdr_live_control_test tests/live_control_test.cpp)
    target_compile_features(espsdr_live_control_test PRIVATE cxx_std_17)
    target_link_libraries(espsdr_live_control_test PRIVATE SoapySDR)
    add_executable(espsdr_live_reconfigure_test tests/live_reconfigure_test.cpp)
    target_compile_features(espsdr_live_reconfigure_test PRIVATE cxx_std_17)
    target_link_libraries(espsdr_live_reconfigure_test PRIVATE SoapySDR)
endif()
