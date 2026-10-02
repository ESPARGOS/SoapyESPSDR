cmake_minimum_required(VERSION 3.16)
project(SoapyESPSDR VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
find_package(SoapySDR CONFIG REQUIRED)
find_package(CURL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

SOAPY_SDR_MODULE_UTIL(
    TARGET ESPSDRSupport
    SOURCES src/SoapyESPSDR.cpp src/Transport.cpp
    LIBRARIES CURL::libcurl PkgConfig::JSONCPP PkgConfig::LIBUSB Threads::Threads
)

target_compile_features(ESPSDRSupport PRIVATE cxx_std_17)
target_compile_options(ESPSDRSupport PRIVATE -Wall -Wextra -Wpedantic)

add_executable(espsdr_loss_monitor apps/espsdr_loss_monitor.cpp)
target_compile_features(espsdr_loss_monitor PRIVATE cxx_std_17)
target_link_libraries(espsdr_loss_monitor PRIVATE SoapySDR)
install(TARGETS espsdr_loss_monitor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

include(CTest)
if(BUILD_TESTING)
    add_executable(espsdr_smoke_test tests/smoke_test.cpp)
    target_compile_features(espsdr_smoke_test PRIVATE cxx_std_17)
    target_link_libraries(espsdr_smoke_test PRIVATE SoapySDR)
    add_test(NAME module_load COMMAND espsdr_smoke_test $<TARGET_FILE:ESPSDRSupport>)
    add_executable(espsdr_protocol_test tests/protocol_test.cpp)
    target_compile_features(espsdr_protocol_test PRIVATE cxx_std_17)
    target_include_directories(espsdr_protocol_test PRIVATE src)
    add_test(NAME protocol COMMAND espsdr_protocol_test)
    add_executable(espsdr_dc_test tests/dc_test.cpp)
    target_include_directories(espsdr_dc_test PRIVATE src)
    add_test(NAME dc_removal COMMAND espsdr_dc_test)
endif()
