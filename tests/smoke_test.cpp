#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Registry.hpp>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: espsdr_smoke_test MODULE\n";
        return 2;
    }
    const std::string module = argv[1];
    const std::string error = SoapySDR::loadModule(module);
    if (!error.empty()) {
        std::cerr << "module load failed: " << error << '\n';
        return 1;
    }
    const auto factories = SoapySDR::Registry::listMakeFunctions();
    if (factories.find("espsdr") == factories.end()) {
        std::cerr << "module did not register the espsdr factory\n";
        return 1;
    }
    const std::string unloadError = SoapySDR::unloadModule(module);
    if (!unloadError.empty()) {
        std::cerr << "module unload failed: " << unloadError << '\n';
        return 1;
    }
    std::cout << "SoapyESPSDR module load/registry/unload: OK\n";
    return 0;
}
