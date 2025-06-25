#include <iostream>

#include "utility/config.h"
#include "backend/estimator.h"

Estimator estimator;

int main(int argc, char* argv[]) {
     if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
        std::cout << "Example: " << argv[0] << " ./config/config.yaml" << std::endl;
        return 1;
    }

    std::string config_file = argv[1];
    if (!utility::g_config.loadFromYaml(config_file)) {
        std::cout << "Failed to load config from " << config_file << std::endl;
        return 1;
    }
    utility::g_config.print();


}