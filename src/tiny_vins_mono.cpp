#include <iostream>
#include <memory>

#include "vio_system.h"
#include "utility/config.h"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_file>" << std::endl;
        std::cout << "Example: " << argv[0] << " ./config/config.yaml" << std::endl;
        return 1;
    }

    // Create configuration instance
    auto config = std::make_shared<utility::Config>();
    
    std::string config_file = argv[1];
    if (!config->loadFromYaml(config_file)) {
        std::cout << "Failed to load config from " << config_file << std::endl;
        return 1;
    }
    config->print();

    // Create VIO system with dependency injection
    VIOSystem vio_system(config);
    
    if (!vio_system.initialize()) {
        std::cout << "Failed to initialize VIO system" << std::endl;
        return 1;
    }

    // Process the sequence
    vio_system.processSequence();

    return 0;
}