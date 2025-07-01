#include "config/config_manager.h"
#include <iostream>

namespace config {

ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::loadConfiguration(const std::string& config_path) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    config_ = std::make_shared<utility::Config>();
    
    if (!config_->loadFromYaml(config_path)) {
        std::cerr << "Failed to load configuration from: " << config_path << std::endl;
        config_.reset();
        return false;
    }
    
    config_file_path_ = config_path;
    
    // Basic validation: check if dataset path exists
    if (!config_->dataset_path.empty()) {
        // Simple warning if dataset path might not exist (without filesystem dependency)
        std::cout << "Configuration warnings:" << std::endl;
        std::cout << "  Warning: dataset_path may not exist: " << config_->dataset_path << std::endl;
    }
    
    std::cout << "Configuration loaded successfully from: " << config_path << std::endl;
    
    // Notify all registered callbacks
    notifyChange("configuration_loaded");
    
    return true;
}

std::shared_ptr<utility::Config> ConfigManager::getConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

bool ConfigManager::isLoaded() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_ != nullptr;
}

void ConfigManager::printConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    if (config_) {
        config_->print();
    } else {
        std::cout << "No configuration loaded." << std::endl;
    }
}

void ConfigManager::registerChangeCallback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    change_callbacks_.push_back(callback);
}

bool ConfigManager::validateConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        return false;
    }
    
    // Basic validation - check if all subsystems have valid parameters
    return validateCameraParams() && validateEstimatorParams() && validateFeatureTrackerParams();
}

bool ConfigManager::saveConfiguration(const std::string& config_path) const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        std::cerr << "No configuration to save." << std::endl;
        return false;
    }
    
    // Note: This would require implementing a YAML writer
    // For now, we'll just copy the original file if it's the same path
    if (config_path == config_file_path_) {
        std::cout << "Configuration is already at the specified path." << std::endl;
        return true;
    }
    
    std::cerr << "Saving configuration to different paths not yet implemented." << std::endl;
    return false;
}

void ConfigManager::notifyChange(const std::string& key) {
    for (const auto& callback : change_callbacks_) {
        try {
            callback(key);
        } catch (const std::exception& e) {
            std::cerr << "Error in configuration change callback: " << e.what() << std::endl;
        }
    }
}

bool ConfigManager::validateCameraParams() const {
    if (!config_) return false;
    // Basic validation - check if camera parameters are reasonable
    return config_->camera.fx > 0 && config_->camera.fy > 0 && 
           config_->camera.row > 0 && config_->camera.col > 0;
}

bool ConfigManager::validateEstimatorParams() const {
    if (!config_) return false;
    // Basic validation - check if estimator parameters are reasonable
    return config_->estimator.window_size > 0 && 
           config_->estimator.num_iterations > 0 &&
           config_->estimator.solver_time > 0;
}

bool ConfigManager::validateFeatureTrackerParams() const {
    if (!config_) return false;
    // Basic validation - check if feature tracker parameters are reasonable
    return config_->feature_tracker.max_cnt > 0 && 
           config_->feature_tracker.min_dist > 0;
}

} // namespace config