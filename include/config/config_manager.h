#ifndef CONFIG__CONFIG_MANAGER_H
#define CONFIG__CONFIG_MANAGER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include <mutex>
#include "utility/config.h"

namespace config {

/**
 * @brief Configuration manager for the VIO system
 * 
 * This class provides a centralized way to manage configuration,
 * handle configuration updates, and provide type-safe access
 * to configuration parameters.
 */
class ConfigManager {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the ConfigManager instance
     */
    static ConfigManager& getInstance();
    
    /**
     * @brief Load configuration from file
     * @param config_path Path to the configuration file
     * @return true if successful
     */
    bool loadConfiguration(const std::string& config_path);
    
    /**
     * @brief Get configuration instance
     * @return Shared pointer to configuration
     */
    std::shared_ptr<utility::Config> getConfig() const;
    
    /**
     * @brief Get a specific parameter with type safety
     * @param key Parameter key
     * @param default_value Default value if key not found
     * @return Parameter value
     */
    template<typename T>
    T getParameter(const std::string& key, const T& default_value = T{}) const;
    
    /**
     * @brief Set a parameter value
     * @param key Parameter key
     * @param value Parameter value
     */
    template<typename T>
    void setParameter(const std::string& key, const T& value);
    
    /**
     * @brief Check if configuration is loaded
     * @return true if configuration is available
     */
    bool isLoaded() const;
    
    /**
     * @brief Print current configuration
     */
    void printConfiguration() const;
    
    /**
     * @brief Register a configuration change callback
     * @param callback Function to call when configuration changes
     */
    void registerChangeCallback(std::function<void(const std::string&)> callback);
    
    /**
     * @brief Validate the current configuration
     * @return true if configuration is valid
     */
    bool validateConfiguration() const;
    
    /**
     * @brief Save configuration to file
     * @param config_path Path to save the configuration
     * @return true if successful
     */
    bool saveConfiguration(const std::string& config_path) const;

private:
    ConfigManager() = default;
    ~ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    
    mutable std::mutex config_mutex_;
    std::shared_ptr<utility::Config> config_;
    std::string config_file_path_;
    std::vector<std::function<void(const std::string&)>> change_callbacks_;
    
    // Parameter access methods
    void notifyChange(const std::string& key);
    bool validateCameraParams() const;
    bool validateEstimatorParams() const;
    bool validateFeatureTrackerParams() const;
};

// Template implementations
template<typename T>
T ConfigManager::getParameter(const std::string& key, const T& default_value) const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        return default_value;
    }
    
    // Map common parameter keys to config fields
    if constexpr (std::is_same_v<T, int>) {
        if (key == "frame_skip") return static_cast<T>(config_->frame_skip);
        if (key == "start_frame") return static_cast<T>(config_->start_frame);
        if (key == "end_frame") return static_cast<T>(config_->end_frame);
        if (key == "estimator.window_size") return static_cast<T>(config_->estimator.window_size);
        if (key == "estimator.num_iterations") return static_cast<T>(config_->estimator.num_iterations);
        if (key == "feature_tracker.max_cnt") return static_cast<T>(config_->feature_tracker.max_cnt);
        if (key == "feature_tracker.min_dist") return static_cast<T>(config_->feature_tracker.min_dist);
        if (key == "feature_tracker.window_size") return static_cast<T>(config_->feature_tracker.window_size);
        if (key == "feature_tracker.show_track") return static_cast<T>(config_->feature_tracker.show_track);
        if (key == "feature_tracker.equalize") return static_cast<T>(config_->feature_tracker.equalize);
        if (key == "feature_tracker.fisheye") return static_cast<T>(config_->feature_tracker.fisheye);
    }
    else if constexpr (std::is_same_v<T, double>) {
        if (key == "camera.focal_length") return static_cast<T>(config_->camera.focal_length);
        if (key == "camera.fx") return static_cast<T>(config_->camera.fx);
        if (key == "camera.fy") return static_cast<T>(config_->camera.fy);
        if (key == "camera.cx") return static_cast<T>(config_->camera.cx);
        if (key == "camera.cy") return static_cast<T>(config_->camera.cy);
        if (key == "estimator.solver_time") return static_cast<T>(config_->estimator.solver_time);
        if (key == "estimator.min_parallax") return static_cast<T>(config_->estimator.min_parallax);
        if (key == "estimator.init_depth") return static_cast<T>(config_->estimator.init_depth);
        if (key == "estimator.acc_n") return static_cast<T>(config_->estimator.acc_n);
        if (key == "estimator.acc_w") return static_cast<T>(config_->estimator.acc_w);
        if (key == "estimator.gyr_n") return static_cast<T>(config_->estimator.gyr_n);
        if (key == "estimator.gyr_w") return static_cast<T>(config_->estimator.gyr_w);
        if (key == "feature_tracker.f_threshold") return static_cast<T>(config_->feature_tracker.f_threshold);
    }
    else if constexpr (std::is_same_v<T, std::string>) {
        if (key == "dataset_path") return static_cast<T>(config_->dataset_path);
        if (key == "config_filepath") return static_cast<T>(config_->config_filepath);
        if (key == "feature_tracker.fisheye_mask") return static_cast<T>(config_->feature_tracker.fisheye_mask);
    }
    
    return default_value;
}

template<typename T>
void ConfigManager::setParameter(const std::string& key, const T& value) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!config_) {
        return;
    }
    
    // Map parameter keys to config fields for setting
    if constexpr (std::is_same_v<T, int>) {
        if (key == "frame_skip") { config_->frame_skip = value; notifyChange(key); return; }
        if (key == "start_frame") { config_->start_frame = value; notifyChange(key); return; }
        if (key == "end_frame") { config_->end_frame = value; notifyChange(key); return; }
        if (key == "estimator.window_size") { config_->estimator.window_size = value; notifyChange(key); return; }
        if (key == "estimator.num_iterations") { config_->estimator.num_iterations = value; notifyChange(key); return; }
        if (key == "feature_tracker.max_cnt") { config_->feature_tracker.max_cnt = value; notifyChange(key); return; }
        if (key == "feature_tracker.min_dist") { config_->feature_tracker.min_dist = value; notifyChange(key); return; }
        if (key == "feature_tracker.window_size") { config_->feature_tracker.window_size = value; notifyChange(key); return; }
        if (key == "feature_tracker.show_track") { config_->feature_tracker.show_track = value; notifyChange(key); return; }
        if (key == "feature_tracker.equalize") { config_->feature_tracker.equalize = value; notifyChange(key); return; }
        if (key == "feature_tracker.fisheye") { config_->feature_tracker.fisheye = value; notifyChange(key); return; }
    }
    else if constexpr (std::is_same_v<T, double>) {
        if (key == "camera.focal_length") { config_->camera.focal_length = value; notifyChange(key); return; }
        if (key == "camera.fx") { config_->camera.fx = value; notifyChange(key); return; }
        if (key == "camera.fy") { config_->camera.fy = value; notifyChange(key); return; }
        if (key == "camera.cx") { config_->camera.cx = value; notifyChange(key); return; }
        if (key == "camera.cy") { config_->camera.cy = value; notifyChange(key); return; }
        if (key == "estimator.solver_time") { config_->estimator.solver_time = value; notifyChange(key); return; }
        if (key == "estimator.min_parallax") { config_->estimator.min_parallax = value; notifyChange(key); return; }
        if (key == "estimator.init_depth") { config_->estimator.init_depth = value; notifyChange(key); return; }
        if (key == "estimator.acc_n") { config_->estimator.acc_n = value; notifyChange(key); return; }
        if (key == "estimator.acc_w") { config_->estimator.acc_w = value; notifyChange(key); return; }
        if (key == "estimator.gyr_n") { config_->estimator.gyr_n = value; notifyChange(key); return; }
        if (key == "estimator.gyr_w") { config_->estimator.gyr_w = value; notifyChange(key); return; }
        if (key == "feature_tracker.f_threshold") { config_->feature_tracker.f_threshold = value; notifyChange(key); return; }
    }
    else if constexpr (std::is_same_v<T, std::string>) {
        if (key == "dataset_path") { config_->dataset_path = value; notifyChange(key); return; }
        if (key == "config_filepath") { config_->config_filepath = value; notifyChange(key); return; }
        if (key == "feature_tracker.fisheye_mask") { config_->feature_tracker.fisheye_mask = value; notifyChange(key); return; }
    }
}

} // namespace config

#endif // CONFIG__CONFIG_MANAGER_H