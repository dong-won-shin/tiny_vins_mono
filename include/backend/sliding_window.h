#ifndef SLIDING_WINDOW_H
#define SLIDING_WINDOW_H

#include <Eigen/Dense>

#include "common/frame.h"

namespace backend {

class SlidingWindow {
public:
    SlidingWindow() {
        clearSlidingWindow();
    }

    void clearSlidingWindow();

    Frame& operator[](int index) {
        return sliding_window[index];
    }

    const Frame& operator[](int index) const {
        return sliding_window[index];
    }

    void clearBuffer(int32_t index) {
        sliding_window[index].dt_buf.clear();
        sliding_window[index].linear_acceleration_buf.clear();
        sliding_window[index].angular_velocity_buf.clear();
    }

    void copyFrame(int32_t index, int32_t other_index) {
        sliding_window[index].timestamp = sliding_window[other_index].timestamp;
        sliding_window[index].R = sliding_window[other_index].R;
        sliding_window[index].P = sliding_window[other_index].P;
        sliding_window[index].V = sliding_window[other_index].V;
        sliding_window[index].Ba = sliding_window[other_index].Ba;
        sliding_window[index].Bg = sliding_window[other_index].Bg;
    }

    void swapFrame(int32_t index, int32_t other_index) {
        std::swap(sliding_window[index].timestamp, sliding_window[other_index].timestamp);
        std::swap(sliding_window[index].R, sliding_window[other_index].R);
        std::swap(sliding_window[index].P, sliding_window[other_index].P);
        std::swap(sliding_window[index].V, sliding_window[other_index].V);
        std::swap(sliding_window[index].Ba, sliding_window[other_index].Ba);
        std::swap(sliding_window[index].Bg, sliding_window[other_index].Bg);
        std::swap(sliding_window[index].pre_integration, sliding_window[other_index].pre_integration);
    }

    void swapBuffer(int32_t index, int32_t other_index) {
        std::swap(sliding_window[index].dt_buf, sliding_window[other_index].dt_buf);
        std::swap(sliding_window[index].linear_acceleration_buf, sliding_window[other_index].linear_acceleration_buf);
        std::swap(sliding_window[index].angular_velocity_buf, sliding_window[other_index].angular_velocity_buf);
    }

    void pushBackBuffer(int32_t index, double dt, const Eigen::Vector3d& linear_acceleration,
                        const Eigen::Vector3d& angular_velocity) {
        sliding_window[index].dt_buf.push_back(dt);
        sliding_window[index].linear_acceleration_buf.push_back(linear_acceleration);
        sliding_window[index].angular_velocity_buf.push_back(angular_velocity);
    }

    void pushBackPreintegration(int32_t index, double dt, const Eigen::Vector3d& linear_acceleration,
                                const Eigen::Vector3d& angular_velocity) {
        sliding_window[index].pre_integration->push_back(dt, linear_acceleration, angular_velocity);
    }

    void createNewPreintegration(int32_t index, const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity) {
        delete sliding_window[index].pre_integration;
        sliding_window[index].pre_integration = new IntegrationBase(linear_acceleration, angular_velocity,
                                                                    sliding_window[index].Ba, sliding_window[index].Bg);
    }

    const Frame& front() const {
        return sliding_window[0];
    }

    const Frame& back() const {
        return sliding_window[WINDOW_SIZE];
    }

private:
    std::array<Frame, WINDOW_SIZE + 1> sliding_window;
};

}  // namespace backend

#endif