#ifndef SLIDING_WINDOW_H
#define SLIDING_WINDOW_H

#include <Eigen/Dense>

#include "backend/factor/integration_base.h"
#include "common/frame.h"
#include "utility/config.h"

namespace backend {

class SlidingWindow {
public:
    SlidingWindow();

    void clearSlidingWindow();

    common::Frame& operator[](int index) {
        return sliding_window[index];
    }

    const common::Frame& operator[](int index) const {
        return sliding_window[index];
    }

    void clearBuffer(int32_t index);
    void copyFrame(int32_t index, int32_t other_index);
    void swapFrame(int32_t index, int32_t other_index);
    void swapBuffer(int32_t index, int32_t other_index);

    void pushBackBuffer(int32_t index, double dt, const Eigen::Vector3d& linear_acceleration,
                        const Eigen::Vector3d& angular_velocity);

    void pushBackPreintegration(int32_t index, double dt, const Eigen::Vector3d& linear_acceleration,
                                const Eigen::Vector3d& angular_velocity);

    void createNewPreintegration(int32_t index, const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity);

    const common::Frame& front() const;
    const common::Frame& back() const;

private:
    std::array<common::Frame, WINDOW_SIZE + 1> sliding_window;
};

}  // namespace backend

#endif