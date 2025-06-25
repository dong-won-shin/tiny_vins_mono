cmake_minimum_required(VERSION 2.8)
project(tiny_vins_mono)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# Utility library
add_library(utility src/utility/utility.cpp src/utility/config.cpp)

add_library(pose_local_parameterization src/backend/factor/pose_local_parameterization.cpp)
target_link_libraries(pose_local_parameterization ceres)

add_library(marginalization_factor src/backend/factor/marginalization_factor.cpp)
target_link_libraries(marginalization_factor ceres)

add_library(projection_factor src/backend/factor/projection_factor.cpp)
target_link_libraries(projection_factor ceres)

add_library(optimizer src/backend/optimizer.cpp)
target_link_libraries(optimizer ceres)

add_library(failure_detector src/frontend/failure_detector.cpp)

add_library(initializer
    src/frontend/initialization/initializer.cpp
    src/frontend/initialization/initial_alignment.cpp
    src/frontend/initialization/initial_sfm.cpp
    src/frontend/initialization/solve_5pts.cpp
)
target_link_libraries(initializer ceres utility)


add_library(estimator
    src/backend/estimator.cpp
    src/common/frame.cpp
    src/frontend/feature_manager.cpp
    src/backend/sliding_window.cpp
)
target_link_libraries(estimator
    utility
    ceres
    pose_local_parameterization
    marginalization_factor
    projection_factor
    optimizer
    failure_detector
    initializer
)

add_executable(tiny_vins_mono src/tiny_vins_mono.cpp)
target_link_libraries(tiny_vins_mono ${OpenCV_LIBS} yaml-cpp ${EIGEN3_LIBRARIES} estimator)