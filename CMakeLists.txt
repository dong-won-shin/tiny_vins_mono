cmake_minimum_required(VERSION 2.8)
project(tiny_vins_mono)

# Enable ccache for faster rebuilds
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache for faster builds")
else()
    message(STATUS "ccache not found, builds will not be cached")
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

# Libraries 
add_library(utility src/utility/utility.cpp src/utility/config.cpp)
target_link_libraries(utility ${YAML_CPP_LIBRARIES})

add_library(pose_local_parameterization src/backend/factor/pose_local_parameterization.cpp)
target_link_libraries(pose_local_parameterization ceres)

add_library(marginalization_factor src/backend/factor/marginalization_factor.cpp)
target_link_libraries(marginalization_factor ceres)

add_library(projection_factor src/backend/factor/projection_factor.cpp)
target_link_libraries(projection_factor ceres)

add_library(optimizer src/backend/optimizer.cpp)
target_link_libraries(optimizer ceres marginalization_factor pose_local_parameterization)

add_library(failure_detector src/frontend/failure_detector.cpp)

add_library(initializer
    src/frontend/initialization/initializer.cpp
    src/frontend/initialization/initial_alignment.cpp
    src/frontend/initialization/initial_sfm.cpp
    src/frontend/initialization/solve_5pts.cpp
)
target_link_libraries(initializer ceres utility ${OpenCV_LIBS})

add_library(gpl 
    src/common/gpl/gpl.cc
    src/common/gpl/EigenQuaternionParameterization.cc
)
target_link_libraries(gpl ceres)

add_library(camera_models
    src/common/camera_models/CameraFactory.cc
    src/common/camera_models/CataCamera.cc
    src/common/camera_models/PinholeCamera.cc
    src/common/camera_models/Camera.cc
    src/common/camera_models/EquidistantCamera.cc
    src/common/camera_models/ScaramuzzaCamera.cc
    src/common/camera_models/CostFunctionFactory.cc
)
target_link_libraries(camera_models gpl)

add_library(feature_tracker
    src/frontend/feature_tracker.cpp
)
target_link_libraries(feature_tracker
    utility
    ceres
    camera_models
)

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
    ${OpenCV_LIBS}
)

# Executable
add_executable(tiny_vins_mono 
    src/tiny_vins_mono.cpp 
    src/utility/measurement_processor.cpp
    src/utility/visualizer.cpp
    src/utility/test_result_logger.cpp
)
target_link_libraries(tiny_vins_mono 
    ${OpenCV_LIBS} 
    ${YAML_CPP_LIBRARIES} 
    ${EIGEN3_LIBRARIES} 
    estimator
    feature_tracker
    utility
    ${Pangolin_LIBRARIES}
    stdc++fs
)