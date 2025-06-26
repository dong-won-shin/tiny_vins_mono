#ifndef COMMON__CAMERA_MODELS__CAMERA_FACTORY_H
#define COMMON__CAMERA_MODELS__CAMERA_FACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "common/camera_models/Camera.h"

namespace common {
namespace camera_models {

class CameraFactory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType, const std::string& cameraName, cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}  // namespace camera_models
}  // namespace common

#endif  // COMMON__CAMERA_MODELS__CAMERA_FACTORY_H
