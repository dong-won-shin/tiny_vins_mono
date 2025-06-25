#ifndef MEASUREMENT_PROCESSOR_H
#define MEASUREMENT_PROCESSOR_H

#include "utility/measurement_reader.h"
#include "frontend/feature_tracker.h"
#include "common/image_frame.h"
#include <vector>
#include <string>

struct IMUData {
    double timestamp;
    double linear_acc_x, linear_acc_y, linear_acc_z;
    double angular_vel_x, angular_vel_y, angular_vel_z;
};

struct ImageFileData {
    double timestamp;
    std::string filename;
    std::string full_path;
};

class MeasurementProcessor {
public:
    MeasurementProcessor();
    ~MeasurementProcessor();
    
    // 초기화
    bool initialize(const std::string& imu_filepath, 
                   const std::string& image_csv_filepath,
                   const std::string& image_dir,
                   const std::string& config_filepath);
    
    // 데이터 로드
    bool loadImuData(const std::string& filepath);
    bool loadImageFileData(const std::string& csv_filepath, const std::string& image_dir);
    
    // MeasurementMsg 생성
    MeasurementMsg createMeasurementMsg(int measurement_id, 
                                       const ImageFileData& image_data);
    
    // 데이터 접근자
    const std::vector<IMUData>& getIMUData() const { return imu_data_; }
    const std::vector<ImageFileData>& getImageFileData() const { return image_file_data_; }
    
    // 유틸리티 함수
    void printDataRange() const;
    void printTimeInfo(double timestamp) const;
    
private:
    // 멤버 변수
    std::vector<IMUData> imu_data_;
    std::vector<ImageFileData> image_file_data_;
    std::vector<double> image_timestamps_;
    std::vector<std::string> image_files_;
    
    // 이전 이미지 타임스탬프 저장
    double prev_image_timestamp_;
    
    // Feature tracker 관련
    std::unique_ptr<feature_tracker::FeatureTracker> feature_tracker_;
    
    // 이미지 특징점 추출
    ImageFeatureMsg extractImageFeatures(const ImageFileData& image_data);
    
    // 파일명 정리 (공백, 개행 제거)
    std::string cleanFilename(const std::string& filename);
};

#endif // MEASUREMENT_PROCESSOR_H 