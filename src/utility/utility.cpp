#include "utility/utility.h"

#include <dirent.h>

#include <algorithm>
#include <cstring>

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d& g) {
  Eigen::Matrix3d R0;
  Eigen::Vector3d ng1 = g.normalized();
  Eigen::Vector3d ng2{0, 0, 1.0};
  R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
  return R0;
}

std::vector<std::string> utility::getImageFilesFromDirectory(const std::string& directory_path) {
  std::vector<std::string> image_files;

  DIR* dir = opendir(directory_path.c_str());
  if (dir == nullptr) {
    std::cerr << "Error opening directory " << directory_path << ": " << strerror(errno)
              << std::endl;
    return image_files;
  }

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_type == DT_REG) {  // Regular file
      std::string filename = entry->d_name;
      std::string file_path = directory_path + "/" + filename;

      // Check file extension
      std::string extension;
      size_t dot_pos = filename.find_last_of('.');
      if (dot_pos != std::string::npos) {
        extension = filename.substr(dot_pos);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
            extension == ".bmp" || extension == ".tiff" || extension == ".tif") {
          image_files.push_back(file_path);
        }
      }
    }
  }

  closedir(dir);

  // Sort files by name to ensure chronological order
  std::sort(image_files.begin(), image_files.end());

  std::cout << "Found " << image_files.size() << " image files in directory: " << directory_path
            << std::endl;

  return image_files;
}