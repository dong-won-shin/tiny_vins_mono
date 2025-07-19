# EuRoC Dataset

## Overview
The EuRoC Micro Aerial Vehicle (MAV) Visual-Inertial Dataset is a collection of visual-inertial sensor data recorded on board a micro aerial vehicle. This dataset is widely used for benchmarking visual-inertial odometry (VIO) and simultaneous localization and mapping (SLAM) algorithms.

## Download
You can download the EuRoC dataset from the official website:
- **Official Website**: [https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

## Dataset Structure
The EuRoC dataset contains the following sequences:

### Machine Hall (MH) Sequences
- **MH_01_easy**: Simple trajectory with good lighting
- **MH_02_easy**: Similar to MH_01 but with different trajectory
- **MH_03_medium**: Medium difficulty with some challenging sections
- **MH_04_difficult**: Difficult trajectory with rapid motion
- **MH_05_difficult**: Another difficult sequence with aggressive motion

### Vicon Room (V1) Sequences
- **V1_01_easy**: Simple trajectory in Vicon room
- **V1_02_medium**: Medium difficulty in Vicon room
- **V1_03_difficult**: Difficult trajectory in Vicon room

### Vicon Room 2 (V2) Sequences
- **V2_01_easy**: Simple trajectory in second Vicon room
- **V2_02_medium**: Medium difficulty in second Vicon room
- **V2_03_difficult**: Difficult trajectory in second Vicon room

## Data Format
Each sequence contains:
- **IMU data**: `mav0/imu0/data.csv` - Accelerometer and gyroscope measurements
- **Camera data**: `mav0/cam0/data.csv` - Image timestamps and filenames
- **Image files**: `mav0/cam0/data/` - Actual image files

## Usage with Tiny-VINS-Mono
Tiny-VINS-Mono directly supports the EuRoC dataset format. Simply place your EuRoC dataset in this directory and update the configuration file to point to the desired sequence.

### Example Configuration
```yaml
dataset_path: "data/EuRoC/MH_01_easy"
```

## Citation
If you use the EuRoC dataset in your research, please cite:
```
@inproceedings{burri2016euroc,
  title={The EuRoC micro aerial vehicle datasets},
  author={Burri, Michael and Nikolic, Janosch and Gohl, Pascal and Schneider, Thomas and Rehder, Joern and Omari, Sammy and Achtelik, Markus W and Siegwart, Roland},
  booktitle={The International Journal of Robotics Research},
  volume={35},
  number={10},
  pages={1157--1163},
  year={2016},
  publisher={SAGE Publications Sage UK: London, England}
}
```