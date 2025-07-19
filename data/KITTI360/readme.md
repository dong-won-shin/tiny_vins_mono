# KITTI-360 Dataset

## Overview
The KITTI-360 dataset is a large-scale dataset for autonomous driving research, containing synchronized data from multiple sensors including cameras, LiDAR, and IMU. It provides rich visual and geometric information for various computer vision and robotics tasks.

## Download
You can download the KITTI-360 dataset from the official website:
- **Official Website**: [https://www.cvlibs.net/datasets/kitti-360/index.php](https://www.cvlibs.net/datasets/kitti-360/index.php)

## Dataset Structure
The KITTI-360 dataset contains:
- **Visual data**: High-resolution stereo images
- **IMU data**: Inertial measurements

## Usage with Tiny-VINS-Mono

**Note**: Tiny-VINS-Mono does not directly support the KITTI-360 format. You have two options to use KITTI-360 data:

### Option 1: Convert KITTI-360 to EuRoC Format (Recommended)

Use the provided conversion script to transform KITTI-360 data into EuRoC format:

#### Conversion Script
```bash
python script/dataset_conversion/kitti360_to_euroc.py <kitti_path> <output_path> <sequence>
```

#### Usage Example
```bash
python script/dataset_conversion/kitti360_to_euroc.py \
    /path/to/kitti360/dataset \
    data/KITTI360/converted \
    2013_05_28_drive_0000
```

#### Script Arguments
- **kitti_path**: Path to KITTI-360 dataset base directory
- **output_path**: Output path for EuRoC format dataset
- **sequence**: Sequence name (e.g., '2013_05_28_drive_0000')

#### Conversion Process
The script performs the following operations:
1. Reads KITTI-360 IMU data and converts to EuRoC format
2. Processes camera images and creates EuRoC-style timestamps
3. Organizes data into the expected directory structure
4. Generates compatible CSV files for IMU and camera data

### Option 2: Use Pre-converted Sample Dataset
For quick testing and evaluation, you can download a pre-converted KITTI-360 sample dataset that is ready to use with Tiny-VINS-Mono:

**Download Link**: [2013_05_28_drive_0010.zip](https://www.dropbox.com/scl/fi/lyj7oq1zbht7i5u1y7rbo/2013_05_28_drive_0010.zip?rlkey=iw0rlzbo3rvxp634xtu6g997b&st=m5ggmeiv&dl=0)

**Instructions**:
1. Download the zip file from the link above
2. Extract it to `data/KITTI360/` directory
3. Update your configuration file to use the dataset:
   ```yaml
   dataset_path: "data/KITTI360/2013_05_28_drive_0010"
   ```

## Citation
If you use the KITTI-360 dataset in your research, please cite:
```
@inproceedings{xie2020kitti360,
  title={KITTI-360: A novel dataset and benchmarks for urban scene understanding in 2D and 3D},
  author={Xie, Yan and Kiefel, Martin and Sun, Ming and Geiger, Andreas},
  booktitle={IEEE Transactions on Pattern Analysis and Machine Intelligence},
  volume={44},
  number={7},
  pages={3292--3310},
  year={2022},
  publisher={IEEE}
}
```
