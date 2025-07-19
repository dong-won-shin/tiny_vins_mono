#!/usr/bin/env python3
"""
Convert KITTI-360 dataset to EuRoC MAV format for monocular-IMU VIO
"""

import os
import argparse
import shutil
from pathlib import Path
import numpy as np
import yaml
from datetime import datetime
import csv


class KITTI360ToEurocConverter:
    def __init__(self, kitti_base_path, output_path, sequence_name):
        self.kitti_base = Path(kitti_base_path)
        self.output_path = Path(output_path)
        self.sequence_name = sequence_name
        
        # KITTI-360 paths
        self.image_path = self.kitti_base / "data_2d_raw" / f"{sequence_name}_sync" / "image_00"
        self.oxts_path = self.kitti_base / "data_poses" / f"{sequence_name}_extract" / "oxts"
        
        # EuRoC output paths
        self.euroc_base = self.output_path / "mav0"
        self.cam0_path = self.euroc_base / "cam0"
        self.imu0_path = self.euroc_base / "imu0"
        
    def create_directory_structure(self):
        """Create EuRoC directory structure"""
        print("Creating EuRoC directory structure...")
        (self.cam0_path / "data").mkdir(parents=True, exist_ok=True)
        self.imu0_path.mkdir(parents=True, exist_ok=True)
        
    def convert_timestamp_to_nanoseconds(self, timestamp_str):
        """Convert KITTI-360 timestamp string to nanoseconds since epoch"""
        # Parse timestamp: "2013-05-28 08:46:02.904483072"
        dt = datetime.strptime(timestamp_str[:19], "%Y-%m-%d %H:%M:%S")
        # Add microseconds
        microseconds = int(timestamp_str[20:26])
        nanoseconds = int(timestamp_str[26:29]) if len(timestamp_str) > 26 else 0
        
        # Convert to nanoseconds since epoch
        epoch_seconds = dt.timestamp()
        total_nanoseconds = int(epoch_seconds * 1e9) + microseconds * 1000 + nanoseconds
        
        return total_nanoseconds
    
    def process_camera_data(self):
        """Process camera images and timestamps"""
        print("Processing camera data...")
        
        # Read image timestamps
        timestamp_file = self.image_path / "timestamps.txt"
        if not timestamp_file.exists():
            raise FileNotFoundError(f"Camera timestamp file not found: {timestamp_file}")
            
        image_data = []
        with open(timestamp_file, 'r') as f:
            for idx, line in enumerate(f):
                timestamp_str = line.strip()
                timestamp_ns = self.convert_timestamp_to_nanoseconds(timestamp_str)
                
                # Copy image file
                src_image = self.image_path / "data_rect" / f"{idx:010d}.png"
                dst_image = self.cam0_path / "data" / f"{timestamp_ns}.png"
                
                if src_image.exists():
                    shutil.copy2(src_image, dst_image)
                    image_data.append([timestamp_ns, f"{timestamp_ns}.png"])
                    
        # Write cam0/data.csv
        with open(self.cam0_path / "data.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["#timestamp [ns]", "filename"])
            writer.writerows(image_data)
            
        print(f"Processed {len(image_data)} camera images")
        
    def process_imu_data(self):
        """Process IMU data from OXTS files"""
        print("Processing IMU data...")
        
        # Read IMU timestamps
        timestamp_file = self.oxts_path / "timestamps.txt"
        if not timestamp_file.exists():
            raise FileNotFoundError(f"IMU timestamp file not found: {timestamp_file}")
            
        imu_data = []
        with open(timestamp_file, 'r') as f:
            timestamps = [line.strip() for line in f]
            
        # Process each OXTS file
        oxts_data_path = self.oxts_path / "data"
        for idx, timestamp_str in enumerate(timestamps):
            oxts_file = oxts_data_path / f"{idx:010d}.txt"
            
            if oxts_file.exists():
                with open(oxts_file, 'r') as f:
                    values = list(map(float, f.readline().strip().split()))
                    
                if len(values) >= 20:  # Ensure we have enough data
                    timestamp_ns = self.convert_timestamp_to_nanoseconds(timestamp_str)
                    
                    # Extract IMU data (angular velocities and accelerations)
                    # KITTI-360 OXTS format:
                    # wx, wy, wz at indices 17, 18, 19 (angular rates in rad/s)
                    # ax, ay, az at indices 11, 12, 13 (accelerations in m/s^2)
                    wx, wy, wz = values[17], values[18], values[19]
                    ax, ay, az = values[11], values[12], values[13]
                    
                    imu_data.append([timestamp_ns, wx, wy, wz, ax, ay, az])
                    
        # Write imu0/data.csv
        with open(self.imu0_path / "data.csv", 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", 
                           "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", 
                           "a_RS_S_z [m s^-2]"])
            writer.writerows(imu_data)
            
        print(f"Processed {len(imu_data)} IMU measurements")
        
    def convert(self):
        """Main conversion function"""
        print(f"Converting KITTI-360 sequence '{self.sequence_name}' to EuRoC format...")
        print(f"Input path: {self.kitti_base}")
        print(f"Output path: {self.output_path}")
        
        # Create directory structure
        self.create_directory_structure()
        
        # Process data
        self.process_camera_data()
        self.process_imu_data()
        
        print("Conversion completed successfully!")


def main():
    parser = argparse.ArgumentParser(description="Convert KITTI-360 to EuRoC format for monocular-IMU VIO")
    parser.add_argument("kitti_path", help="Path to KITTI-360 dataset base directory")
    parser.add_argument("output_path", help="Output path for EuRoC format dataset")
    parser.add_argument("sequence", help="Sequence name (e.g., '2013_05_28_drive_0000')")
    
    args = parser.parse_args()
    
    converter = KITTI360ToEurocConverter(args.kitti_path, args.output_path, args.sequence)
    converter.convert()


if __name__ == "__main__":
    main()