#!/usr/bin/env python3
"""
Trajectory Comparison Script for VINS

This script compares the optimized pose trajectory from VINS with the ground truth trajectory
from the Euroc dataset. It provides visualization and error metrics.

Usage:
    python compare_trajectories.py [experiment_path] [--save] [--no-display]

Arguments:
    experiment_path: Optional. Path to the experiment results directory (e.g., logs/TIMESTAMP).
                     If not provided, the script will automatically use the latest
                     directory in 'logs/'.

Flags:
    --save:        Save comparison plots and results to files in the experiment directory.
    --no-display:  Do not show plots in an interactive window.

Example:
    # Compare with latest run, automatically finding paths
    python scripts/compare_trajectories.py

    # Compare with a specific result directory
    python scripts/compare_trajectories.py logs/20250623_120815

    # Compare with a specific run and save results without showing plots
    python scripts/compare_trajectories.py logs/20250623_120815 --save --no-display
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import yaml
import warnings
warnings.filterwarnings('ignore')

class TrajectoryComparator:
    def __init__(self, dataset_path, optimized_pose_file, output_dir="."):
        self.dataset_path = dataset_path
        self.optimized_pose_file = optimized_pose_file
        self.output_dir = output_dir
        self.gt_file = os.path.join(dataset_path, "mav0/state_groundtruth_estimate0/data.csv")
        
        # Load extrinsic parameters
        self.t_ic = None
        self.r_ic = None
        self.load_extrinsic_parameters()
        
        # Load data
        self.gt_data = None
        self.opt_data = None
        self.load_data()
        
    def load_extrinsic_parameters(self):
        # No extrinsic transform needed for translation alignment only
        self.r_ic = np.eye(3)
        self.t_ic = np.zeros(3)
        return

    def load_data(self):
        """Load ground truth and optimized pose data"""
        print("📊 Loading trajectory data...")
        
        # Load ground truth data
        if not os.path.exists(self.gt_file):
            raise FileNotFoundError(f"Ground truth file not found: {self.gt_file}")
        
        try:
            # Always use the first line starting with '#' as header
            with open(self.gt_file, 'r') as f:
                for line in f:
                    if line.startswith('#'):
                        header = line.replace('#', '').strip().split(',')
                        header = [h.strip() for h in header]
                        break
            self.gt_data = pd.read_csv(self.gt_file, names=header, skiprows=1)
            self.gt_data.columns = [c.strip() for c in self.gt_data.columns]
            print(f"✅ Ground truth data loaded: {len(self.gt_data)} poses")
        except Exception as e:
            raise Exception(f"Error loading ground truth data: {e}")
        
        # Load optimized pose data
        if not os.path.exists(self.optimized_pose_file):
            raise FileNotFoundError(f"Optimized pose file not found: {self.optimized_pose_file}")
        
        try:
            self.opt_data = self.parse_optimized_pose_file()
            print(f"✅ Optimized pose data loaded: {len(self.opt_data)} poses")
            # No extrinsic transform, just keep as is
        except Exception as e:
            raise Exception(f"Error loading optimized pose data: {e}")
    
    def parse_optimized_pose_file(self):
        """Parse the optimized pose file (TUM format: timestamp tx ty tz qx qy qz qw)."""
        data = []
        with open(self.optimized_pose_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                
                parts = line.split()
                if len(parts) != 8:
                    print(f"⚠️ Skipping malformed line in pose file: {line}")
                    continue
                
                try:
                    # timestamp tx ty tz qx qy qz qw
                    row = [float(p) for p in parts]
                    data.append(row)
                except ValueError:
                    print(f"⚠️ Could not parse values in line: {line}")

        return pd.DataFrame(data, columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    
    def align_trajectories(self):
        """Align trajectories by timestamp and coordinate frame (rotation and translation)"""
        print("🔄 Aligning trajectories...")
        
        # Convert ground truth timestamps to seconds (from nanoseconds)
        self.gt_data['timestamp_sec'] = self.gt_data['timestamp'] / 1e9 if 'timestamp' in self.gt_data.columns else self.gt_data.iloc[:,0] / 1e9
        
        # Find common time range
        gt_start = self.gt_data['timestamp_sec'].min()
        gt_end = self.gt_data['timestamp_sec'].max()
        opt_start = self.opt_data['timestamp'].min()
        opt_end = self.opt_data['timestamp'].max()
        
        print(f"Ground truth time range: {gt_start:.3f} - {gt_end:.3f}")
        print(f"Optimized time range: {opt_start:.3f} - {opt_end:.3f}")
        
        # Find overlapping time range
        start_time = max(gt_start, opt_start)
        end_time = min(gt_end, opt_end)
        
        if start_time >= end_time:
            raise ValueError("No overlapping time range between trajectories")
        
        print(f"Overlapping time range: {start_time:.3f} - {end_time:.3f}")
        
        # Filter data to overlapping range
        gt_filtered = self.gt_data[
            (self.gt_data['timestamp_sec'] >= start_time) & 
            (self.gt_data['timestamp_sec'] <= end_time)
        ].copy()
        
        opt_filtered = self.opt_data[
            (self.opt_data['timestamp'] >= start_time) & 
            (self.opt_data['timestamp'] <= end_time)
        ].copy()
        
        # Use exact column names for position
        x_col = 'p_RS_R_x [m]'
        y_col = 'p_RS_R_y [m]'
        z_col = 'p_RS_R_z [m]'
        if not (x_col in self.gt_data.columns and y_col in self.gt_data.columns and z_col in self.gt_data.columns):
            raise Exception('Position columns not found in ground truth data')
        gt_interp = interp1d(
            gt_filtered['timestamp_sec'], 
            gt_filtered[[x_col, y_col, z_col]].values,
            axis=0, 
            bounds_error=False, 
            fill_value='extrapolate'
        )
        
        gt_positions_interp = gt_interp(opt_filtered['timestamp'])
        
        # Align: shift predicted trajectory so its first position matches gt's first position
        pred_xyz = opt_filtered[['x','y','z']].values
        gt_xyz = gt_positions_interp

        if len(pred_xyz) < 2 or len(gt_xyz) < 2:
            print("⚠️ Not enough data points for rotation alignment, using translation only.")
            if len(pred_xyz) > 0 and len(gt_xyz) > 0:
                translation = gt_xyz[0] - pred_xyz[0]
                pred_xyz_aligned = pred_xyz + translation
            else:
                pred_xyz_aligned = pred_xyz
        else:
            # Align using Z-axis rotation and translation
            vec_pred = pred_xyz[1] - pred_xyz[0]
            vec_gt = gt_xyz[1] - gt_xyz[0]
            
            # Project vectors onto XY plane
            vec_pred_xy = vec_pred[:2]
            vec_gt_xy = vec_gt[:2]
            
            if np.linalg.norm(vec_pred_xy) < 1e-8 or np.linalg.norm(vec_gt_xy) < 1e-8:
                print("⚠️ Initial XY movement vector is too small for Z-axis rotation, falling back to translation only.")
                translation = gt_xyz[0] - pred_xyz[0]
                pred_xyz_aligned = pred_xyz + translation
            else:
                # Calculate angles on XY plane
                angle_pred = np.arctan2(vec_pred_xy[1], vec_pred_xy[0])
                angle_gt = np.arctan2(vec_gt_xy[1], vec_gt_xy[0])
                
                # Calculate angle difference for Z-axis rotation
                angle_diff = angle_gt - angle_pred
                
                # Create rotation object for Z-axis rotation
                rotation = R.from_euler('z', angle_diff)
                
                # Center the predicted trajectory at its start point
                pred_xyz_centered = pred_xyz - pred_xyz[0]
                
                # Rotate the centered trajectory
                pred_xyz_rotated = rotation.apply(pred_xyz_centered)
                
                # Translate the rotated trajectory to the start point of the ground truth
                pred_xyz_aligned = pred_xyz_rotated + gt_xyz[0]
        
        # Create aligned dataframes
        self.aligned_gt = pd.DataFrame({
            'timestamp': opt_filtered['timestamp'],
            'x': gt_xyz[:, 0],
            'y': gt_xyz[:, 1],
            'z': gt_xyz[:, 2]
        })
        self.aligned_opt = pd.DataFrame({
            'timestamp': opt_filtered['timestamp'],
            'x': pred_xyz_aligned[:, 0],
            'y': pred_xyz_aligned[:, 1],
            'z': pred_xyz_aligned[:, 2]
        })
        
        print(f"✅ Aligned trajectories: {len(self.aligned_gt)} poses")
        
        return start_time, end_time
    
    def calculate_errors(self):
        """Calculate trajectory errors"""
        print("📈 Calculating error metrics...")
        
        # Calculate position errors
        position_errors = np.sqrt(
            (self.aligned_opt['x'] - self.aligned_gt['x'])**2 +
            (self.aligned_opt['y'] - self.aligned_gt['y'])**2 +
            (self.aligned_opt['z'] - self.aligned_gt['z'])**2
        )
        
        # Calculate individual axis errors
        x_errors = np.abs(self.aligned_opt['x'] - self.aligned_gt['x'])
        y_errors = np.abs(self.aligned_opt['y'] - self.aligned_gt['y'])
        z_errors = np.abs(self.aligned_opt['z'] - self.aligned_gt['z'])
        
        # Calculate statistics
        stats = {
            'mean_position_error': np.mean(position_errors),
            'std_position_error': np.std(position_errors),
            'max_position_error': np.max(position_errors),
            'median_position_error': np.median(position_errors),
            'rmse_position': np.sqrt(np.mean(position_errors**2)),
            'mean_x_error': np.mean(x_errors),
            'mean_y_error': np.mean(y_errors),
            'mean_z_error': np.mean(z_errors),
            'std_x_error': np.std(x_errors),
            'std_y_error': np.std(y_errors),
            'std_z_error': np.std(z_errors)
        }
        
        print("📊 Error Statistics:")
        print(f"  Mean Position Error: {stats['mean_position_error']:.4f} m")
        print(f"  Std Position Error: {stats['std_position_error']:.4f} m")
        print(f"  Max Position Error: {stats['max_position_error']:.4f} m")
        print(f"  RMSE Position: {stats['rmse_position']:.4f} m")
        print(f"  Mean X Error: {stats['mean_x_error']:.4f} m")
        print(f"  Mean Y Error: {stats['mean_y_error']:.4f} m")
        print(f"  Mean Z Error: {stats['mean_z_error']:.4f} m")
        
        return stats, position_errors, x_errors, y_errors, z_errors
    
    def visualize_trajectories(self, save_plots=True, show_plots=True):
        """Visualize both trajectories"""
        print("🎨 Creating visualizations...")
        
        # Create figure with subplots - only top row plots
        fig = plt.figure(figsize=(20, 8))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(1, 3, 1, projection='3d')
        ax1.plot(self.aligned_gt['x'], self.aligned_gt['y'], self.aligned_gt['z'], 
                'b-', label='Ground Truth', linewidth=2, alpha=0.8)
        ax1.plot(self.aligned_opt['x'], self.aligned_opt['y'], self.aligned_opt['z'], 
                'r-', label='Optimized', linewidth=2, alpha=0.8)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_title('3D Trajectory Comparison')
        ax1.legend()
        ax1.grid(True)

        xlim = ax1.get_xlim3d()
        ylim = ax1.get_ylim3d()
        zlim = ax1.get_zlim3d()
        xmid = 0.5 * (xlim[0] + xlim[1])
        ymid = 0.5 * (ylim[0] + ylim[1])
        zmid = 0.5 * (zlim[0] + zlim[1])
        max_range = max(xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]) / 2
        ax1.set_xlim3d([xmid - max_range, xmid + max_range])
        ax1.set_ylim3d([ymid - max_range, ymid + max_range])
        ax1.set_zlim3d([zmid - max_range, zmid + max_range])

        # XY projection
        ax2 = fig.add_subplot(1, 3, 2)
        ax2.plot(self.aligned_gt['x'], self.aligned_gt['y'], 'b-', label='Ground Truth', linewidth=2)
        ax2.plot(self.aligned_opt['x'], self.aligned_opt['y'], 'r-', label='Optimized', linewidth=2)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Y [m]')
        ax2.set_title('XY Projection')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # XZ projection
        ax3 = fig.add_subplot(1, 3, 3)
        ax3.plot(self.aligned_gt['x'], self.aligned_gt['z'], 'b-', label='Ground Truth', linewidth=2)
        ax3.plot(self.aligned_opt['x'], self.aligned_opt['z'], 'r-', label='Optimized', linewidth=2)
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_title('XZ Projection')
        ax3.legend()
        ax3.grid(True)
        ax3.axis('equal')
        
        plt.tight_layout()
        
        if save_plots:
            plot_filename = os.path.join(self.output_dir, f"trajectory_comparison_{os.path.basename(self.dataset_path)}.png")
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"✅ Visualization saved as: {plot_filename}")
        
        if show_plots:
            plt.show()
        else:
            plt.close()
        
        # Calculate basic error statistics for results saving
        stats, _, _, _, _ = self.calculate_errors()
        return stats
    
    def save_comparison_results(self, stats, output_file=None):
        """Save comparison results to file"""
        if output_file is None:
            output_file = os.path.join(self.output_dir, f"trajectory_comparison_results_{os.path.basename(self.dataset_path)}.txt")
        
        with open(output_file, 'w') as f:
            f.write("VINS Trajectory Comparison Results\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Dataset: {self.dataset_path}\n")
            f.write(f"Optimized pose file: {self.optimized_pose_file}\n")
            f.write(f"Ground truth file: {self.gt_file}\n\n")
            
            f.write("Trajectory Statistics:\n")
            f.write(f"  Number of poses (optimized): {len(self.aligned_opt)}\n")
            f.write(f"  Number of poses (ground truth): {len(self.aligned_gt)}\n")
            f.write(f"  Time range: {self.aligned_opt['timestamp'].min():.3f} - {self.aligned_opt['timestamp'].max():.3f} s\n\n")
            
            f.write("Error Metrics:\n")
            f.write(f"  Mean Position Error: {stats['mean_position_error']:.6f} m\n")
            f.write(f"  Std Position Error: {stats['std_position_error']:.6f} m\n")
            f.write(f"  Max Position Error: {stats['max_position_error']:.6f} m\n")
            f.write(f"  Median Position Error: {stats['median_position_error']:.6f} m\n")
            f.write(f"  RMSE Position: {stats['rmse_position']:.6f} m\n")
            f.write(f"  Mean X Error: {stats['mean_x_error']:.6f} m\n")
            f.write(f"  Mean Y Error: {stats['mean_y_error']:.6f} m\n")
            f.write(f"  Mean Z Error: {stats['mean_z_error']:.6f} m\n")
            f.write(f"  Std X Error: {stats['std_x_error']:.6f} m\n")
            f.write(f"  Std Y Error: {stats['std_y_error']:.6f} m\n")
            f.write(f"  Std Z Error: {stats['std_z_error']:.6f} m\n")
        
        print(f"✅ Results saved to: {output_file}")
    
    def run_comparison(self, save_plots=False, save_results=False, show_plots=True):
        """Run complete trajectory comparison"""
        print("🚀 Starting trajectory comparison...")
        print(f"Dataset path: {self.dataset_path}")
        print(f"Optimized pose file: {self.optimized_pose_file}")
        
        # Align trajectories
        start_time, end_time = self.align_trajectories()
        
        # Calculate errors and visualize
        stats = self.visualize_trajectories(save_plots=save_plots, show_plots=show_plots)
        
        # Save results
        if save_results:
            self.save_comparison_results(stats)
        
        print("✅ Trajectory comparison completed!")
        return stats

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('experiment_path', nargs='?', default=None,
                       help='Path to an experiment results directory.\nIf not provided, the latest directory in "logs/" is used.')
    parser.add_argument('--save', action='store_true',
                       help='Save plots and result summary to files in the experiment directory.')
    parser.add_argument('--no-display', action='store_true',
                       help='Do not display plots in an interactive window.')
    
    args = parser.parse_args()
    
    experiment_path = args.experiment_path

    # If no path is provided, find the latest log directory
    if not experiment_path:
        log_root = 'logs'
        if os.path.isdir(log_root):
            all_log_dirs = [os.path.join(log_root, d) for d in os.listdir(log_root) if os.path.isdir(os.path.join(log_root, d))]
            if all_log_dirs:
                experiment_path = max(all_log_dirs, key=os.path.getmtime)
                print(f"ℹ️ No experiment path provided, using latest: {experiment_path}")
            else:
                print(f"❌ Error: 'logs' directory is empty or does not exist.")
                sys.exit(1)
        else:
            print(f"❌ Error: 'logs' directory not found.")
            sys.exit(1)

    if not os.path.isdir(experiment_path):
        print(f"❌ Error: Provided path is not a directory: {experiment_path}")
        sys.exit(1)

    # Find config file inside experiment_path
    config_files = [f for f in os.listdir(experiment_path) if f.endswith(('.yaml', '.yml'))]
    if not config_files:
        print(f"❌ Error: No .yaml config file found in {experiment_path}")
        sys.exit(1)
    if len(config_files) > 1:
        print(f"⚠️ Warning: Multiple .yaml files found in {experiment_path}. Using '{config_files[0]}'.")
    
    config_file = os.path.join(experiment_path, config_files[0])
    optimized_pose_file = os.path.join(experiment_path, 'trajectory_pose.txt')
    output_dir = experiment_path

    if not os.path.exists(optimized_pose_file):
        print(f"❌ Error: Pose file not found: {optimized_pose_file}")
        sys.exit(1)

    # Read dataset_path from the config file
    try:
        with open(config_file, 'r') as f:
            content = f.read()
            # Remove problematic YAML directive if it exists
            if content.startswith('%YAML:1.0'):
                content = content.splitlines()[1:]
                content = "\n".join(content)
            
            # Use a loader that ignores unknown tags like !!opencv-matrix
            class IgnoreUnknownTagsLoader(yaml.SafeLoader):
                def __init__(self, *args, **kwargs):
                    super(IgnoreUnknownTagsLoader, self).__init__(*args, **kwargs)
                    self.add_constructor(None, self.ignore_unknown)
                
                def ignore_unknown(self, loader, node):
                    return None

            config_data = yaml.load(content, Loader=IgnoreUnknownTagsLoader)
            
        dataset_path = config_data.get('dataset_path')
        if not dataset_path:
            print(f"❌ Error: 'dataset_path' not found in {config_file}")
            sys.exit(1)
        print(f"ℹ️ Found dataset_path in config: {dataset_path}")
    except yaml.YAMLError as e:
        print(f"❌ Error parsing YAML config file {config_file}: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error reading config file {config_file}: {e}")
        sys.exit(1)

    # Create comparator and run comparison
    comparator = TrajectoryComparator(dataset_path, optimized_pose_file, output_dir)
    stats = comparator.run_comparison(
        save_plots=args.save,
        save_results=args.save,
        show_plots=not args.no_display
    )

    # Print summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Dataset: {os.path.basename(dataset_path)}")
    print(f"Experiment: {os.path.basename(experiment_path)}")
    print(f"Mean Position Error: {stats['mean_position_error']:.4f} m")
    print(f"RMSE Position: {stats['rmse_position']:.4f} m")
    print(f"Max Position Error: {stats['max_position_error']:.4f} m")
    print("="*60)


if __name__ == "__main__":
    main() 