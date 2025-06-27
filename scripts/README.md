# Scripts

This directory contains utility scripts for analyzing and visualizing VINS results.

## compare_trajectories.py

A comprehensive trajectory comparison tool that evaluates the performance of VINS by comparing the optimized pose trajectory with ground truth data from EuRoC datasets.

### Features

- **Trajectory Alignment**: Automatically aligns trajectories using Z-axis rotation and translation
- **Error Metrics**: Calculates comprehensive error statistics including RMSE, mean, median, and maximum errors
- **3D Visualization**: Provides 3D trajectory plots and 2D projections (XY, XZ planes)
- **Flexible Input**: Supports automatic detection of latest results or specific experiment directories
- **Export Options**: Can save comparison plots and detailed results to files

### Requirements

The script requires the following Python packages:
- numpy
- pandas
- matplotlib
- scipy
- pyyaml

### Usage

#### Basic Usage

```bash
# Compare with the latest experiment results (automatically detects latest directory in logs/)
python scripts/compare_trajectories.py

# Compare with a specific experiment directory
python scripts/compare_trajectories.py logs/20250623_120815
```

#### Advanced Usage

```bash
# Save plots and results without displaying interactive plots
python scripts/compare_trajectories.py logs/20250623_120815 --save --no-display

# Save results and show interactive plots
python scripts/compare_trajectories.py --save

# Run without interactive display (useful for automated testing)
python scripts/compare_trajectories.py --no-display
```

### Command Line Arguments

#### Positional Arguments

- `experiment_path` (optional): Path to the experiment results directory (e.g., `logs/TIMESTAMP`)
  - If not provided, automatically uses the latest directory in `logs/`

#### Optional Flags

- `--save`: Save comparison plots and results to files in the experiment directory
- `--no-display`: Do not show plots in an interactive window (useful for headless environments)

### Input Requirements

The script expects the following file structure in the experiment directory:

```
logs/TIMESTAMP/
├── config.yaml           # Configuration file containing dataset_path
└── trajectory_pose.txt   # Optimized trajectory in TUM format
```

#### Trajectory Format

The optimized trajectory file should be in TUM format:
```
timestamp tx ty tz qx qy qz qw
```

Where:
- `timestamp`: Time in seconds
- `tx`, `ty`, `tz`: Translation coordinates
- `qx`, `qy`, `qz`, `qw`: Quaternion rotation (x, y, z, w)

### Output

#### Console Output

The script provides detailed progress information and error statistics:

```
📊 Loading trajectory data...
✅ Ground truth data loaded: 27025 poses
✅ Optimized pose data loaded: 1205 poses
🔄 Aligning trajectories...
Ground truth time range: 1403636579.763 - 1403636695.763
Optimized time range: 1403636582.175 - 1403636693.975
Overlapping time range: 1403636582.175 - 1403636693.975
✅ Aligned trajectories: 1205 poses
📈 Calculating error metrics...
📊 Error Statistics:
  Mean Position Error: 0.1234 m
  Std Position Error: 0.0567 m
  Max Position Error: 0.4321 m
  RMSE Position: 0.1356 m
  Mean X Error: 0.0891 m
  Mean Y Error: 0.0743 m
  Mean Z Error: 0.0512 m
```

#### Generated Files (with --save flag)

- `trajectory_comparison_[dataset_name].png`: Visualization plots
- `trajectory_comparison_results_[dataset_name].txt`: Detailed error statistics

### Error Metrics

The script calculates the following error metrics:

- **Mean Position Error**: Average 3D Euclidean distance error
- **RMSE Position**: Root Mean Square Error for position
- **Standard Deviation**: Variability of position errors
- **Maximum Error**: Worst-case position error
- **Median Error**: 50th percentile error
- **Per-axis Errors**: Individual X, Y, Z axis error statistics

### Visualization

The script generates three types of plots:

1. **3D Trajectory Plot**: Complete 3D visualization of both trajectories
2. **XY Projection**: Top-down view showing horizontal movement
3. **XZ Projection**: Side view showing vertical movement

### Examples

#### Example 1: Quick Comparison
```bash
# Compare latest results with interactive visualization
python scripts/compare_trajectories.py
```

#### Example 2: Batch Processing
```bash
# Process specific experiment and save results without display
python scripts/compare_trajectories.py logs/20250623_120815 --save --no-display
```

#### Example 3: Detailed Analysis
```bash
# Interactive analysis with saved results
python scripts/compare_trajectories.py logs/experiment_v1 --save
```

### Troubleshooting

#### Common Issues

1. **No dataset_path found**: Ensure the config.yaml file contains a valid `dataset_path` field
2. **Ground truth file not found**: Verify the dataset path points to a valid EuRoC dataset with ground truth data
3. **Trajectory file missing**: Check that `trajectory_pose.txt` exists in the experiment directory
4. **No overlapping time range**: Ensure the optimized trajectory covers the same time period as the ground truth

#### Error Messages

- `❌ Error: 'logs' directory not found`: Run the script from the project root directory
- `❌ Error: No .yaml config file found`: Ensure the experiment directory contains a configuration file
- `❌ Error: Pose file not found`: Verify that trajectory estimation completed successfully

### Performance Notes

- The script can handle trajectories with thousands of poses efficiently
- Memory usage scales with trajectory length
- Visualization generation may take longer for very dense trajectories
- Use `--no-display` flag for faster processing in automated environments