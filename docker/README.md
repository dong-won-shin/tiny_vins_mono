# Docker Usage Guide for Tiny VINS Mono

This directory contains Docker configuration files for running Tiny VINS Mono in a containerized environment.

## Files Overview

- `Dockerfile`: Main Docker image definition
- `build.sh`: Script to build the Docker image
- `run.sh`: Script to run the container with X11 forwarding

## Quick Start

### 1. Build the Docker Image

```bash
cd docker
./build.sh
```

### 2. Run the Container

#### Using the run script (Recommended for GUI)
```bash
./run.sh
```

## Running the Application

Once inside the container:

```bash
# Navigate to build directory (default working directory)
cd /workspace/tiny_vins_mono

# Run with config file
./build/tiny_vins_mono ../config/config.yaml
```

## Data and Configuration

### Volume Mounts
The container automatically mounts:
- `../data` → `/workspace/tiny_vins_mono/data` (for datasets)
- `../logs` → `/workspace/tiny_vins_mono/logs` (for output logs)
- `../config` → `/workspace/tiny_vins_mono/config` (for configuration files)

### Preparing Dataset
Place your EuRoC dataset in the `data` directory:
```bash
mkdir -p data/
# Copy your dataset to data/
```

## GUI Support

The Docker configuration includes X11 forwarding for the Pangolin visualization:

### Prerequisites
- X11 server running on host
- `xauth` package installed on host

### Troubleshooting GUI Issues

If GUI doesn't work:
1. Ensure X11 forwarding is enabled: `xhost +local:docker`
2. Check DISPLAY variable: `echo $DISPLAY`
3. Verify X11 socket exists: `ls -la /tmp/.X11-unix/`

## System Requirements

- Docker installed
- X11 server (for GUI applications)
- At least 4GB RAM recommended
