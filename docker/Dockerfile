FROM ubuntu:22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set timezone to avoid tzdata interactive prompt
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update package list and install basic development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    unzip \
    python3 \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Install core dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libceres-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install graphics and GUI dependencies for Pangolin
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev \
    libglew-dev \
    libepoxy-dev \
    libglfw3-dev \
    libegl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install additional X11 dependencies for GUI applications
RUN apt-get update && apt-get install -y \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Pangolin from source
WORKDIR /tmp
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout v0.8 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_CXX_FLAGS="-Wno-missing-braces" .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/Pangolin

# Set up library path
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Create working directory for the project
WORKDIR /workspace/tiny_vins_mono

# Copy project files (excluding .git and other sensitive files via .dockerignore)
COPY . .

# install Python dependencies
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r /workspace/tiny_vins_mono/requirements.txt

RUN rm -rf build && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc)

# Set environment variables for GUI applications
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Create logs directory
RUN mkdir -p logs

# Set the default working directory to build
WORKDIR /workspace/tiny_vins_mono/build

# Default command to show help or usage
CMD ["echo", "Tiny VINS Mono Docker container is ready! Run: ./tiny_vins_mono ../config/config.yaml"]