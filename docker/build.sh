#!/bin/bash

# Build script for Tiny VINS Mono Docker image

set -e

echo "Building Tiny VINS Mono Docker image..."

# Navigate to the project root directory
cd "$(dirname "$0")/.."

# Build the Docker image
docker build -t tiny-vins-mono:latest -f docker/Dockerfile .

echo "Docker image built successfully!"
echo "Image name: tiny-vins-mono:latest"
echo ""
echo "To run the container:"
echo "  cd docker && docker-compose up -d"
echo "  docker exec -it tiny-vins-mono bash"
echo ""
echo "Or run directly:"
echo "  docker run -it --rm tiny-vins-mono:latest"