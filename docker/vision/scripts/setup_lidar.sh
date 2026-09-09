#!/bin/bash
# Quick LiDAR Setup Script for Vision Pi

echo "=========================================="
echo "LSLIDAR N10 Setup for Vision Pi"
echo "=========================================="
echo ""

# 1. Configure network for LiDAR
echo "Step 1: Checking LiDAR connection..."
echo "LSLIDAR N10 should be connected to Vision Pi via USB/Serial port"
echo ""

# Check if USB device exists
if [ -e /dev/ttyUSB0 ]; then
    echo "✓ Found /dev/ttyUSB0 device"
elif [ -e /dev/ttyUSB1 ]; then
    echo "✓ Found /dev/ttyUSB1 device"
    echo "⚠️  WARNING: LiDAR might be on /dev/ttyUSB1, update docker-compose.yaml if needed"
else
    echo "✗ No /dev/ttyUSB* devices found!"
    echo "  Check:"
    echo "  - Is LiDAR connected via USB?"
    echo "  - Is LiDAR powered on?"
    echo "  - Run: ls -la /dev/ttyUSB*"
    exit 1
fi

# 2. Pull latest code
echo ""
echo "Step 2: Pulling latest code from GitHub..."
cd ~/rob_box_project
git pull

# 3. Build LiDAR container
echo ""
echo "Step 3: Building LSLIDAR container..."
cd docker/vision
docker-compose build lslidar

if [ $? -ne 0 ]; then
    echo "✗ Build failed!"
    exit 1
fi

echo "✓ Build successful!"

# 4. Start LiDAR
echo ""
echo "Step 4: Starting LSLIDAR container..."
docker-compose up -d lslidar

echo "Waiting for container to start..."
sleep 10

# 5. Verify
echo ""
echo "Step 5: Verifying LiDAR is publishing..."
docker logs lslidar --tail 20

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "LiDAR should be publishing on /scan topic."
echo ""
echo "To verify:"
echo "  docker logs lslidar -f         # Watch logs"
echo "  ros2 topic hz /scan            # Check scan rate (~10Hz)"
echo "  ros2 topic echo /scan --once   # View scan data"
echo ""
echo "If issues, see: docker/LSLIDAR_SETUP.md"
echo ""
