#!/bin/bash
# Quick LiDAR Setup Script for Vision Pi

echo "=========================================="
echo "LSLIDAR N10 Setup for Vision Pi"
echo "=========================================="
echo ""

# 1. Configure network for LiDAR
echo "Step 1: Configuring network..."
echo "Your network is already configured:"
echo "  Vision Pi: 10.1.1.21 (wlan0 or eth0)"
echo "  Main Pi:   10.1.1.20"
echo "  LiDAR:     10.1.1.200 (needs to be set on device)"
echo ""
echo "⚠️  IMPORTANT: Configure LiDAR IP to 10.1.1.200"
echo "   Use LSLIDAR configuration tool or web interface"
echo "   Default IP is usually 192.168.1.200"
echo ""

read -p "Have you configured LiDAR IP to 10.1.1.200? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Please configure LiDAR first, then run this script again."
    exit 1
fi

# 2. Test LiDAR connectivity
echo ""
echo "Step 2: Testing LiDAR connectivity..."
echo "Pinging LiDAR at 10.1.1.200..."

if ping -c 3 10.1.1.200 > /dev/null 2>&1; then
    echo "✓ LiDAR is reachable!"
else
    echo "✗ LiDAR not responding. Check:"
    echo "  - Ethernet cable connected to Vision Pi?"
    echo "  - LiDAR powered on?"
    echo "  - LiDAR IP configured to 10.1.1.200?"
    echo "  - Vision Pi has IP 10.1.1.21?"
    exit 1
fi

# 3. Pull latest code
echo ""
echo "Step 3: Pulling latest code from GitHub..."
cd ~/rob_box_project
git pull

# 4. Build LiDAR container
echo ""
echo "Step 4: Building LSLIDAR container..."
cd docker/vision
docker-compose build lslidar

if [ $? -ne 0 ]; then
    echo "✗ Build failed!"
    exit 1
fi

echo "✓ Build successful!"

# 5. Start LiDAR
echo ""
echo "Step 5: Starting LSLIDAR container..."
docker-compose up -d lslidar

echo "Waiting for container to start..."
sleep 10

# 6. Verify
echo ""
echo "Step 6: Verifying LiDAR is publishing..."
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
