#!/bin/bash
# Quick verification script for car deployment

echo "=========================================="
echo "  CAR DEPLOYMENT SAFETY CHECK"
echo "=========================================="
echo ""

cd "$(dirname "$0")"

# Check if dependencies are installed
echo "Checking dependencies..."
python3 -c "import torch, cv2, numpy, ultralytics" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing dependencies. Installing..."
    pip install -r requirements.txt
    echo ""
fi

# Check if models exist
echo "Checking model files..."
if [ ! -f "data/weights/yolopv2.pt" ]; then
    echo "❌ YOLOPv2 model missing: data/weights/yolopv2.pt"
    echo "   Download from: https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
    exit 1
fi

if [ ! -f "yolov8m-seg.pt" ] && [ ! -f "yolov8n-seg.pt" ]; then
    echo "⚠️  YOLOv8 model not found. It will be downloaded automatically."
fi

echo ""
echo "Running system verification..."
echo ""

# Run verification
python3 verify_system.py

exit $?
