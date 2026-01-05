#!/usr/bin/env python3
"""
System Verification Script for Car Deployment
Run this BEFORE deploying to car to ensure all models work correctly
"""

from realsense_segmenter import RealSenseSegmenter
import sys

def main():
    print("\n" + "="*70)
    print("  CAR DEPLOYMENT VERIFICATION - CRITICAL SAFETY CHECK")
    print("="*70 + "\n")
    
    print("Initializing segmentation system...")
    segmenter = RealSenseSegmenter(device='cpu')
    
    # Verify models
    is_ready = segmenter.verify_ready_for_car()
    
    if is_ready:
        print("\n✓✓✓ SUCCESS: System is ready for car deployment ✓✓✓")
        print("\nYou can now:")
        print("  - Deploy to car safely")
        print("  - Run: python stream.py")
        print("  - Run: python cpu_example.py --camera")
        print("\nLane/road detection: WORKING ✓")
        print("Object detection: WORKING ✓")
        sys.exit(0)
    else:
        print("\n✗✗✗ FAILURE: System NOT ready for car ✗✗✗")
        print("\nProblems detected:")
        
        if segmenter.engine.yolopv2_model is None:
            print("  ✗ YOLOPv2 (lane/road detection) FAILED")
            print("    → This is CRITICAL - car won't see lanes/road!")
            print("    → Check that data/weights/yolopv2.pt exists")
            print("    → Model may be GPU-only (need CPU-compatible version)")
        
        if segmenter.engine.yolov8_model is None:
            print("  ✗ YOLOv8 (object detection) FAILED")
            print("    → Check that yolov8m-seg.pt exists")
        
        print("\n*** DO NOT DEPLOY TO CAR ***")
        sys.exit(1)

if __name__ == "__main__":
    main()
