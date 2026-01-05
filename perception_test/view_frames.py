#!/usr/bin/env python3
"""
View the saved segmented frames from the camera
Run after running: python live_stream.py
"""

import subprocess
import os
import glob

def main():
    # Find segmented frames
    frames = sorted(glob.glob('/tmp/segmented_*.jpg'))
    
    if not frames:
        print("No segmented frames found!")
        print("Run this first: python live_stream.py")
        return
    
    print(f"Found {len(frames)} segmented frames")
    print()
    print("Frame list:")
    for i, frame in enumerate(frames[-10:]):  # Show last 10
        size = os.path.getsize(frame) / 1024
        print(f"  {i+1}. {os.path.basename(frame)} ({size:.0f} KB)")
    
    print()
    print("Viewing options:")
    print(f"  1. View with image viewer: eog /tmp/segmented_*.jpg")
    print(f"  2. Convert to video: ffmpeg -i /tmp/segmented_%04d.jpg -c:v libx264 output.mp4")
    print(f"  3. View specific frame: eog /tmp/segmented_0010.jpg")
    print()
    
    # Try to open with available viewer
    viewers = ['eog', 'feh', 'xdg-open']
    for viewer in viewers:
        if subprocess.run(['which', viewer], capture_output=True).returncode == 0:
            print(f"Opening with {viewer}...")
            # Pass actual file list instead of glob pattern (glob must be expanded)
            if viewer == 'xdg-open':
                # xdg-open only takes one file at a time, open the latest
                subprocess.Popen([viewer, frames[-1]])
            else:
                # eog and feh can take multiple files
                subprocess.Popen([viewer] + frames)
            break
    else:
        print("No image viewer found. Use:")
        print(f"  ls -lh /tmp/segmented_*.jpg")

if __name__ == "__main__":
    main()
