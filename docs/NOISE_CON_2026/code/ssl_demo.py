#!/usr/bin/env python3
"""
Real-Time Road Surface Classification Demo

Uses trained SSL model to classify road surfaces from live IMU data.

Usage:
    python ssl_demo.py --model ../models/finetuned/finetuned.pt

Keys:
    Q - Quit
    R - Reset buffer
"""

import os
import sys
import time
import argparse
from pathlib import Path
from collections import deque

import numpy as np
import torch
import torch.fft as fft

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Add TFC to path
TFC_PATH = Path(__file__).parent / 'TFC-pretraining' / 'code' / 'TFC'
sys.path.insert(0, str(TFC_PATH))

from model import TFC, target_classifier

# Try to import xsens - will fail gracefully if not available
try:
    from sensors.xsens_receiver import XsensReceiver
    XSENS_AVAILABLE = True
except ImportError:
    XSENS_AVAILABLE = False
    print("Warning: XsensReceiver not available, using simulated data")

# Label names
LABEL_NAMES = ['Asphalt', 'Sidewalk', 'Grass', 'Bump']

# Configuration
SAMPLE_RATE = 100  # Hz
WINDOW_SIZE = 100  # 1 second
INFERENCE_INTERVAL = 0.5  # Run inference every 0.5 seconds


def get_config():
    """Load configuration."""
    config_path = TFC_PATH.parent / 'config_files'
    sys.path.insert(0, str(config_path))
    from RoadVibration_Configs import Config
    return Config()


class RealTimeClassifier:
    """Real-time road surface classifier."""

    def __init__(self, model_path: str, device: str = 'cuda'):
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")

        # Load config
        self.config = get_config()

        # Initialize model
        self.model = TFC(self.config).to(self.device)
        self.classifier = target_classifier(self.config).to(self.device)

        # Load checkpoint
        checkpoint = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.classifier.load_state_dict(checkpoint['classifier_state_dict'])

        self.model.eval()
        self.classifier.eval()

        # Load normalization stats if available
        norm_stats_path = Path(model_path).parent.parent / 'data' / 'processed' / 'norm_stats.pt'
        if norm_stats_path.exists():
            self.norm_stats = torch.load(norm_stats_path)
            print(f"Loaded normalization stats from {norm_stats_path}")
        else:
            self.norm_stats = {'mean': 0.0, 'std': 1.0}
            print("Using default normalization (mean=0, std=1)")

        # Buffer for incoming data
        self.buffer = deque(maxlen=WINDOW_SIZE)
        self.last_inference_time = 0

        print(f"Model loaded from {model_path}")

    def add_sample(self, az: float):
        """Add a single accelerometer sample to buffer."""
        self.buffer.append(az)

    def can_classify(self) -> bool:
        """Check if we have enough data for classification."""
        return len(self.buffer) >= WINDOW_SIZE

    def classify(self) -> tuple:
        """
        Classify current buffer contents.

        Returns:
            (class_name, confidence, class_idx)
        """
        if not self.can_classify():
            return None, 0.0, -1

        # Convert buffer to tensor
        data = np.array(list(self.buffer), dtype=np.float32)

        # Normalize
        mean = self.norm_stats.get('mean', 0.0)
        std = self.norm_stats.get('std', 1.0)
        if isinstance(mean, np.ndarray):
            mean = mean[0]
        if isinstance(std, np.ndarray):
            std = std[0]
        data = (data - mean) / (std if std > 0 else 1.0)

        # Reshape: (1, 1, T)
        data = torch.from_numpy(data).float().unsqueeze(0).unsqueeze(0)
        data = data.to(self.device)

        # Compute frequency domain
        data_f = fft.fft(data).abs()

        # Forward pass
        with torch.no_grad():
            h_t, z_t, h_f, z_f = self.model(data, data_f)
            fea_concat = torch.cat((z_t, z_f), dim=1)
            logits = self.classifier(fea_concat)
            probs = torch.softmax(logits, dim=1)

            class_idx = probs.argmax(dim=1).item()
            confidence = probs[0, class_idx].item()

        class_name = LABEL_NAMES[class_idx] if class_idx < len(LABEL_NAMES) else f'Class {class_idx}'

        return class_name, confidence, class_idx

    def reset(self):
        """Clear the buffer."""
        self.buffer.clear()


class SimulatedSensor:
    """Simulated sensor for testing without hardware."""

    def __init__(self):
        self.time_offset = time.time()
        self.current_surface = 0
        self.surface_duration = 5.0  # Change surface every 5 seconds

    def get_sample(self) -> float:
        """Generate a simulated accelerometer sample."""
        t = time.time() - self.time_offset

        # Change surface periodically
        surface_idx = int(t / self.surface_duration) % 4

        # Generate surface-specific vibration pattern
        base_noise = np.random.randn() * 0.5

        if surface_idx == 0:  # Asphalt - smooth
            az = base_noise * 0.3
        elif surface_idx == 1:  # Sidewalk - moderate roughness
            az = base_noise + 0.2 * np.sin(2 * np.pi * 10 * t)
        elif surface_idx == 2:  # Grass - low frequency bumps
            az = base_noise * 1.5 + 0.3 * np.sin(2 * np.pi * 3 * t)
        else:  # Bump - periodic spikes
            phase = (t % 1.0)
            az = base_noise + (2.0 if phase < 0.1 else 0.0)

        return az


def run_live_demo(classifier, use_simulated: bool = False, duration: float = 0):
    """Run live classification demo.

    Args:
        classifier: RealTimeClassifier instance
        use_simulated: Use simulated sensor data
        duration: If > 0, run for this many seconds then exit (for testing)
    """
    print("\n" + "="*50)
    print("Real-Time Road Surface Classification Demo")
    print("="*50)

    if use_simulated:
        print("\nUsing SIMULATED sensor data")
        print("Surface changes every 5 seconds: Asphalt -> Sidewalk -> Grass -> Bump")
        sensor = SimulatedSensor()
    else:
        print("\nConnecting to Xsens IMU...")
        sensor = XsensReceiver(update_rate=100)
        if not sensor.start(wait_for_rtk_fixed=False):
            print("ERROR: Failed to connect to Xsens")
            return
        print("Xsens connected!")

    # Check if running interactively
    interactive = sys.stdin.isatty()

    if interactive:
        print("\nPress 'Q' to quit, 'R' to reset buffer")
        print("-"*50)

        # Set up terminal for non-blocking input
        import select
        import tty
        import termios

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    else:
        print(f"\nRunning in non-interactive mode for {duration if duration > 0 else 'unlimited'} seconds")
        print("-"*50)
        old_settings = None

    sample_interval = 1.0 / SAMPLE_RATE
    last_sample_time = time.time()
    last_display_time = time.time()
    start_time = time.time()

    try:
        while True:
            now = time.time()

            # Check duration limit for non-interactive mode
            if duration > 0 and (now - start_time) >= duration:
                print("\n\nDuration limit reached.")
                break

            # Check for keypress (only in interactive mode)
            if interactive:
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'q':
                        break
                    elif key == 'r':
                        classifier.reset()
                        print("\n[RESET] Buffer cleared")

            # Collect sample at target rate
            if now - last_sample_time >= sample_interval:
                if use_simulated:
                    az = sensor.get_sample()
                else:
                    data = sensor.get_all_data()
                    acc = data.get('acceleration', {})
                    az = acc.get('up', 0.0) or 0.0

                classifier.add_sample(az)
                last_sample_time = now

            # Run classification and update display
            if now - last_display_time >= INFERENCE_INTERVAL:
                if classifier.can_classify():
                    class_name, confidence, _ = classifier.classify()

                    # Color-coded output (skip colors in non-interactive)
                    if interactive:
                        if confidence > 0.8:
                            conf_str = f"\033[92m{confidence*100:.1f}%\033[0m"  # Green
                        elif confidence > 0.5:
                            conf_str = f"\033[93m{confidence*100:.1f}%\033[0m"  # Yellow
                        else:
                            conf_str = f"\033[91m{confidence*100:.1f}%\033[0m"  # Red
                        print(f"\r  Surface: {class_name:10s} | Confidence: {conf_str:15s} | "
                              f"Buffer: {len(classifier.buffer):3d}/{WINDOW_SIZE}  ",
                              end='', flush=True)
                    else:
                        print(f"  Surface: {class_name:10s} | Confidence: {confidence*100:.1f}% | "
                              f"Buffer: {len(classifier.buffer):3d}/{WINDOW_SIZE}")
                else:
                    if interactive:
                        print(f"\r  Collecting data... Buffer: {len(classifier.buffer):3d}/{WINDOW_SIZE}  ",
                              end='', flush=True)
                    else:
                        print(f"  Collecting data... Buffer: {len(classifier.buffer):3d}/{WINDOW_SIZE}")

                last_display_time = now

            # Small sleep to prevent busy waiting
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        if interactive and old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        if not use_simulated and hasattr(sensor, 'stop'):
            sensor.stop()

        print("\n\nDemo ended.")


def main():
    parser = argparse.ArgumentParser(description='Real-time road surface classification')
    parser.add_argument('--model', required=True, help='Path to trained model')
    parser.add_argument('--device', default='cuda', help='Device (cuda/cpu)')
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated sensor data (for testing without hardware)')
    parser.add_argument('--duration', type=float, default=0,
                        help='Run for N seconds then exit (0=unlimited, for non-interactive testing)')
    args = parser.parse_args()

    # Force simulation if xsens not available
    use_simulated = args.simulate or not XSENS_AVAILABLE

    # Initialize classifier
    classifier = RealTimeClassifier(args.model, args.device)

    # Run demo
    run_live_demo(classifier, use_simulated, args.duration)


if __name__ == '__main__':
    main()
