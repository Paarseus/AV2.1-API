#!/usr/bin/env python3
"""
SSL Preprocessing for Road Surface Classification

Converts CSV data from vibration_logger.py to TFC-compatible .pt format.

Usage:
    python ssl_preprocessing.py --input ../data/unlabeled/ --output ../data/processed/
    python ssl_preprocessing.py --input ../data/unlabeled/drive1.csv --output ../data/processed/
"""

import os
import argparse
import numpy as np
import pandas as pd
import torch
from pathlib import Path
from typing import List, Tuple, Optional

# Configuration
SAMPLE_RATE = 100  # Hz (from vibration_logger.py)
WINDOW_SIZE = 100  # 1 second at 100 Hz
WINDOW_OVERLAP = 0.5  # 50% overlap
CHANNELS = ['az']  # Vertical acceleration is primary signal for road roughness
# Full channels: ['ax', 'ay', 'az', 'gx', 'gy', 'gz']

LABEL_MAP = {
    'asphalt': 0,
    'sidewalk': 1,
    'grass': 2,
    'bump': 3,
    'unknown': -1,  # For unlabeled data
}


def load_csv(filepath: str) -> pd.DataFrame:
    """Load vibration logger CSV file."""
    df = pd.read_csv(filepath)

    # Verify required columns exist
    required = ['timestamp', 'az']
    for col in required:
        if col not in df.columns:
            raise ValueError(f"Missing required column: {col}")

    print(f"  Loaded {len(df):,} samples from {Path(filepath).name}")
    return df


def segment_windows(df: pd.DataFrame,
                    window_size: int = WINDOW_SIZE,
                    overlap: float = WINDOW_OVERLAP,
                    channels: List[str] = CHANNELS) -> Tuple[np.ndarray, np.ndarray]:
    """
    Segment dataframe into fixed-size windows.

    Returns:
        samples: (N, C, T) array of windows
        labels: (N,) array of integer labels
    """
    step = int(window_size * (1 - overlap))
    n_samples = len(df)
    n_windows = (n_samples - window_size) // step + 1

    if n_windows <= 0:
        return np.array([]), np.array([])

    # Extract channel data
    channel_data = df[channels].values  # (n_samples, n_channels)

    # Handle labels if present
    has_labels = 'label' in df.columns

    samples = []
    labels = []

    for i in range(n_windows):
        start = i * step
        end = start + window_size

        # Extract window (T, C) -> (C, T)
        window = channel_data[start:end].T  # (n_channels, window_size)
        samples.append(window)

        # Get label (majority vote in window)
        if has_labels:
            window_labels = df['label'].iloc[start:end].values
            unique, counts = np.unique(window_labels, return_counts=True)
            majority_label = unique[np.argmax(counts)]
            labels.append(LABEL_MAP.get(majority_label, -1))
        else:
            labels.append(-1)  # Unknown for unlabeled data

    return np.array(samples, dtype=np.float32), np.array(labels, dtype=np.int64)


def normalize_samples(samples: np.ndarray,
                      per_channel: bool = True) -> Tuple[np.ndarray, dict]:
    """
    Normalize samples to zero mean and unit variance.

    Returns:
        normalized: Normalized samples
        stats: Dict with mean/std for each channel (for inference)
    """
    if per_channel:
        # Normalize each channel independently
        mean = samples.mean(axis=(0, 2), keepdims=True)  # (1, C, 1)
        std = samples.std(axis=(0, 2), keepdims=True)
        std[std == 0] = 1.0  # Avoid division by zero

        normalized = (samples - mean) / std
        stats = {
            'mean': mean.squeeze(),
            'std': std.squeeze(),
        }
    else:
        # Global normalization
        mean = samples.mean()
        std = samples.std()
        if std == 0:
            std = 1.0
        normalized = (samples - mean) / std
        stats = {'mean': float(mean), 'std': float(std)}

    return normalized, stats


def process_directory(input_dir: str,
                      output_dir: str,
                      channels: List[str] = CHANNELS,
                      split_ratio: Tuple[float, float, float] = (0.7, 0.15, 0.15)) -> dict:
    """
    Process all CSV files in directory and create train/val/test splits.

    Args:
        input_dir: Directory containing CSV files
        output_dir: Output directory for .pt files
        channels: List of channel names to use
        split_ratio: (train, val, test) ratios

    Returns:
        Dictionary with processing statistics
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # Find all CSV files
    csv_files = list(input_path.glob('*.csv'))
    if not csv_files:
        raise ValueError(f"No CSV files found in {input_dir}")

    print(f"Found {len(csv_files)} CSV files")

    # Process all files
    all_samples = []
    all_labels = []

    for csv_file in csv_files:
        df = load_csv(str(csv_file))
        samples, labels = segment_windows(df, channels=channels)

        if len(samples) > 0:
            all_samples.append(samples)
            all_labels.append(labels)

    if not all_samples:
        raise ValueError("No valid windows extracted")

    # Concatenate all data
    samples = np.concatenate(all_samples, axis=0)
    labels = np.concatenate(all_labels, axis=0)

    print(f"\nTotal windows: {len(samples):,}")
    print(f"Shape: {samples.shape} (N, C, T)")

    # Print label distribution
    unique, counts = np.unique(labels, return_counts=True)
    print("\nLabel distribution:")
    reverse_map = {v: k for k, v in LABEL_MAP.items()}
    for lbl, cnt in zip(unique, counts):
        name = reverse_map.get(lbl, 'unknown')
        print(f"  {name}: {cnt:,} ({100*cnt/len(labels):.1f}%)")

    # Normalize
    samples, norm_stats = normalize_samples(samples)
    print(f"\nNormalization stats: {norm_stats}")

    # Shuffle
    perm = np.random.permutation(len(samples))
    samples = samples[perm]
    labels = labels[perm]

    # Split
    n_train = int(len(samples) * split_ratio[0])
    n_val = int(len(samples) * split_ratio[1])

    train_samples = samples[:n_train]
    train_labels = labels[:n_train]
    val_samples = samples[n_train:n_train + n_val]
    val_labels = labels[n_train:n_train + n_val]
    test_samples = samples[n_train + n_val:]
    test_labels = labels[n_train + n_val:]

    # Save as TFC-compatible format
    def save_split(samples, labels, name):
        data = {
            'samples': torch.from_numpy(samples),
            'labels': torch.from_numpy(labels),
        }
        path = output_path / f'{name}.pt'
        torch.save(data, path)
        print(f"Saved {name}.pt: {len(samples):,} samples")

    save_split(train_samples, train_labels, 'train')
    save_split(val_samples, val_labels, 'val')
    save_split(test_samples, test_labels, 'test')

    # Save normalization stats
    stats_path = output_path / 'norm_stats.pt'
    torch.save(norm_stats, stats_path)

    # Save metadata
    metadata = {
        'channels': channels,
        'sample_rate': SAMPLE_RATE,
        'window_size': WINDOW_SIZE,
        'window_overlap': WINDOW_OVERLAP,
        'label_map': LABEL_MAP,
        'norm_stats': norm_stats,
        'n_train': len(train_samples),
        'n_val': len(val_samples),
        'n_test': len(test_samples),
    }
    torch.save(metadata, output_path / 'metadata.pt')

    return metadata


def process_single_file(input_file: str,
                        output_dir: str,
                        channels: List[str] = CHANNELS,
                        mode: str = 'unlabeled') -> dict:
    """
    Process a single CSV file (for unlabeled pretraining data).

    Args:
        input_file: Path to CSV file
        output_dir: Output directory
        channels: Channel names to use
        mode: 'unlabeled' or 'labeled'
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    df = load_csv(input_file)
    samples, labels = segment_windows(df, channels=channels)

    if len(samples) == 0:
        raise ValueError(f"No valid windows from {input_file}")

    print(f"Extracted {len(samples):,} windows")
    print(f"Shape: {samples.shape}")

    # Normalize
    samples, norm_stats = normalize_samples(samples)

    # Save
    data = {
        'samples': torch.from_numpy(samples),
        'labels': torch.from_numpy(labels),
    }

    filename = Path(input_file).stem
    torch.save(data, output_path / f'{filename}.pt')
    torch.save(norm_stats, output_path / f'{filename}_norm_stats.pt')

    print(f"Saved to {output_path / filename}.pt")

    return {'n_samples': len(samples), 'shape': samples.shape}


def main():
    parser = argparse.ArgumentParser(description='Preprocess vibration data for SSL')
    parser.add_argument('--input', '-i', required=True,
                        help='Input CSV file or directory')
    parser.add_argument('--output', '-o', default='../data/processed/',
                        help='Output directory')
    parser.add_argument('--channels', '-c', nargs='+', default=CHANNELS,
                        help=f'Channels to use (default: {CHANNELS})')
    parser.add_argument('--mode', '-m', choices=['single', 'directory'], default=None,
                        help='Processing mode (auto-detected if not specified)')
    args = parser.parse_args()

    input_path = Path(args.input)

    # Auto-detect mode
    if args.mode is None:
        mode = 'single' if input_path.is_file() else 'directory'
    else:
        mode = args.mode

    print(f"SSL Preprocessing")
    print(f"  Input: {args.input}")
    print(f"  Output: {args.output}")
    print(f"  Channels: {args.channels}")
    print(f"  Mode: {mode}")
    print()

    if mode == 'single':
        process_single_file(args.input, args.output, args.channels)
    else:
        process_directory(args.input, args.output, args.channels)

    print("\nPreprocessing complete!")


if __name__ == '__main__':
    main()
