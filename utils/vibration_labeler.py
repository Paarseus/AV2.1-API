#!/usr/bin/env python3
"""
Vibration Data Labeler (Timestamp-Based)

Adds surface type labels to vibration CSV data based on time ranges.
You note timestamps while driving, then create a simple labels file.

Usage:
    1. Run vibration_logger.py and note when surfaces change:
       "0:00 smooth, 0:32 bump, 0:38 smooth, 0:45 rough..."

    2. Create labels file (time_labels.csv):
       start,end,label
       0,32,smooth
       32,38,bump
       38,45,smooth
       45,70,rough

    3. Run labeler:
       python utils/vibration_labeler.py --data data/vibration.csv --labels data/time_labels.csv

Surface Types (4 classes):
    - asphalt:  Paved roads (good condition)
    - sidewalk: Concrete paths/sidewalks
    - grass:    Grass, dirt, unpaved surfaces
    - bump:     Speed bumps
"""

import os
import sys
import csv
import argparse
from typing import List, Optional

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_time_labels(labels_file: str) -> List[dict]:
    """Load time-based labels from CSV file."""
    labels = []

    with open(labels_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Skip comments
            if row.get('start', '').startswith('#'):
                continue
            labels.append({
                'start': float(row['start']),
                'end': float(row['end']),
                'label': row['label'].strip(),
            })

    # Sort by start time
    labels.sort(key=lambda x: x['start'])

    print(f"[LABELER] Loaded {len(labels)} time segments")
    for seg in labels:
        print(f"    {seg['start']:6.1f}s - {seg['end']:6.1f}s : {seg['label']}")

    return labels


def find_label_for_time(elapsed: float, labels: List[dict]) -> Optional[str]:
    """Find the surface label for a given elapsed time."""
    for seg in labels:
        if seg['start'] <= elapsed < seg['end']:
            return seg['label']
    return None


def label_data(data_file: str, labels_file: str, output_file: str):
    """
    Add surface type labels to vibration data based on timestamps.

    Args:
        data_file: Input CSV with vibration data
        labels_file: CSV with time-based labels
        output_file: Output CSV with labels added
    """
    # Load time labels
    labels = load_time_labels(labels_file)

    # Find time range of labels
    if not labels:
        print("[LABELER] ERROR: No labels found in file")
        return

    # Read data to find start timestamp
    with open(data_file, 'r') as f:
        reader = csv.DictReader(f)
        first_row = next(reader)
        start_timestamp = float(first_row['timestamp'])

    print(f"[LABELER] Data start timestamp: {start_timestamp:.3f}")

    # Process data
    labeled_count = 0
    unlabeled_count = 0
    total_count = 0

    with open(data_file, 'r') as f_in, open(output_file, 'w', newline='') as f_out:
        reader = csv.DictReader(f_in)

        # Add 'label' and 'elapsed' columns to output
        fieldnames = reader.fieldnames + ['elapsed', 'label']
        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            total_count += 1

            # Compute elapsed time from start
            timestamp = float(row['timestamp'])
            elapsed = timestamp - start_timestamp

            # Find label for this time
            label = find_label_for_time(elapsed, labels)

            row['elapsed'] = f"{elapsed:.3f}"

            if label:
                row['label'] = label
                labeled_count += 1
            else:
                row['label'] = 'unknown'
                unlabeled_count += 1

            writer.writerow(row)

    # Print summary
    print(f"\n[LABELER] Summary:")
    print(f"  Total samples: {total_count:,}")
    print(f"  Labeled: {labeled_count:,} ({100*labeled_count/total_count:.1f}%)")
    print(f"  Unlabeled: {unlabeled_count:,} ({100*unlabeled_count/total_count:.1f}%)")
    print(f"  Output: {output_file}")

    # Print per-class counts
    print(f"\n[LABELER] Label distribution:")
    label_counts = {}
    with open(output_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lbl = row['label']
            label_counts[lbl] = label_counts.get(lbl, 0) + 1

    for lbl, count in sorted(label_counts.items()):
        pct = 100 * count / total_count
        print(f"    {lbl}: {count:,} ({pct:.1f}%)")


def create_example_labels(output_file: str):
    """Create an example time labels CSV."""
    example = [
        ('start', 'end', 'label'),
        ('# Time in seconds from start of recording', '', ''),
        ('# Labels: asphalt, sidewalk, grass, bump', '', ''),
        ('0', '30', 'asphalt'),
        ('30', '35', 'bump'),
        ('35', '60', 'asphalt'),
        ('60', '85', 'sidewalk'),
        ('85', '110', 'grass'),
        ('110', '120', 'asphalt'),
    ]

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        for row in example:
            if row[0].startswith('#'):
                f.write(f"# {row[0][2:]}\n")
            else:
                writer.writerow(row)

    print(f"[LABELER] Created example labels file: {output_file}")
    print("  Edit this file with your actual timestamps!")


def main():
    parser = argparse.ArgumentParser(
        description='Add surface labels to vibration data based on timestamps'
    )
    parser.add_argument(
        '--data', '-d',
        help='Input vibration CSV file'
    )
    parser.add_argument(
        '--labels', '-l',
        help='Time labels CSV file (start,end,label)'
    )
    parser.add_argument(
        '--output', '-o',
        help='Output labeled CSV file (default: input_labeled.csv)'
    )
    parser.add_argument(
        '--create-example',
        metavar='FILE',
        help='Create an example time labels CSV and exit'
    )

    args = parser.parse_args()

    # Create example if requested
    if args.create_example:
        create_example_labels(args.create_example)
        return

    # Validate required arguments
    if not args.data or not args.labels:
        parser.error("--data and --labels are required (or use --create-example)")

    # Default output filename
    if not args.output:
        base, ext = os.path.splitext(args.data)
        args.output = f"{base}_labeled{ext}"

    # Run labeling
    label_data(args.data, args.labels, args.output)


if __name__ == '__main__':
    main()
