#!/usr/bin/env python3
"""
Costmap Synthetic Test

Demonstrates costmap inflation with synthetic obstacles.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '..')
from perception import OccupancyGrid2D, Costmap


def add_obstacle(grid, cx, cy, radius, hits=5):
    """Add circular obstacle to grid with multiple hits"""
    for angle in np.linspace(0, 2 * np.pi, 20):
        for r in np.linspace(0, radius, 3):
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            for _ in range(hits):
                grid.update(np.array([[x, y]]))


def main():
    # Create grid and costmap
    grid = OccupancyGrid2D(
        width=20.0,
        height=20.0,
        resolution=0.2,
        use_raycasting=False
    )

    costmap = Costmap(robot_radius=0.5, safety_margin=0.2)

    print("=== Costmap Synthetic Test ===")
    print(f"Grid: {grid.width}m x {grid.height}m, resolution={grid.resolution}m")
    print(f"Robot radius: {costmap.robot_radius}m")
    print(f"Safety margin: {costmap.safety_margin}m")
    print(f"Inflation radius: {costmap.inflation_radius}m")
    print()

    # Add synthetic obstacles
    obstacles = [
        (5.0, 0.0, 0.5),    # 5m ahead, centered
        (7.0, 2.0, 0.8),    # 7m ahead, 2m left
        (6.0, -1.5, 0.6),   # 6m ahead, 1.5m right
        (3.0, 1.0, 0.3),    # 3m ahead, 1m left
    ]

    print("Adding obstacles:")
    for cx, cy, radius in obstacles:
        add_obstacle(grid, cx, cy, radius)
        print(f"  ({cx}, {cy}) radius={radius}m")

    # Update costmap
    costmap.update(grid, threshold=0.55)

    # Count cells
    collision_cells = np.sum(costmap._costmap > 0.5)
    total_cells = costmap._costmap.size
    print(f"\nCollision cells: {collision_cells} / {total_cells} ({100*collision_cells/total_cells:.1f}%)")

    # Test path collision
    print("\n=== Path Collision Tests ===")

    test_paths = [
        ("Clear path (y=3)", np.array([[0, 3], [2, 3], [4, 3], [6, 3], [8, 3]])),
        ("Through obstacle 1", np.array([[0, 0], [2, 0], [4, 0], [5, 0], [6, 0]])),
        ("Through obstacle 2", np.array([[0, 0], [3, 1], [5, 1.5], [7, 2]])),
        ("Near miss (y=-3)", np.array([[0, -3], [3, -3], [6, -3], [9, -3]])),
    ]

    for name, path in test_paths:
        collision, point = costmap.check_path(path)
        if collision:
            print(f"  {name}: COLLISION at ({point[0]:.1f}, {point[1]:.1f})")
        else:
            print(f"  {name}: Clear")

    # Visualization
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    extent = [
        -grid.origin_x, grid.width - grid.origin_x,
        -grid.origin_y, grid.height - grid.origin_y
    ]

    # Left: Occupancy Grid (probability)
    ax1 = axes[0]
    prob = grid.to_probability_grid()
    im1 = ax1.imshow(prob, extent=extent, origin='upper', cmap='RdYlGn_r', vmin=0, vmax=1)
    ax1.set_title('Occupancy Grid (Probability)')
    ax1.set_xlabel('X (m) - Forward')
    ax1.set_ylabel('Y (m) - Left')
    ax1.plot(0, 0, 'b^', markersize=10, label='Vehicle')
    ax1.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax1.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-2, 10)
    ax1.set_ylim(-5, 5)
    ax1.set_aspect('equal')
    fig.colorbar(im1, ax=ax1, label='Probability', shrink=0.8)

    # Right: Costmap (binary, inflated)
    ax2 = axes[1]
    im2 = ax2.imshow(costmap._costmap, extent=extent, origin='upper', cmap='Greys', vmin=0, vmax=1)
    ax2.set_title(f'Costmap (Inflated by {costmap.inflation_radius}m)')
    ax2.set_xlabel('X (m) - Forward')
    ax2.set_ylabel('Y (m) - Left')
    ax2.plot(0, 0, 'b^', markersize=10, label='Vehicle')

    # Draw inflation circle at origin
    circle = plt.Circle((0, 0), costmap.inflation_radius, fill=False,
                         color='blue', linestyle='--', linewidth=2, label='Inflation radius')
    ax2.add_patch(circle)

    # Draw test paths
    colors = ['green', 'red', 'orange', 'cyan']
    for (name, path), color in zip(test_paths, colors):
        collision, _ = costmap.check_path(path)
        style = '--' if collision else '-'
        ax2.plot(path[:, 0], path[:, 1], style, color=color, linewidth=2,
                 label=f'{name} ({"blocked" if collision else "clear"})')

    ax2.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax2.axvline(0, color='gray', linewidth=0.5, alpha=0.5)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(-2, 10)
    ax2.set_ylim(-5, 5)
    ax2.set_aspect('equal')
    ax2.legend(loc='upper right', fontsize=8)

    cbar2 = fig.colorbar(im2, ax=ax2, shrink=0.8)
    cbar2.set_ticks([0, 1])
    cbar2.set_ticklabels(['Free', 'Collision'])

    plt.tight_layout()
    plt.savefig('costmap_test.png', dpi=150)
    print("\nSaved: costmap_test.png")
    plt.show()


if __name__ == "__main__":
    main()
