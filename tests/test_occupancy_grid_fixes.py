"""
Test script for OccupancyGrid2D V1 fixes

Tests:
1. Input validation
2. Time-based temporal decay
3. Fast inflation performance
4. Thread safety with copy()
5. Basic functionality
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import time

def test_input_validation():
    """Test that input validation catches invalid parameters"""
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing input validation...")

    # Valid parameters should work
    try:
        grid = OccupancyGrid2D(width=40.0, height=40.0, resolution=0.1)
        print("  ✓ Valid parameters accepted")
    except AssertionError:
        print("  ✗ Valid parameters rejected!")
        return False

    # Invalid width
    try:
        grid = OccupancyGrid2D(width=-10.0)
        print("  ✗ Negative width not rejected!")
        return False
    except AssertionError:
        print("  ✓ Negative width rejected")

    # Invalid resolution
    try:
        grid = OccupancyGrid2D(resolution=0.0)
        print("  ✗ Zero resolution not rejected!")
        return False
    except AssertionError:
        print("  ✓ Zero resolution rejected")

    # Invalid decay factor
    try:
        grid = OccupancyGrid2D(decay_factor=1.5)
        print("  ✗ Invalid decay factor (>1) not rejected!")
        return False
    except AssertionError:
        print("  ✓ Invalid decay factor rejected")

    # Invalid ground z bounds
    try:
        grid = OccupancyGrid2D(ground_z_min=0.5, ground_z_max=0.3)
        print("  ✗ Invalid ground z bounds not rejected!")
        return False
    except AssertionError:
        print("  ✓ Invalid ground z bounds rejected")

    print("  ✓ All input validation tests passed\n")
    return True


def test_time_based_decay():
    """Test that temporal decay is time-based, not frame-based"""
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing time-based temporal decay...")

    # Create grid with known decay factor (0.5 means 50% decay per second)
    grid = OccupancyGrid2D(decay_factor=0.5, log_odds_occ=2.0)

    # Add obstacle
    points = np.array([[5.0, 0.0, 0.0]])
    grid.update_from_lidar(points)
    initial_value = grid.grid[grid._world_to_grid(5.0, 0.0)]

    print(f"  Initial log-odds: {initial_value:.3f}")

    # Wait 1 second and update with empty points
    time.sleep(1.0)
    grid.update_from_lidar(np.array([]))

    after_1s = grid.grid[grid._world_to_grid(5.0, 0.0)]
    print(f"  After 1s: {after_1s:.3f}")

    # Expected: initial_value * 0.5
    expected = initial_value * 0.5
    error = abs(after_1s - expected)

    if error < 0.1:  # Allow 10% tolerance for timing
        print(f"  ✓ Decay matches expected ({expected:.3f}), error: {error:.3f}")
    else:
        print(f"  ✗ Decay mismatch! Expected {expected:.3f}, got {after_1s:.3f}")
        return False

    # Wait another second
    time.sleep(1.0)
    grid.update_from_lidar(np.array([]))

    after_2s = grid.grid[grid._world_to_grid(5.0, 0.0)]
    expected_2s = initial_value * (0.5 ** 2)
    error_2s = abs(after_2s - expected_2s)

    print(f"  After 2s: {after_2s:.3f}")

    if error_2s < 0.1:
        print(f"  ✓ Second decay matches expected ({expected_2s:.3f}), error: {error_2s:.3f}")
    else:
        print(f"  ✗ Second decay mismatch! Expected {expected_2s:.3f}, got {after_2s:.3f}")
        return False

    print("  ✓ Time-based decay working correctly\n")
    return True


def test_inflation_performance():
    """Test that inflation runs in ~5ms with scipy"""
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing inflation performance...")

    # Create grid with obstacles
    grid = OccupancyGrid2D(width=40.0, height=40.0, resolution=0.1)

    # Add multiple obstacles
    np.random.seed(42)
    for i in range(10):
        angles = np.linspace(0, 2*np.pi, 50)
        cx, cy = np.random.uniform(2, 15), np.random.uniform(-10, 10)
        x = cx + 0.5 * np.cos(angles)
        y = cy + 0.5 * np.sin(angles)
        z = np.random.uniform(0.0, 0.3, 50)
        obs = np.column_stack([x, y, z])
        grid.update_from_lidar(obs)

    # Benchmark inflation
    times = []
    for _ in range(5):
        start = time.time()
        grid.inflate_obstacles(radius=1.0, threshold=0.5)
        elapsed = (time.time() - start) * 1000  # Convert to ms
        times.append(elapsed)

    avg_time = np.mean(times)
    max_time = np.max(times)

    print(f"  Inflation times: {[f'{t:.1f}ms' for t in times]}")
    print(f"  Average: {avg_time:.1f}ms")
    print(f"  Max: {max_time:.1f}ms")

    # Check if scipy is available
    try:
        import scipy.ndimage
        has_scipy = True
        threshold = 20.0  # Should be ~5ms with scipy
    except ImportError:
        has_scipy = False
        threshold = 1000.0  # Fallback is slow but should complete
        print("  (scipy not available, using fallback method)")

    if avg_time < threshold:
        print(f"  ✓ Performance acceptable (< {threshold}ms)\n")
        return True
    else:
        print(f"  ✗ Performance too slow (> {threshold}ms)\n")
        return False


def test_thread_safety():
    """Test that copy() creates independent snapshots"""
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing thread safety (copy method)...")

    # Create grid and add obstacles
    grid = OccupancyGrid2D()
    points = np.array([[5.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    grid.update_from_lidar(points)
    grid.inflate_obstacles(radius=1.0)

    # Create snapshot
    snapshot = grid.copy()

    # Verify snapshot has same data
    if not np.array_equal(snapshot.grid, grid.grid):
        print("  ✗ Snapshot grid data doesn't match!")
        return False
    print("  ✓ Snapshot has same grid data")

    if snapshot.inflated_grid is not None and grid.inflated_grid is not None:
        if not np.array_equal(snapshot.inflated_grid, grid.inflated_grid):
            print("  ✗ Snapshot inflated grid doesn't match!")
            return False
        print("  ✓ Snapshot has same inflated grid data")

    # Verify snapshot is independent (different memory)
    if snapshot.grid is grid.grid:
        print("  ✗ Snapshot shares memory with original (not independent)!")
        return False
    print("  ✓ Snapshot is independent (different memory)")

    # Modify original and verify snapshot unchanged
    original_snapshot_value = snapshot.grid.copy()
    grid.clear()

    if np.array_equal(snapshot.grid, grid.grid):
        print("  ✗ Snapshot changed when original was modified!")
        return False
    print("  ✓ Snapshot unchanged after modifying original")

    if not np.array_equal(snapshot.grid, original_snapshot_value):
        print("  ✗ Snapshot data changed unexpectedly!")
        return False
    print("  ✓ Snapshot data preserved")

    print("  ✓ Thread safety (copy) working correctly\n")
    return True


def test_basic_functionality():
    """Test that basic grid operations still work"""
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing basic functionality...")

    grid = OccupancyGrid2D(width=20.0, height=20.0, resolution=0.2)

    # Test update from lidar
    points = np.array([
        [5.0, 0.0, 0.0],
        [5.0, 0.0, 0.1],
        [5.0, 0.0, 0.2],
    ])
    count = grid.update_from_lidar(points)

    if count != 3:
        print(f"  ✗ Update failed: expected 3 points, got {count}")
        return False
    print(f"  ✓ Updated {count} points from LIDAR")

    # Test occupancy query
    occ = grid.get_occupancy(5.0, 0.0)
    if occ <= 0.0 or occ > 1.0:
        print(f"  ✗ Invalid occupancy value: {occ}")
        return False
    print(f"  ✓ Occupancy at (5.0, 0.0): {occ:.3f}")

    # Test closest obstacle
    result = grid.get_closest_obstacle(max_range=20.0, threshold=0.5)
    if result is None:
        print("  ✗ Failed to find obstacle")
        return False
    x, y, dist = result
    print(f"  ✓ Closest obstacle at ({x:.1f}, {y:.1f}), dist={dist:.1f}m")

    # Test path collision
    waypoints = np.array([[5.0, 0.0], [10.0, 0.0]])
    collision, wp = grid.check_path_collision(waypoints, threshold=0.5)
    if not collision:
        print("  ✗ Failed to detect collision on occupied cell")
        return False
    print(f"  ✓ Path collision detected at waypoint {wp}")

    # Test inflation
    grid.inflate_obstacles(radius=0.5, threshold=0.5)
    if grid.inflated_grid is None:
        print("  ✗ Inflation failed")
        return False
    print("  ✓ Obstacle inflation successful")

    # Test inflated occupancy
    inflated_occ = grid.get_inflated_occupancy(5.0, 0.0)
    if inflated_occ <= 0.0:
        print(f"  ✗ Inflated occupancy is zero at obstacle location")
        return False
    print(f"  ✓ Inflated occupancy: {inflated_occ:.3f}")

    # Test probability grid conversion
    prob_grid = grid.to_probability_grid()
    if prob_grid.shape != (grid.cells_y, grid.cells_x):
        print(f"  ✗ Probability grid has wrong shape")
        return False
    if not np.all((prob_grid >= 0.0) & (prob_grid <= 1.0)):
        print(f"  ✗ Probability grid has values outside [0, 1]")
        return False
    print(f"  ✓ Probability grid conversion works")

    print("  ✓ All basic functionality tests passed\n")
    return True


def test_perception_state_integration():
    """Test PerceptionState thread safety with snapshot"""
    from perception.state import PerceptionState
    from perception.occupancy_grid import OccupancyGrid2D

    print("Testing PerceptionState integration...")

    state = PerceptionState()
    grid = OccupancyGrid2D()

    # Add obstacle to grid
    points = np.array([[5.0, 0.0, 0.0]])
    grid.update_from_lidar(points)

    # Update state with grid
    state.update(occupancy_grid=grid)

    # Get control data (should return snapshot)
    control_data = state.get_control_data()
    snapshot = control_data['occupancy_grid']

    if snapshot is None:
        print("  ✗ No occupancy grid in control data")
        return False
    print("  ✓ Got occupancy grid from control data")

    # Verify it's a snapshot (different object)
    if snapshot is grid:
        print("  ✗ Control data returned direct reference (not thread-safe)!")
        return False
    print("  ✓ Control data returned snapshot (thread-safe)")

    # Verify snapshot has same data
    if not np.array_equal(snapshot.grid, grid.grid):
        print("  ✗ Snapshot data doesn't match original")
        return False
    print("  ✓ Snapshot data matches original")

    # Modify original and verify snapshot unchanged
    original_snapshot_data = snapshot.grid.copy()
    grid.clear()

    if np.array_equal(snapshot.grid, grid.grid):
        print("  ✗ Snapshot was modified when original changed (not independent)!")
        return False
    print("  ✓ Snapshot independent from original")

    print("  ✓ PerceptionState integration working correctly\n")
    return True


def main():
    print("=" * 60)
    print("OccupancyGrid2D V1 - Fix Verification Tests")
    print("=" * 60)
    print()

    tests = [
        ("Input Validation", test_input_validation),
        ("Time-Based Decay", test_time_based_decay),
        ("Inflation Performance", test_inflation_performance),
        ("Thread Safety (copy)", test_thread_safety),
        ("Basic Functionality", test_basic_functionality),
        ("PerceptionState Integration", test_perception_state_integration),
    ]

    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"  ✗ Test crashed: {e}\n")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    print("=" * 60)
    print("Test Summary")
    print("=" * 60)

    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status:8} {name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)

    print()
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("\n✓ All fixes verified successfully!")
        return 0
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        return 1


if __name__ == '__main__':
    sys.exit(main())
