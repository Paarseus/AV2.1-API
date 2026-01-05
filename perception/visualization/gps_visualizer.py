import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')  # Use non-blocking backend
import osmnx as ox
import numpy as np
from typing import Optional, Callable

class GPSVisualizer:
    """
    Real-time GPS tracking visualizer with route display.

    MODULAR DESIGN for control loop integration:
    - Push-based: Control loop calls update(x, y)
    - Non-blocking: Uses plt.pause() for fast rendering
    - Optional: Easy to disable by not creating or calling update()
    - Loosely coupled: No dependency on specific Navigator class

    Shows:
    - Street network from OSMnx
    - Planned route
    - Current vehicle position
    - Position trail history
    - Start and end points
    """

    def __init__(
        self,
        graph,
        route_waypoints: np.ndarray,
        start_xy: tuple,
        end_xy: tuple,
        figsize: tuple = (14, 12),
        auto_show: bool = True
    ):
        """
        Initialize the GPS visualizer.

        Args:
            graph: OSMnx graph object (from Navigator.G)
            route_waypoints: Nx2 array of [x, y] waypoints in UTM meters
            start_xy: (x, y) start position in UTM meters
            end_xy: (x, y) end position in UTM meters
            figsize: Figure size (width, height) in inches
            auto_show: If True, automatically show window on first update()
        """
        self.graph = graph
        self.route_waypoints = route_waypoints
        self.start_x, self.start_y = start_xy
        self.end_x, self.end_y = end_xy
        self.auto_show = auto_show
        self._shown = False

        # Position history
        self.pos_history_x = []
        self.pos_history_y = []

        # Setup the figure
        self._setup_figure(figsize)

    def _setup_figure(self, figsize: tuple):
        """Setup the matplotlib figure and axes."""
        self.fig, self.ax = plt.subplots(figsize=figsize)

        # Plot the street network
        ox.plot_graph(
            self.graph,
            ax=self.ax,
            node_size=0,
            edge_linewidth=0.5,
            edge_color='gray',
            show=False,
            close=False
        )
        self.ax.set_aspect('equal', adjustable='box')

        # Plot the planned route
        route_x = self.route_waypoints[:, 0]
        route_y = self.route_waypoints[:, 1]
        self.ax.plot(route_x, route_y, 'b-', linewidth=3, label='Route', zorder=5)

        # Plot start and end points
        self.ax.plot(self.start_x, self.start_y, 'go', markersize=15, label='Start', zorder=6)
        self.ax.plot(self.end_x, self.end_y, 'ro', markersize=15, label='End', zorder=6)

        # Current position (to be updated)
        self.current_pos, = self.ax.plot([], [], 'mo', markersize=20, label='You', zorder=7)

        # Position trail
        self.trail, = self.ax.plot([], [], 'm-', linewidth=2, alpha=0.7, label='Trail', zorder=6)

        # Target point (Pure Pursuit lookahead target)
        self.target_pos, = self.ax.plot([], [], 'go', markersize=15, label='Target', zorder=7)

        # Line from vehicle to target
        self.target_line, = self.ax.plot([], [], 'g--', linewidth=2, alpha=0.6, zorder=6)

        # Labels and title
        self.ax.set_xlabel('Easting (m)', fontsize=12)
        self.ax.set_ylabel('Northing (m)', fontsize=12)
        self.ax.set_title('Autonomous Navigation - Real-Time Position (UTM)', fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right', fontsize=10)

        plt.tight_layout()

        # Make interactive
        plt.ion()  # Turn on interactive mode

    def update(self, x: float, y: float, target_x: float = None, target_y: float = None):
        """
        Update the visualization with new position and optional target point.

        Call this method from your control loop to push position updates.
        This method is non-blocking and returns quickly (< 10ms).

        Args:
            x: Current x position in UTM meters (Easting)
            y: Current y position in UTM meters (Northing)
            target_x: Optional target point x in UTM meters (for Pure Pursuit debugging)
            target_y: Optional target point y in UTM meters (for Pure Pursuit debugging)

        Example:
            while running:
                lat, lon, _ = xsens.get_current_position()
                x, y = navigator.gps_to_utm(lat, lon)
                visualizer.update(x, y, target_x, target_y)  # Non-blocking

                # Control loop continues immediately
                steering = controller.get_steering_angle(...)
        """
        # Show window on first update if auto_show
        if self.auto_show and not self._shown:
            plt.show(block=False)
            self._shown = True

        # Update current position marker
        self.current_pos.set_data([x], [y])

        # Update trail
        self.pos_history_x.append(x)
        self.pos_history_y.append(y)
        self.trail.set_data(self.pos_history_x, self.pos_history_y)

        # Update target point if provided
        if target_x is not None and target_y is not None:
            self.target_pos.set_data([target_x], [target_y])
            self.target_line.set_data([x, target_x], [y, target_y])
        else:
            # Hide target if not provided
            self.target_pos.set_data([], [])
            self.target_line.set_data([], [])

        # Redraw (non-blocking)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # Small pause to allow rendering

    def show(self):
        """
        Manually show the visualization window.
        Only needed if auto_show=False in __init__.
        """
        if not self._shown:
            plt.show(block=False)
            self._shown = True

    def close(self):
        """Close the visualization window."""
        plt.close(self.fig)
        self._shown = False

    def clear_trail(self):
        """Clear the position trail history."""
        self.pos_history_x.clear()
        self.pos_history_y.clear()
        self.trail.set_data([], [])
        if self._shown:
            self.fig.canvas.draw_idle()
            plt.pause(0.001)

    def save_trail(self, filename: str = "trail.csv"):
        """Save the position trail to a CSV file."""
        if not self.pos_history_x:
            print("No trail data to save")
            return

        trail_data = np.column_stack([self.pos_history_x, self.pos_history_y])
        np.savetxt(
            filename,
            trail_data,
            delimiter=',',
            header='utm_x,utm_y',
            comments=''
        )
        print(f"Trail saved to {filename} ({len(self.pos_history_x)} points)")


# =============================================================================
# Example Usage
# =============================================================================

if __name__ == "__main__":
    """
    Example of how to use the modular GPSVisualizer in a control loop.
    """
    from planning import Navigator
    import time

    print("="*70)
    print("GPS Visualizer Test (Modular Design)")
    print("="*70)
    print()

    # Setup navigator
    nav = Navigator(verbose=True)

    # Plan a route
    start_lat, start_lon = 34.0591507, -117.8219452
    end_lat = 34.059765
    end_lon = -117.820689

    distance = nav.plan_route(start_lat, start_lon, end_lat, end_lon)
    if not distance:
        print("Could not plan route!")
        exit(1)

    nav.get_waypoints(spacing_meters=1.0)
    waypoints = nav.get_path_array()

    print(f"Route planned: {distance:.1f}m with {len(waypoints)} waypoints\n")

    # Create visualizer (modular - pass only what's needed)
    start_xy = nav.gps_to_utm(start_lat, start_lon)
    end_xy = nav.gps_to_utm(end_lat, end_lon)

    visualizer = GPSVisualizer(
        graph=nav.G,
        route_waypoints=waypoints,
        start_xy=start_xy,
        end_xy=end_xy,
        figsize=(14, 12),
        auto_show=True
    )

    print("Starting control loop simulation...")
    print("Visualizer updates pushed from control loop (non-blocking)\n")

    # Simulated control loop
    sim_index = 0
    loop_rate = 20  # Hz
    dt = 1.0 / loop_rate

    try:
        while sim_index < len(waypoints):
            loop_start = time.time()

            # 1. SENSE: Get position (simulated)
            x = waypoints[sim_index, 0] + np.random.normal(0, 0.5)
            y = waypoints[sim_index, 1] + np.random.normal(0, 0.5)

            # 2. PLAN: (skipped in this example)

            # 3. CONTROL: (skipped in this example)

            # 4. VISUALIZE: Push update (non-blocking, < 10ms)
            visualizer.update(x, y)

            # 5. MAINTAIN LOOP RATE
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Print loop timing
            if sim_index % 20 == 0:
                print(f"Loop {sim_index}: {elapsed*1000:.1f}ms (target: {dt*1000:.1f}ms)")

            sim_index += 1

        print("\nSimulation complete. Keeping window open...")
        print("Close window to exit.\n")

        # Keep window open
        plt.show(block=True)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        visualizer.close()
        print("Visualization closed")
