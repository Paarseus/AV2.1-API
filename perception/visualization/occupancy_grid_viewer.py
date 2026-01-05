"""
Occupancy Grid Visualizer

Real-time occupancy grid visualization using matplotlib.

Usage:
    from perception import OccupancyGrid2D
    from perception.visualization import OccupancyGridVisualizer

    grid = OccupancyGrid2D()
    viz = OccupancyGridVisualizer(grid)

    # In your update loop:
    grid.update(points)
    viz.update()

    # Or use run() with a callback:
    viz.run(update_callback=lambda: grid.update(get_points()))
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Optional, Callable


class OccupancyGridVisualizer:
    """Real-time occupancy grid visualization"""

    def __init__(self, grid, update_hz: int = 10, title: str = 'Occupancy Grid'):
        """
        Args:
            grid: OccupancyGrid2D instance
            update_hz: Update frequency for run() mode (default 10 Hz)
            title: Window title
        """
        self.grid = grid
        self.interval = 1000 // update_hz
        self.title = title

        # Grid extent in world coordinates
        self.extent = [
            -grid.origin_x, grid.width - grid.origin_x,
            -grid.origin_y, grid.height - grid.origin_y
        ]

        self._setup_figure()

    def _setup_figure(self):
        """Initialize figure and artists"""
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title(self.title)

        # Grid display
        self.im = self.ax.imshow(
            np.zeros((self.grid.cells_y, self.grid.cells_x)),
            extent=self.extent,
            origin='upper',
            cmap='RdYlGn_r',
            vmin=0.0,
            vmax=1.0,
            interpolation='nearest'
        )

        # Axis config
        self.ax.set_xlim(self.extent[0], self.extent[1])
        self.ax.set_ylim(self.extent[2], self.extent[3])
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
        self.ax.axvline(0, color='gray', linewidth=0.5, alpha=0.5)

        # Vehicle marker
        self.ax.plot(0, 0, 'b^', markersize=8)

        # Colorbar
        self.fig.colorbar(self.im, ax=self.ax, label='Occupancy', shrink=0.8)

        # Stats text
        self.stats = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            va='top', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

    def update(self, stats_text: Optional[str] = None):
        """
        Update visualization with current grid state

        Args:
            stats_text: Optional text to display in stats box
        """
        prob = self.grid.to_probability_grid()
        self.im.set_data(prob)

        if stats_text:
            self.stats.set_text(stats_text)
        else:
            occupied = np.sum(prob > 0.5)
            self.stats.set_text(f"Occupied: {occupied}")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self, update_callback: Optional[Callable[[], Optional[str]]] = None):
        """
        Start visualization with animation loop

        Args:
            update_callback: Optional function called each frame.
                             Should update the grid and optionally return stats text.
                             Example: lambda: (grid.update(points), f"Points: {len(points)}")[1]
        """
        def _animate(frame):
            stats_text = None
            if update_callback:
                stats_text = update_callback()

            prob = self.grid.to_probability_grid()
            self.im.set_data(prob)

            if stats_text:
                self.stats.set_text(stats_text)
            else:
                occupied = np.sum(prob > 0.5)
                self.stats.set_text(f"Occupied: {occupied}")

            return [self.im, self.stats]

        self.ani = FuncAnimation(
            self.fig, _animate,
            interval=self.interval,
            blit=True,
            cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()

    def show(self):
        """Display current state (non-animated)"""
        self.update()
        plt.show()
