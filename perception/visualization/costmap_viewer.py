"""
Costmap Visualizer

Real-time costmap visualization using matplotlib.

Usage:
    from perception import OccupancyGrid2D, Costmap
    from perception.visualization import CostmapVisualizer

    grid = OccupancyGrid2D()
    costmap = Costmap(robot_radius=0.5, safety_margin=0.2)

    viz = CostmapVisualizer(costmap, grid)

    # In your update loop:
    grid.update(points)
    costmap.update(grid)
    viz.update()
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from typing import Optional, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from ..costmap import Costmap
    from ..occupancy_grid import OccupancyGrid2D


class CostmapVisualizer:
    """Real-time costmap visualization"""

    def __init__(
        self,
        costmap: 'Costmap',
        grid: 'OccupancyGrid2D',
        update_hz: int = 10,
        title: str = 'Costmap'
    ):
        """
        Args:
            costmap: Costmap instance
            grid: OccupancyGrid2D instance (for extent calculation)
            update_hz: Update frequency for run() mode (default 10 Hz)
            title: Window title
        """
        self.costmap = costmap
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

        # Costmap display (RGB: white=free, orange=inflation, red=obstacle)
        self.im = self.ax.imshow(
            np.ones((self.grid.cells_y, self.grid.cells_x, 3)),
            extent=self.extent,
            origin='upper',
            interpolation='nearest'
        )

        # Axis config
        self.ax.set_xlim(self.extent[0], self.extent[1])
        self.ax.set_ylim(self.extent[2], self.extent[3])
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (m) - Forward')
        self.ax.set_ylabel('Y (m) - Left')
        self.ax.axhline(0, color='blue', linewidth=0.5, alpha=0.5)
        self.ax.axvline(0, color='blue', linewidth=0.5, alpha=0.5)
        self.ax.grid(True, alpha=0.3)

        # Vehicle marker with robot radius
        self.ax.plot(0, 0, 'b^', markersize=10, label='Vehicle')
        robot_circle = Circle(
            (0, 0), self.costmap.inflation_radius,
            fill=False, color='blue', linestyle='--', linewidth=1.5,
            label=f'Inflation radius ({self.costmap.inflation_radius:.2f}m)'
        )
        self.ax.add_patch(robot_circle)

        # Color legend (manual patches)
        from matplotlib.patches import Patch
        legend_elements = [
            self.ax.plot([], [], 'b^', markersize=10)[0],
            Circle((0, 0), 0.1, fill=False, color='blue', linestyle='--'),
            Patch(facecolor='#CC0000', label='Obstacle'),
            Patch(facecolor='#FFB366', label='Inflation zone'),
        ]
        self.ax.legend(
            legend_elements,
            ['Vehicle', f'Inflation radius ({self.costmap.inflation_radius:.2f}m)',
             'Obstacle', 'Inflation zone'],
            loc='upper right'
        )

        # Stats text
        self.stats = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            va='top', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

    def _create_rgb_image(self):
        """Get RGB image from costmap."""
        rgb = self.costmap.to_rgb()
        if rgb is None:
            return np.ones((self.grid.cells_y, self.grid.cells_x, 3))
        return rgb

    def update(self, stats_text: Optional[str] = None):
        """
        Update visualization with current costmap state

        Args:
            stats_text: Optional text to display in stats box
        """
        if self.costmap._costmap is None:
            return

        self.im.set_data(self._create_rgb_image())

        if stats_text:
            self.stats.set_text(stats_text)
        else:
            inflated_cells = np.sum(self.costmap._costmap > 0.5)
            raw_cells = np.sum(self.costmap._raw_obstacle_grid > 0.5) if self.costmap._raw_obstacle_grid is not None else 0
            inflation_cells = inflated_cells - raw_cells
            total_cells = self.costmap._costmap.size
            self.stats.set_text(
                f"Obstacle cells: {raw_cells}\n"
                f"Inflation cells: {inflation_cells}\n"
                f"Total collision: {inflated_cells} ({100*inflated_cells/total_cells:.1f}%)\n"
                f"Robot radius: {self.costmap.robot_radius:.2f}m\n"
                f"Safety margin: {self.costmap.safety_margin:.2f}m"
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self, update_callback: Optional[Callable[[], Optional[str]]] = None):
        """
        Start visualization with animation loop

        Args:
            update_callback: Optional function called each frame.
                             Should update grid/costmap and optionally return stats text.
        """
        def _animate(frame):
            stats_text = None
            if update_callback:
                stats_text = update_callback()

            if self.costmap._costmap is not None:
                self.im.set_data(self._create_rgb_image())

                if stats_text:
                    self.stats.set_text(stats_text)
                else:
                    inflated_cells = np.sum(self.costmap._costmap > 0.5)
                    raw_cells = np.sum(self.costmap._raw_obstacle_grid > 0.5) if self.costmap._raw_obstacle_grid is not None else 0
                    inflation_cells = inflated_cells - raw_cells
                    total_cells = self.costmap._costmap.size
                    self.stats.set_text(
                        f"Obstacle cells: {raw_cells}\n"
                        f"Inflation cells: {inflation_cells}\n"
                        f"Total collision: {inflated_cells} ({100*inflated_cells/total_cells:.1f}%)\n"
                        f"Robot radius: {self.costmap.robot_radius:.2f}m\n"
                        f"Safety margin: {self.costmap.safety_margin:.2f}m"
                    )

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
