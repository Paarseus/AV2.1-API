=============
Visualization
=============

Visualization tools for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The UTM Navigator includes several visualization tools for debugging and monitoring:

- **GPS Visualizer**: Real-time position and route display
- **Occupancy Grid Visualizer**: 2D obstacle map
- **Open3D Viewer**: 3D point cloud visualization
- **BEV Viewer**: Bird's-eye view display

GPS Visualizer
==============

Real-time display of vehicle position and planned route.

Usage
-----

.. code-block:: python

   from gps_visualizer import GPSVisualizer

   # Initialize with waypoints
   viz = GPSVisualizer(waypoints)

   # Update in control loop
   while running:
       position = gps.get_position()
       heading = gps.get_heading()
       viz.update(position, heading)

Features
--------

- Current position marker
- Heading arrow
- Planned route line
- Visited path trail
- RTK status indicator

Implementation
--------------

.. code-block:: python

   class GPSVisualizer:
       def __init__(self, waypoints):
           self.fig, self.ax = plt.subplots()
           self.waypoints = waypoints

           # Plot route
           xs = [w[0] for w in waypoints]
           ys = [w[1] for w in waypoints]
           self.ax.plot(xs, ys, 'g-', label='Route')

           # Position marker
           self.position_marker, = self.ax.plot([], [], 'bo', markersize=10)

           # Heading arrow
           self.heading_arrow = None

           plt.ion()
           plt.show()

       def update(self, position, heading):
           x, y = position

           # Update position
           self.position_marker.set_data([x], [y])

           # Update heading arrow
           if self.heading_arrow:
               self.heading_arrow.remove()
           dx = np.cos(heading) * 5
           dy = np.sin(heading) * 5
           self.heading_arrow = self.ax.arrow(x, y, dx, dy,
                                               head_width=1, color='red')

           self.fig.canvas.draw()
           self.fig.canvas.flush_events()

Occupancy Grid Visualizer
=========================

Real-time 2D display of the probabilistic occupancy grid.

Usage
-----

.. code-block:: python

   from perception.visualization.occupancy_visualizer import OccupancyGridVisualizer

   grid = OccupancyGrid()
   viz = OccupancyGridVisualizer(grid)

   while running:
       grid.update_from_lidar(lidar_points)
       viz.update()

Features
--------

- Color-coded occupancy probability (white=free, black=occupied)
- Vehicle position marker
- Grid coordinate overlay
- Real-time updates

Implementation
--------------

.. code-block:: python

   class OccupancyGridVisualizer:
       def __init__(self, grid):
           self.grid = grid
           self.fig, self.ax = plt.subplots()

           # Initialize heatmap
           prob_grid = grid.to_probability()
           self.im = self.ax.imshow(
               prob_grid,
               cmap='gray_r',
               vmin=0, vmax=1,
               origin='lower'
           )

           plt.colorbar(self.im, label='Occupancy Probability')
           plt.ion()
           plt.show()

       def update(self):
           prob_grid = self.grid.to_probability()
           self.im.set_data(prob_grid)
           self.fig.canvas.draw()
           self.fig.canvas.flush_events()

Open3D Point Cloud Viewer
=========================

3D visualization of LIDAR point clouds.

Usage
-----

.. code-block:: python

   from perception.visualization.open3d_viewer import Open3DViewer

   viewer = Open3DViewer()

   while running:
       points = lidar.get_scan()
       viewer.update(points)

Features
--------

- 3D point cloud display
- Color by height, intensity, or custom
- Interactive camera controls
- Vehicle coordinate frame axes

Implementation
--------------

.. code-block:: python

   import open3d as o3d

   class Open3DViewer:
       def __init__(self):
           self.vis = o3d.visualization.Visualizer()
           self.vis.create_window()

           # Create point cloud object
           self.pcd = o3d.geometry.PointCloud()
           self.vis.add_geometry(self.pcd)

           # Add coordinate frame
           frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
           self.vis.add_geometry(frame)

       def update(self, points):
           self.pcd.points = o3d.utility.Vector3dVector(points)

           # Color by height
           colors = self.height_to_color(points[:, 2])
           self.pcd.colors = o3d.utility.Vector3dVector(colors)

           self.vis.update_geometry(self.pcd)
           self.vis.poll_events()
           self.vis.update_renderer()

       def height_to_color(self, heights):
           """Map heights to colors."""
           normalized = (heights - heights.min()) / (heights.max() - heights.min() + 1e-6)
           colors = plt.cm.viridis(normalized)[:, :3]
           return colors

BEV (Bird's Eye View) Viewer
============================

Display of inverse perspective mapped camera images.

Usage
-----

.. code-block:: python

   from perception.visualization.bev_viewer import BEVViewer

   ipm = IPMProcessor(camera_matrix, dist_coeffs)
   viewer = BEVViewer()

   while running:
       frame = camera.get_frame()
       bev = ipm.transform(frame)
       viewer.update(bev)

Combined Visualization
======================

Display multiple visualizations simultaneously:

.. code-block:: python

   class CombinedVisualizer:
       def __init__(self):
           self.fig = plt.figure(figsize=(15, 5))

           # GPS view
           self.ax_gps = self.fig.add_subplot(131)
           self.ax_gps.set_title('GPS Position')

           # Occupancy grid
           self.ax_grid = self.fig.add_subplot(132)
           self.ax_grid.set_title('Occupancy Grid')

           # Camera view
           self.ax_cam = self.fig.add_subplot(133)
           self.ax_cam.set_title('Camera')

           plt.tight_layout()
           plt.ion()
           plt.show()

       def update(self, gps_data, grid, camera_frame):
           # Update GPS plot
           self.ax_gps.clear()
           self.ax_gps.plot(*gps_data['position'], 'bo')

           # Update grid
           self.ax_grid.clear()
           self.ax_grid.imshow(grid.to_probability(), cmap='gray_r')

           # Update camera
           self.ax_cam.clear()
           self.ax_cam.imshow(cv2.cvtColor(camera_frame, cv2.COLOR_BGR2RGB))

           self.fig.canvas.draw()
           self.fig.canvas.flush_events()

Performance Tips
================

1. **Update Rate**: Limit visualization to 10-30 Hz
2. **Disable When Not Needed**: Visualization consumes CPU/GPU
3. **Separate Thread**: Run visualization in background thread
4. **Downsampling**: Display subset of points for large clouds

.. code-block:: python

   class AsyncVisualizer:
       def __init__(self, visualizer, update_hz=10):
           self.visualizer = visualizer
           self.update_period = 1.0 / update_hz
           self.data_queue = queue.Queue(maxsize=1)
           self._running = True

           self._thread = threading.Thread(target=self._update_loop)
           self._thread.start()

       def _update_loop(self):
           while self._running:
               try:
                   data = self.data_queue.get(timeout=0.1)
                   self.visualizer.update(data)
               except queue.Empty:
                   pass

       def submit(self, data):
           try:
               self.data_queue.put_nowait(data)
           except queue.Full:
               pass  # Skip if previous update not finished
