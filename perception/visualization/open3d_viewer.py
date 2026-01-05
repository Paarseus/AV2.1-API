"""
Open3D Visualizer
Real-time 3D visualization of BEV image on ground plane

Displays BEV transformation as textured mesh on ground
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Dict, Optional
from ..core_types import BEVRepresentation, LayerConfig


class Open3DViewer:
    """
    Real-time Open3D visualizer for BEV representation

    Displays BEV image as textured mesh on ground plane (z=0)

    Usage:
        viewer = Open3DViewer(window_name="BEV 3D View")
        viewer.update(bev_representation)
        # ... in loop
        viewer.close()
    """

    def __init__(
        self,
        window_name: str = "BEV 3D Viewer",
        width: int = 1280,
        height: int = 720,
        show_grid: bool = True
    ):
        """
        Initialize Open3D visualizer

        Args:
            window_name: Window title
            width: Window width
            height: Window height
            show_grid: Show ground plane grid
        """
        # Add coordinate system info to window title
        full_title = f"{window_name} | Origin=(0,0,0) Yellow Sphere | X=Red Y=Green Z=Blue"
        self.window_name = full_title
        self.show_grid = show_grid

        # Create visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=full_title, width=width, height=height)

        # Configure rendering - disable lighting for pure color display
        opt = self.vis.get_render_option()
        opt.light_on = False  # Disable lighting
        opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Color  # Show vertex colors directly
        opt.background_color = np.array([0.05, 0.05, 0.05])  # Dark gray background

        # Geometry objects
        self.bev_mesh = None
        self.grid_lines = None
        self.coordinate_frame = None
        self.origin_marker = None
        self._mesh_initialized = False
        self._bev_bounds = None

        # Track first frame for bounding box reset
        self.first_frame = True

        # Set view angle
        self._setup_view()

        # Track if window is open
        self.is_open = True

    def _create_coordinate_frame(self, size: float = 3.0) -> o3d.geometry.TriangleMesh:
        """
        Create coordinate frame at origin (0,0,0)

        Standard color coding:
        - X-axis: RED (forward)
        - Y-axis: GREEN (left)
        - Z-axis: BLUE (up)
        """
        # Use Open3D's built-in coordinate frame
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=size,
            origin=[0, 0, 0]
        )
        return frame

    def _create_origin_marker(self) -> o3d.geometry.TriangleMesh:
        """Create yellow sphere at vehicle origin (0,0,0)"""
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        sphere.paint_uniform_color([1, 1, 0])  # Yellow
        sphere.compute_vertex_normals()
        return sphere

    def _create_grid(self, bounds: Tuple[float, float, float, float], spacing: float = 1.0) -> o3d.geometry.LineSet:
        """Create grid lines on ground plane"""
        x_min, x_max, y_min, y_max = bounds
        lines = []
        points = []
        idx = 0

        # Lines along X direction
        y = y_min
        while y <= y_max:
            points.append([x_min, y, 0.0])
            points.append([x_max, y, 0.0])
            lines.append([idx, idx + 1])
            idx += 2
            y += spacing

        # Lines along Y direction
        x = x_min
        while x <= x_max:
            points.append([x, y_min, 0.0])
            points.append([x, y_max, 0.0])
            lines.append([idx, idx + 1])
            idx += 2
            x += spacing

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([[0.3, 0.3, 0.3] for _ in lines])
        return line_set

    def _create_bev_mesh(self, bounds: Tuple[float, float, float, float], img_height: int, img_width: int) -> o3d.geometry.TriangleMesh:
        """Create mesh once with high resolution grid"""
        x_min, x_max, y_min, y_max = bounds

        # Use original image resolution for vertices (downsample less aggressively)
        step = 2  # Higher resolution for better quality
        h_verts = img_height // step
        w_verts = img_width // step

        # Create vertex grid
        v_indices = np.linspace(0, img_height - 1, h_verts)
        u_indices = np.linspace(0, img_width - 1, w_verts)

        v_grid, u_grid = np.meshgrid(v_indices, u_indices, indexing='ij')

        # Convert to world coordinates (STANDARD)
        # v=0 → x_max (far), v=height-1 → x_min (near)
        # u=0 → y_max (left), u=width-1 → y_min (right)
        x_coords = x_max - (v_grid / (img_height - 1)) * (x_max - x_min)
        y_coords = y_max - (u_grid / (img_width - 1)) * (y_max - y_min)
        z_coords = np.zeros_like(x_coords)  # Ground plane

        vertices = np.stack([x_coords.ravel(), y_coords.ravel(), z_coords.ravel()], axis=1).astype(np.float64)

        # Create triangles
        rows, cols = h_verts, w_verts
        triangles = []
        for i in range(rows - 1):
            for j in range(cols - 1):
                idx00 = i * cols + j
                idx01 = i * cols + (j + 1)
                idx10 = (i + 1) * cols + j
                idx11 = (i + 1) * cols + (j + 1)

                triangles.append([idx00, idx10, idx11])
                triangles.append([idx00, idx11, idx01])

        triangles = np.array(triangles, dtype=np.int32)

        # Create mesh
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()  # Required for visibility

        # Store grid info for color updates
        self._vertex_grid_shape = (h_verts, w_verts)
        self._vertex_step = step

        return mesh

    def _update_mesh_colors(self, bev_image: np.ndarray):
        """Update mesh vertex colors from BEV image (fast)"""
        # Sample colors at vertex positions
        step = self._vertex_step
        sampled = bev_image[::step, ::step]

        # Ensure size matches
        h, w = self._vertex_grid_shape
        sampled = sampled[:h, :w]

        # Extract colors (already RGB)
        colors = sampled.reshape(-1, 3).astype(np.float64) / 255.0

        # Update mesh colors
        self.bev_mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    def _setup_view(self):
        """
        Setup camera view - standard third-person perspective

        Standard autonomous vehicle visualization (matches ROS RViz):
        - Camera behind and above vehicle
        - Looking forward and down at scene
        - Z-axis pointing up
        """
        ctr = self.vis.get_view_control()

        # Standard third-person view
        ctr.set_lookat([10.0, 0.0, 0.0])     # Look at center of BEV (10m ahead)
        ctr.set_front([-0.5, 0.0, 0.2])    # Camera behind and above (backward, up)
        ctr.set_up([0.0, 0.0, 1.0])          # Z-axis is up (right-handed coordinate system)
        ctr.set_zoom(0.7)                     # Zoom level to see full scene

    def update(self, bev_repr: BEVRepresentation) -> bool:
        """Update visualization with BEV (optimized - mesh created once)"""
        if not self.is_open:
            return False

        # First frame: create mesh and grid
        if not self._mesh_initialized:
            height, width = bev_repr.bev_image.shape[:2]
            self._bev_bounds = bev_repr.bounds

            # Create coordinate frame at origin (0,0,0)
            self.coordinate_frame = self._create_coordinate_frame(size=3.0)
            self.vis.add_geometry(self.coordinate_frame, reset_bounding_box=False)

            # Create origin marker (yellow sphere at vehicle position)
            self.origin_marker = self._create_origin_marker()
            self.vis.add_geometry(self.origin_marker, reset_bounding_box=False)

            # Create mesh once
            self.bev_mesh = self._create_bev_mesh(bev_repr.bounds, height, width)
            self.vis.add_geometry(self.bev_mesh, reset_bounding_box=True)

            # Create grid if enabled
            if self.show_grid:
                self.grid_lines = self._create_grid(bev_repr.bounds, spacing=1.0)
                self.vis.add_geometry(self.grid_lines, reset_bounding_box=False)

            self._setup_view()
            self._mesh_initialized = True

        # Update colors only (FAST - no geometry recreation)
        self._update_mesh_colors(bev_repr.bev_image)
        self.vis.update_geometry(self.bev_mesh)

        # Render
        self.is_open = self.vis.poll_events()
        self.vis.update_renderer()

        return self.is_open

    def close(self):
        """Close the visualizer window"""
        if self.is_open:
            self.vis.destroy_window()
            self.is_open = False


class BEV2DViewer:
    """
    Simple 2D OpenCV-based BEV viewer
    Lightweight alternative to Open3D for 2D visualization
    """

    def __init__(self, window_name: str = "BEV View", scale: int = 2):
        """
        Initialize 2D BEV viewer

        Args:
            window_name: Window title
            scale: Upscaling factor for display
        """
        self.window_name = window_name
        self.scale = scale
        self.is_open = True

    def update(self, bev_repr: BEVRepresentation) -> bool:
        """
        Display BEV image

        Args:
            bev_repr: BEV representation

        Returns:
            True if window open, False if user pressed 'q'
        """
        import cv2

        # Upscale for better visibility
        display = cv2.resize(
            bev_repr.bev_image,
            (bev_repr.bev_image.shape[1] * self.scale,
             bev_repr.bev_image.shape[0] * self.scale),
            interpolation=cv2.INTER_CUBIC
        )

        # Add text overlay
        cv2.putText(display, f"BEV: {bev_repr.grid_resolution*1000:.1f}mm/px",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(self.window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.is_open = False
            return False

        return True

    def close(self):
        """Close the viewer window"""
        import cv2
        if self.is_open:
            cv2.destroyWindow(self.window_name)
            self.is_open = False


class MultiLayerViewer:
    """
    Multi-layer 3D visualizer with dynamic layer management

    Supports multiple data sources:
    - BEV mesh from camera IPM
    - LIDAR point clouds
    - Occupancy grids (voxel grids)
    - Planned paths (line sets)
    - Static reference layers (grid, coordinate frame, etc.)

    Usage:
        viewer = MultiLayerViewer()

        # Add layers dynamically
        viewer.add_layer("bev", LayerConfig("bev", "mesh", visible=True))
        viewer.add_layer("lidar", LayerConfig("lidar", "pointcloud", color=(1, 0, 0)))
        viewer.add_layer("path", LayerConfig("path", "lineset", color=(0, 1, 0)))

        # Update different data types
        viewer.update_bev(bev_representation)
        viewer.update_lidar(point_cloud)
        viewer.update_path(path_points)

        # Toggle visibility
        viewer.toggle_layer("lidar")

        # Cleanup
        viewer.close()
    """

    def __init__(
        self,
        window_name: str = "Multi-Layer 3D Viewer",
        width: int = 1280,
        height: int = 720,
        show_grid: bool = True,
        show_origin: bool = True
    ):
        """
        Initialize multi-layer visualizer

        Args:
            window_name: Window title
            width: Window width
            height: Window height
            show_grid: Show ground plane metric grid
            show_origin: Show coordinate frame and origin marker
        """
        self.window_name = window_name
        self.show_grid = show_grid
        self.show_origin = show_origin

        # Create visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=window_name, width=width, height=height)

        # Configure rendering
        opt = self.vis.get_render_option()
        opt.light_on = False
        opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Color
        opt.background_color = np.array([0.05, 0.05, 0.05])
        opt.point_size = 2.0

        # Layer registry: {layer_name: {"config": LayerConfig, "geometry": o3d.Geometry, "visible": bool}}
        self.layers: Dict[str, Dict] = {}

        # Static reference layers
        self._init_static_layers()

        # Setup view
        self._setup_view()

        # Window state
        self.is_open = True

        # BEV mesh tracking (for fast color updates)
        self._bev_vertex_grid_shape = None
        self._bev_vertex_step = None

    def _init_static_layers(self):
        """Initialize static reference layers (grid, coordinate frame, origin)"""
        if self.show_origin:
            # Coordinate frame at origin
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])
            self.vis.add_geometry(frame, reset_bounding_box=False)
            self.layers["_coordinate_frame"] = {
                "config": LayerConfig("_coordinate_frame", "mesh", visible=True),
                "geometry": frame,
                "visible": True
            }

            # Yellow sphere at origin
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
            sphere.paint_uniform_color([1, 1, 0])
            sphere.compute_vertex_normals()
            self.vis.add_geometry(sphere, reset_bounding_box=False)
            self.layers["_origin_marker"] = {
                "config": LayerConfig("_origin_marker", "mesh", visible=True, color=(1, 1, 0)),
                "geometry": sphere,
                "visible": True
            }

        if self.show_grid:
            # Default grid (will be replaced if BEV provides bounds)
            grid = self._create_grid_lines((0, 20, -10, 10), spacing=1.0)
            self.vis.add_geometry(grid, reset_bounding_box=False)
            self.layers["_grid"] = {
                "config": LayerConfig("_grid", "lineset", visible=True, color=(0.3, 0.3, 0.3)),
                "geometry": grid,
                "visible": True
            }

    def _create_grid_lines(self, bounds: Tuple[float, float, float, float], spacing: float = 1.0) -> o3d.geometry.LineSet:
        """Create grid lines on ground plane"""
        x_min, x_max, y_min, y_max = bounds
        lines = []
        points = []
        idx = 0

        # Lines along X direction
        y = y_min
        while y <= y_max:
            points.append([x_min, y, 0.0])
            points.append([x_max, y, 0.0])
            lines.append([idx, idx + 1])
            idx += 2
            y += spacing

        # Lines along Y direction
        x = x_min
        while x <= x_max:
            points.append([x, y_min, 0.0])
            points.append([x, y_max, 0.0])
            lines.append([idx, idx + 1])
            idx += 2
            x += spacing

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([[0.3, 0.3, 0.3] for _ in lines])
        return line_set

    def _create_bev_mesh(self, bounds: Tuple[float, float, float, float], img_height: int, img_width: int) -> o3d.geometry.TriangleMesh:
        """Create BEV mesh with high resolution grid"""
        x_min, x_max, y_min, y_max = bounds

        # Use image resolution for vertices
        step = 2
        h_verts = img_height // step
        w_verts = img_width // step

        # Create vertex grid
        v_indices = np.linspace(0, img_height - 1, h_verts)
        u_indices = np.linspace(0, img_width - 1, w_verts)
        v_grid, u_grid = np.meshgrid(v_indices, u_indices, indexing='ij')

        # Convert to world coordinates
        x_coords = x_max - (v_grid / (img_height - 1)) * (x_max - x_min)
        y_coords = y_max - (u_grid / (img_width - 1)) * (y_max - y_min)
        z_coords = np.zeros_like(x_coords)

        vertices = np.stack([x_coords.ravel(), y_coords.ravel(), z_coords.ravel()], axis=1).astype(np.float64)

        # Create triangles
        rows, cols = h_verts, w_verts
        triangles = []
        for i in range(rows - 1):
            for j in range(cols - 1):
                idx00 = i * cols + j
                idx01 = i * cols + (j + 1)
                idx10 = (i + 1) * cols + j
                idx11 = (i + 1) * cols + (j + 1)
                triangles.append([idx00, idx10, idx11])
                triangles.append([idx00, idx11, idx01])

        triangles = np.array(triangles, dtype=np.int32)

        # Create mesh
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()

        # Store grid info for fast color updates
        self._bev_vertex_grid_shape = (h_verts, w_verts)
        self._bev_vertex_step = step

        return mesh

    def _update_bev_mesh_colors(self, mesh: o3d.geometry.TriangleMesh, bev_image: np.ndarray):
        """Update BEV mesh vertex colors (fast)"""
        step = self._bev_vertex_step
        sampled = bev_image[::step, ::step]

        h, w = self._bev_vertex_grid_shape
        sampled = sampled[:h, :w]

        colors = sampled.reshape(-1, 3).astype(np.float64) / 255.0
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    def _setup_view(self):
        """Setup third-person camera view"""
        ctr = self.vis.get_view_control()
        ctr.set_lookat([10.0, 0.0, 0.0])
        ctr.set_front([-0.5, 0.0, 0.2])
        ctr.set_up([0.0, 0.0, 1.0])
        ctr.set_zoom(0.7)

    def add_layer(self, name: str, config: LayerConfig) -> bool:
        """
        Register a new layer

        Args:
            name: Unique layer identifier
            config: Layer configuration

        Returns:
            True if added, False if name already exists
        """
        if name in self.layers and not name.startswith("_"):
            print(f"[MultiLayerViewer] Layer '{name}' already exists")
            return False

        # Layer will be added when first data is provided
        self.layers[name] = {
            "config": config,
            "geometry": None,
            "visible": config.visible
        }
        return True

    def remove_layer(self, name: str) -> bool:
        """
        Remove a layer

        Args:
            name: Layer identifier

        Returns:
            True if removed, False if not found
        """
        if name not in self.layers or name.startswith("_"):
            print(f"[MultiLayerViewer] Cannot remove layer '{name}'")
            return False

        layer = self.layers[name]
        if layer["geometry"] is not None:
            self.vis.remove_geometry(layer["geometry"], reset_bounding_box=False)

        del self.layers[name]
        return True

    def toggle_layer(self, name: str) -> bool:
        """
        Toggle layer visibility

        Args:
            name: Layer identifier

        Returns:
            New visibility state (True = visible)
        """
        if name not in self.layers:
            print(f"[MultiLayerViewer] Layer '{name}' not found")
            return False

        layer = self.layers[name]
        layer["visible"] = not layer["visible"]

        if layer["geometry"] is not None:
            if layer["visible"]:
                self.vis.add_geometry(layer["geometry"], reset_bounding_box=False)
            else:
                self.vis.remove_geometry(layer["geometry"], reset_bounding_box=False)

        return layer["visible"]

    def update_bev(self, bev_repr: BEVRepresentation) -> bool:
        """
        Update BEV mesh layer

        Args:
            bev_repr: BEV representation from IPM

        Returns:
            True if successful
        """
        layer_name = "bev"

        # Add layer if not exists
        if layer_name not in self.layers:
            self.add_layer(layer_name, LayerConfig(layer_name, "mesh", visible=True))

        layer = self.layers[layer_name]

        # First time: create mesh
        if layer["geometry"] is None:
            height, width = bev_repr.bev_image.shape[:2]
            mesh = self._create_bev_mesh(bev_repr.bounds, height, width)
            layer["geometry"] = mesh
            self.vis.add_geometry(mesh, reset_bounding_box=True)

            # Update grid with BEV bounds
            if "_grid" in self.layers:
                old_grid = self.layers["_grid"]["geometry"]
                self.vis.remove_geometry(old_grid, reset_bounding_box=False)
                new_grid = self._create_grid_lines(bev_repr.bounds, spacing=1.0)
                self.layers["_grid"]["geometry"] = new_grid
                self.vis.add_geometry(new_grid, reset_bounding_box=False)

            # Reset view after bounding box was set (fixes camera positioning)
            self._setup_view()

        # Update colors only (fast)
        self._update_bev_mesh_colors(layer["geometry"], bev_repr.bev_image)
        self.vis.update_geometry(layer["geometry"])

        return True

    def update_lidar(self, points: np.ndarray, name: str = "lidar", color: Optional[Tuple[float, float, float]] = None) -> bool:
        """
        Update LIDAR point cloud layer

        Args:
            points: Point cloud (N, 3) in vehicle frame
            name: Layer name
            color: RGB color (0-1 range), None for per-point coloring

        Returns:
            True if successful
        """
        # Add layer if not exists
        if name not in self.layers:
            default_color = color if color else (1, 0, 0)
            self.add_layer(name, LayerConfig(name, "pointcloud", visible=True, color=default_color))

        layer = self.layers[name]

        # Create or update point cloud
        if layer["geometry"] is None:
            pcd = o3d.geometry.PointCloud()
            layer["geometry"] = pcd
            self.vis.add_geometry(pcd, reset_bounding_box=False)

        pcd = layer["geometry"]
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

        # Apply color
        if color is not None:
            pcd.paint_uniform_color(color)
        elif points.shape[1] >= 6:
            # Use per-point colors if available (assume columns 3-5 are RGB)
            pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6].astype(np.float64) / 255.0)
        else:
            pcd.paint_uniform_color(layer["config"].color)

        self.vis.update_geometry(pcd)
        return True

    def update_occupancy(self, grid: np.ndarray, origin: Tuple[float, float, float], resolution: float, name: str = "occupancy") -> bool:
        """
        Update occupancy grid voxel layer

        Args:
            grid: 3D occupancy grid (X, Y, Z) boolean or probability
            origin: Grid origin (x, y, z) in meters
            resolution: Voxel size in meters
            name: Layer name

        Returns:
            True if successful
        """
        # Add layer if not exists
        if name not in self.layers:
            self.add_layer(name, LayerConfig(name, "voxelgrid", visible=True, color=(0, 0.5, 1)))

        layer = self.layers[name]

        # Create voxel grid
        voxel_grid = o3d.geometry.VoxelGrid()
        voxel_grid.voxel_size = resolution
        voxel_grid.origin = origin

        # Add occupied voxels
        occupied = np.argwhere(grid > 0.5)
        for idx in occupied:
            voxel = o3d.geometry.Voxel()
            voxel.grid_index = idx
            voxel.color = layer["config"].color
            voxel_grid.add_voxel(voxel)

        # Replace geometry
        if layer["geometry"] is not None:
            self.vis.remove_geometry(layer["geometry"], reset_bounding_box=False)

        layer["geometry"] = voxel_grid
        self.vis.add_geometry(voxel_grid, reset_bounding_box=False)

        return True

    def update_path(self, points: np.ndarray, name: str = "path", color: Optional[Tuple[float, float, float]] = None) -> bool:
        """
        Update path line layer

        Args:
            points: Path waypoints (N, 3) in vehicle frame
            name: Layer name
            color: RGB color (0-1 range)

        Returns:
            True if successful
        """
        if len(points) < 2:
            return False

        # Add layer if not exists
        if name not in self.layers:
            default_color = color if color else (0, 1, 0)
            self.add_layer(name, LayerConfig(name, "lineset", visible=True, color=default_color))

        layer = self.layers[name]

        # Create line set
        lines = [[i, i + 1] for i in range(len(points) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        line_set.lines = o3d.utility.Vector2iVector(lines)

        line_color = color if color else layer["config"].color
        line_set.colors = o3d.utility.Vector3dVector([line_color for _ in lines])

        # Replace geometry
        if layer["geometry"] is not None:
            self.vis.remove_geometry(layer["geometry"], reset_bounding_box=False)

        layer["geometry"] = line_set
        self.vis.add_geometry(line_set, reset_bounding_box=False)

        return True

    def render(self) -> bool:
        """
        Render frame and poll events

        Returns:
            True if window open, False if closed
        """
        if not self.is_open:
            return False

        self.is_open = self.vis.poll_events()
        self.vis.update_renderer()
        return self.is_open

    def close(self):
        """Close visualizer window"""
        if self.is_open:
            self.vis.destroy_window()
            self.is_open = False
