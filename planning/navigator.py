# osmnx_class_utm.py — Navigator in a projected metric CRS (UTM), ready for control/viz

import osmnx as ox
import networkx as nx
import numpy as np
from pyproj import Transformer
from shapely.geometry import LineString
import math
import json


class Navigator:
    def __init__(self, place: str =
                 "California State Polytechnic University Pomona, California, USA",
                 network_type: str = "bike",
                 verbose: bool = True):
        self.verbose = verbose
        if self.verbose:
            print("Downloading network from OpenStreetMap...")

        # Build graph in WGS84 (lat/lon)
        self.G = ox.graph_from_place(place, network_type=network_type, simplify=True)

        # Project to a local metric CRS (UTM or local equal-area chosen by OSMnx)
        if self.verbose:
            print("Projecting graph to a local metric CRS...")
        self.G = ox.project_graph(self.G)
        self.crs = self.G.graph["crs"]  # e.g., 'EPSG:32611' around CPP

        if self.verbose:
            print(f"Map ready: {len(self.G.nodes)} nodes, {len(self.G.edges)} edges")
            print(f"Coordinate system: {self.crs}\n")

        # Transformer: WGS84 (lat/lon) -> projected CRS (meters)
        self.transformer = Transformer.from_crs("EPSG:4326", self.crs, always_xy=True)
        self.transformer_inv = Transformer.from_crs(self.crs, "EPSG:4326", always_xy=True)

        self.route = None
        self.waypoints = None

    # ---------------------------- Routing ----------------------------

    def plan_route(self, start_lat: float, start_lon: float,
                   end_lat: float, end_lon: float):
        if self.verbose:
            print(f"Planning route from ({start_lat:.6f}, {start_lon:.6f})")
            print(f"              to ({end_lat:.6f}, {end_lon:.6f})")

        # Convert GPS (lon,lat) -> projected (x,y) meters
        start_x, start_y = self.transformer.transform(start_lon, start_lat)
        end_x,   end_y   = self.transformer.transform(end_lon,   end_lat)

        # Find nearest nodes in projected space (meters)
        start_node = ox.distance.nearest_nodes(self.G, start_x, start_y)
        end_node   = ox.distance.nearest_nodes(self.G, end_x,   end_y)

        # Check if start and end are the same node
        if start_node == end_node:
            if self.verbose:
                print("WARNING: Start and end map to the same road node.")
                print("         Choose a destination further away.\n")
            return None

        try:
            self.route = nx.shortest_path(self.G, start_node, end_node, weight="length")
            distance = nx.shortest_path_length(self.G, start_node, end_node, weight="length")
            if self.verbose:
                print(f"Route found: {distance:.1f} meters\n")
            return distance
        except nx.NetworkXNoPath:
            if self.verbose:
                print("ERROR: No path found!\n")
            return None

    # --------------------------- Waypoints ---------------------------

    def get_waypoints(self, spacing_meters: float = 5.0):
        """Generate evenly-spaced waypoints (x,y in meters) along the planned route."""
        if self.route is None:
            print("ERROR: Plan a route first!")
            return None

        # Build a detailed polyline (projected meters): nodes + edge geometries
        coords = []
        route_nodes = self.route
        for i, node in enumerate(route_nodes):
            # Node point (already in projected CRS), add it
            coords.append((self.G.nodes[node]["x"], self.G.nodes[node]["y"]))

            # Edge geometry points (skip first/last because nodes are included)
            if i < len(route_nodes) - 1:
                next_node = route_nodes[i + 1]
                data = self.G.get_edge_data(node, next_node)
                if data:
                    # Choose the first edge dict that has 'geometry' (multigraph-safe)
                    for ed in data.values():
                        if "geometry" in ed:
                            coords.extend(list(ed["geometry"].coords)[1:-1])
                            break

        # Path as LineString (projected CRS -> geometric length in meters)
        if len(coords) < 2:
            print("ERROR: Route too short (fewer than 2 points)")
            return None
        path = LineString(coords)
        total_length = path.length  # meters

        # Even spacing in meters using absolute distances (not normalized fractions)
        num_waypoints = int(total_length // spacing_meters) + 1
        self.waypoints = []

        for i in range(num_waypoints):
            dist = min(i * spacing_meters, total_length)
            pt = path.interpolate(dist, normalized=False)
            utm_x, utm_y = pt.x, pt.y

            self.waypoints.append({
                "x": utm_x,  # meters (projected CRS)
                "y": utm_y   # meters (projected CRS)
            })

        if self.verbose:
            print(f"Generated {len(self.waypoints)} waypoints\n")
        return self.waypoints

    def get_path_array(self) -> np.ndarray:
        """Return Nx2 array of [x, y] in meters (projected CRS)."""
        if self.waypoints is None:
            print("ERROR: No waypoints generated. Call get_waypoints() first!")
            return None
        return np.array([[wp["x"], wp["y"]] for wp in self.waypoints], dtype=float)

    # --------------------------- Utilities --------------------------

    def gps_to_utm(self, lat: float, lon: float):
        """Convert GPS lat/lon to projected (x,y) in meters (same CRS as graph)."""
        return self.transformer.transform(lon, lat)

    def utm_to_gps(self, x: float, y: float) -> tuple[float, float]:
        """Convert projected (x,y) meters back to GPS lat/lon."""
        lon, lat = self.transformer_inv.transform(x, y)
        return lat, lon

    def save_waypoints(self, filename: str = "waypoints_utm.json"):
        if self.waypoints is None:
            print("No waypoints to save")
            return
        with open(filename, "w") as f:
            json.dump(self.waypoints, f, indent=2)
        if self.verbose:
            print(f"Saved {len(self.waypoints)} waypoints to {filename}\n")

    def visualize(self):
        """Quick look: draw route + (optional) waypoints in projected meters."""
        if self.route is None:
            print("No route to visualize")
            return
        import matplotlib.pyplot as plt

        fig, ax = ox.plot_graph_route(
            self.G, self.route,
            route_linewidth=3,
            node_size=0,
            bgcolor="white",
            route_color="blue",
            show=False,
            close=False,
        )

        if self.waypoints:
            xs = [w["x"] for w in self.waypoints]
            ys = [w["y"] for w in self.waypoints]
            ax.scatter(xs, ys, s=15, color="red", zorder=5) 

        ax.set_aspect("equal", adjustable="box")
        plt.title("Route with Waypoints (projected meters)")
        plt.xlabel("Easting (m)")
        plt.ylabel("Northing (m)")
        plt.show()

    def print_waypoints(self, num: int = 5):
        if self.waypoints is None:
            print("No waypoints generated")
            return
        print(f"Showing first {num} waypoints (UTM meters):")
        print("-" * 30)
        print(f"{'Easting (m)':>12}  {'Northing (m)':>12}")
        print("-" * 30)
        for wp in self.waypoints[:num]:
            print(f"{wp['x']:12.1f}  {wp['y']:12.1f}")
        if len(self.waypoints) > num:
            print(f"... and {len(self.waypoints) - num} more")
        print()


# ---------------------------- Example -----------------------------

if __name__ == "__main__":
    nav = Navigator(verbose=True)

    # Example start/end (GPS)
    start_lat, start_lon = 34.0591507, -117.8219452
    end_lat,   end_lon   = 34.0580000, -117.8200000

    dist_m = nav.plan_route(start_lat, start_lon, end_lat, end_lon)
    if dist_m:
        nav.get_waypoints(spacing_meters=5.0)
        nav.print_waypoints(num=5)

        path_xy = nav.get_path_array()
        print("Path array shape (x,y meters):", path_xy.shape)

        # Convert GPS -> UTM example
        x, y = nav.gps_to_utm(start_lat, start_lon)
        print(f"Start in UTM meters: ({x:.1f}, {y:.1f})")

        # Optional: save + visualize
        nav.save_waypoints("route_waypoints_utm.json")
        nav.visualize()
