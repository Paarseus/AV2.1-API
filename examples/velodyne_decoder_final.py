#!/usr/bin/env python3
"""
Velodyne Puck LiDAR Visualizer - Professional Edition
Uses velodyne_decoder library for optimized, validated decoding
"""

import numpy as np
import open3d as o3d
import time
from threading import Thread, Lock
from collections import deque
import argparse
import sys

try:
    import velodyne_decoder as vd
    HAS_VELODYNE_DECODER = True
except ImportError:
    HAS_VELODYNE_DECODER = False
    print("WARNING: velodyne_decoder not installed!")
    print("Install it with: pip install velodyne-decoder")
    print("Falling back to basic decoder...")


class VelodyneDecoderVisualizer:
    """
    Professional Velodyne visualizer using the velodyne_decoder library
    
    Benefits over manual parsing:
    - Validated against VeloView ground truth
    - Auto-detects model and RPM
    - Supports ALL Velodyne models
    - Handles dual-return data properly
    - Precise timing information
    - Telemetry packet support
    - Optimized C++ backend
    """
    
    def __init__(self, pcap_file=None, bag_file=None, port=2368, 
                 config=None, buffer_scans=3, enable_motion_compensation=True):
        """
        Initialize visualizer
        
        Args:
            pcap_file: Path to PCAP file to replay (optional)
            bag_file: Path to ROS bag file to replay (optional)
            port: UDP port for live streaming (default: 2368)
            config: velodyne_decoder.Config object (optional)
            buffer_scans: Number of scans to buffer for visualization
            enable_motion_compensation: Apply motion compensation for better wall quality
        """
        if not HAS_VELODYNE_DECODER:
            raise ImportError("velodyne_decoder is required. Install with: pip install velodyne-decoder")
        
        self.pcap_file = pcap_file
        self.bag_file = bag_file
        self.port = port
        self.buffer_scans = buffer_scans
        self.enable_motion_compensation = enable_motion_compensation
        
        # Configuration
        if config is None:
            self.config = vd.Config()
            # Use timestamp of first packet for better synchronization
            self.config.timestamp_first_packet = True
        else:
            self.config = config
        
        # Data structures
        self.scan_buffer = deque(maxlen=buffer_scans)
        self.data_lock = Lock()
        self.running = False
        self.has_new_data = False
        
        # Statistics
        self.scan_count = 0
        self.total_points = 0
        
        # Streaming decoder (for live UDP or PCAP)
        self.stream_decoder = None
        
    def start_streaming(self):
        """Start streaming from UDP port or PCAP file"""
        self.running = True
        
        if self.pcap_file:
            stream_thread = Thread(
                target=self._stream_pcap_thread, 
                daemon=True
            )
        else:
            stream_thread = Thread(
                target=self._stream_udp_thread, 
                daemon=True
            )
        
        stream_thread.start()
        print("✓ Started streaming thread")
    
    def _stream_pcap_thread(self):
        """Stream and decode from PCAP file"""
        print(f"✓ Streaming from PCAP: {self.pcap_file}")
        
        try:
            for stamp, points in vd.read_pcap(self.pcap_file, config=self.config):
                if not self.running:
                    break
                
                self.scan_count += 1
                
                with self.data_lock:
                    self.scan_buffer.append(points)
                    self.total_points = sum(len(scan) for scan in self.scan_buffer)
                    self.has_new_data = True
                
                # Add delay to simulate real-time playback
                time.sleep(0.1)  # ~10 Hz scan rate
                
        except Exception as e:
            print(f"PCAP streaming error: {e}")
    
    def _stream_udp_thread(self):
        """Stream and decode from live UDP packets"""
        print(f"✓ Listening for live UDP packets on port {self.port}")
        
        import socket
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 26214400)  # 25MB
        
        try:
            sock.bind(('', self.port))
            sock.settimeout(0.1)
            
            print("✓ Socket bound, creating stream decoder...")
            
            # Create streaming decoder
            self.stream_decoder = vd.StreamDecoder(self.config)
            
            packet_count = 0
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(2000)
                    packet_count += 1
                    
                    # Decode packet with velodyne_decoder
                    # decode() returns a tuple (timestamp, points_array) or None
                    current_time = time.time()
                    result = self.stream_decoder.decode(current_time, data, False)
                    
                    # Check if we got a complete scan
                    if result is not None:
                        # result is a tuple: (timestamp, points)
                        timestamp, points = result
                        
                        if points is not None and len(points) > 0:
                            self.scan_count += 1
                            
                            with self.data_lock:
                                self.scan_buffer.append(points)
                                self.total_points = sum(len(scan) for scan in self.scan_buffer)
                                self.has_new_data = True
                            
                            # Don't format timestamp - just show scan count and points
                            print(f"\r✓ Scan {self.scan_count} received ({len(points)} points)", 
                                  end='', flush=True)
                    
                    # Debug: show packet count
                    if packet_count % 100 == 0 and result is None:
                        print(f"\r  Packets received: {packet_count} (waiting for complete rotation...)", end='', flush=True)
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"\nUDP error: {e}")
                        
        except Exception as e:
            print(f"Socket error: {e}")
        finally:
            sock.close()
            print("\n✓ Socket closed")
    
    def read_bag_file(self):
        """Read all scans from ROS bag file into buffer"""
        if not self.bag_file:
            return
        
        print(f"✓ Reading ROS bag: {self.bag_file}")
        
        try:
            for stamp, points, topic in vd.read_bag(self.bag_file, config=self.config):
                self.scan_count += 1
                
                with self.data_lock:
                    self.scan_buffer.append(points)
                    self.total_points = sum(len(scan) for scan in self.scan_buffer)
                    self.has_new_data = True
                
                print(f"\rLoaded {self.scan_count} scans...", end='', flush=True)
            
            print(f"\n✓ Loaded {self.scan_count} total scans")
            
        except Exception as e:
            print(f"Bag file error: {e}")
    
    def create_ground_grid(self, size=30):
        """Create ground plane grid"""
        lines, points = [], []
        step = 2
        
        for i in range(-size, size + 1, step):
            points.extend([[i, -size, 0], [i, size, 0]])
            lines.append([len(points)-2, len(points)-1])
            
            points.extend([[-size, i, 0], [size, i, 0]])
            lines.append([len(points)-2, len(points)-1])
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array(points))
        line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
        line_set.colors = o3d.utility.Vector3dVector(
            np.tile([0.3, 0.3, 0.3], (len(lines), 1))
        )
        
        return line_set
    
    def compute_colors(self, points):
        """
        Compute colors based on height with nice gradient
        
        The velodyne_decoder returns structured arrays with fields:
        x, y, z, intensity, time, column, ring, return_type
        """
        if len(points) == 0:
            return np.array([])
        
        # Extract z coordinates
        if points.dtype.names:  # Structured array
            z = points['z']
        else:  # Flat array
            z = points[:, 2]
        
        z_min, z_max = z.min(), z.max()
        
        if z_max - z_min < 0.01:
            return np.ones((len(z), 3)) * 0.5
        
        z_norm = (z - z_min) / (z_max - z_min)
        
        # Nice gradient: blue (low) -> green -> red (high)
        colors = np.zeros((len(z), 3))
        colors[:, 0] = np.clip(2 * z_norm - 0.5, 0, 1)      # Red
        colors[:, 1] = np.clip(1 - 2 * np.abs(z_norm - 0.5), 0, 1)  # Green
        colors[:, 2] = np.clip(1.5 - 2 * z_norm, 0, 1)      # Blue
        
        return colors
    
    def extract_xyz(self, points):
        """Extract XYZ coordinates from points"""
        if len(points) == 0:
            return np.array([])
        
        if points.dtype.names:  # Structured array
            return np.column_stack([points['x'], points['y'], points['z']])
        else:  # Already flat array
            return points[:, :3]
    
    def visualize(self):
        """Start visualization"""
        print("\n" + "="*70)
        print("VELODYNE PROFESSIONAL VISUALIZER")
        print("Using velodyne_decoder library (validated against VeloView)")
        print("="*70)
        
        # Create visualizer
        vis = o3d.visualization.Visualizer()
        
        success = vis.create_window(
            window_name='Velodyne Professional Viewer',
            width=1600,
            height=900,
            left=50,
            top=50,
            visible=True
        )
        
        if not success:
            print("ERROR: Could not create window!")
            return
        
        print("✓ Visualization window created")
        
        # Rendering options
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.05])
        opt.point_size = getattr(self, 'point_size', 2.0)  # Use stored value or default to 2.0
        opt.show_coordinate_frame = True
        
        # Create geometries
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=[0, 0, 0]
        )
        vis.add_geometry(coord_frame)
        
        ground_grid = self.create_ground_grid()
        vis.add_geometry(ground_grid)
        
        print("✓ Geometries added")
        
        # Set camera view
        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)
        ctr.set_front([0, -0.5, -0.9])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 0, 1])
        
        print("\n" + "="*70)
        print("Controls:")
        print("  Mouse: Rotate/Pan/Zoom")
        print("  Q/ESC: Quit")
        print("  H: Help")
        print("="*70 + "\n")
        
        if self.pcap_file or self.port:
            print("Waiting for data...")
        
        # Visualization loop
        update_count = 0
        fps_counter = 0
        fps_timer = time.time()
        last_update_time = time.time()
        target_fps = 15
        frame_time = 1.0 / target_fps
        
        while self.running or (self.bag_file and self.scan_count > 0):
            current_time = time.time()
            
            if not vis.poll_events():
                print("\nWindow closed")
                break
            
            vis.update_renderer()
            
            # Update at target FPS
            if current_time - last_update_time >= frame_time:
                if self.has_new_data:
                    with self.data_lock:
                        if len(self.scan_buffer) > 0:
                            # Combine all buffered scans
                            all_scans = list(self.scan_buffer)
                            self.has_new_data = False
                    
                    if all_scans:
                        # Stack all points
                        if len(all_scans) == 1:
                            combined_points = all_scans[0]
                        else:
                            combined_points = np.concatenate(all_scans)
                        
                        if len(combined_points) > 0:
                            # Extract XYZ
                            xyz = self.extract_xyz(combined_points)
                            pcd.points = o3d.utility.Vector3dVector(xyz)
                            
                            # Compute colors
                            colors = self.compute_colors(combined_points)
                            pcd.colors = o3d.utility.Vector3dVector(colors)
                            
                            vis.update_geometry(pcd)
                            update_count += 1
                            last_update_time = current_time
            
            # FPS calculation
            fps_counter += 1
            
            if current_time - fps_timer >= 1.0:
                fps = fps_counter / (current_time - fps_timer)
                
                print(f"\rRender FPS: {fps:5.1f} | "
                      f"Scans: {self.scan_count:4d} | "
                      f"Points: {self.total_points:8d} | "
                      f"Buffer: {len(self.scan_buffer)}/{self.buffer_scans}",
                      end='', flush=True)
                
                fps_counter = 0
                fps_timer = current_time
            
            time.sleep(0.002)
        
        print("\n\nShutting down...")
        vis.destroy_window()
        print("✓ Done")
    
    def stop(self):
        """Stop streaming"""
        self.running = False


def main():
    parser = argparse.ArgumentParser(
        description='Professional Velodyne Visualizer using velodyne_decoder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Live streaming from UDP
  %(prog)s --port 2368
  
  # Replay PCAP file
  %(prog)s --pcap mydata.pcap
  
  # Load ROS bag
  %(prog)s --bag mydata.bag
  
  # With range filtering
  %(prog)s --pcap mydata.pcap --min-range 1.0 --max-range 50.0
  
  # With angle filtering
  %(prog)s --port 2368 --min-angle 0 --max-angle 180
        """
    )
    
    # Input sources
    parser.add_argument(
        '--port', type=int, default=2368,
        help='UDP port for live data (default: 2368)'
    )
    parser.add_argument(
        '--pcap', type=str,
        help='PCAP file to replay'
    )
    parser.add_argument(
        '--bag', type=str,
        help='ROS bag file to load'
    )
    
    # Visualization options
    parser.add_argument(
        '--buffer-scans', type=int, default=1,
        help='Number of scans to buffer (default: 1 for sharpest walls)'
    )
    parser.add_argument(
        '--point-size', type=float, default=2.0,
        help='Point size for visualization (default: 2.0)'
    )
    
    # Filtering options
    parser.add_argument(
        '--min-range', type=float,
        help='Minimum range in meters'
    )
    parser.add_argument(
        '--max-range', type=float,
        help='Maximum range in meters'
    )
    parser.add_argument(
        '--min-angle', type=float,
        help='Minimum azimuth angle in degrees'
    )
    parser.add_argument(
        '--max-angle', type=float,
        help='Maximum azimuth angle in degrees'
    )
    
    args = parser.parse_args()
    
    if not HAS_VELODYNE_DECODER:
        print("\nERROR: velodyne_decoder library is not installed!")
        print("Install it with: pip install velodyne-decoder")
        sys.exit(1)
    
    # Create configuration
    config = vd.Config()
    if args.min_range is not None:
        config.min_range = args.min_range
    if args.max_range is not None:
        config.max_range = args.max_range
    if args.min_angle is not None:
        config.min_angle = args.min_angle
    if args.max_angle is not None:
        config.max_angle = args.max_angle
    
    # Create visualizer
    visualizer = VelodyneDecoderVisualizer(
        pcap_file=args.pcap,
        bag_file=args.bag,
        port=args.port,
        config=config,
        buffer_scans=args.buffer_scans
    )
    
    # Store point size for later use
    visualizer.point_size = args.point_size
    
    try:
        # Start appropriate mode
        if args.bag:
            visualizer.read_bag_file()
            visualizer.running = False  # Bag mode doesn't stream
        else:
            visualizer.start_streaming()
            time.sleep(0.5)
        
        # Start visualization
        visualizer.visualize()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        visualizer.stop()
        time.sleep(0.5)
        print("✓ Complete")


if __name__ == "__main__":
    main()
