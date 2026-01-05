================
Sensor Debugging
================

Tools and techniques for debugging sensor issues.

.. contents:: Contents
   :local:
   :depth: 2

GPS/IMU Debugging
=================

Check Connection
----------------

.. code-block:: bash

   # List serial devices
   ls /dev/ttyUSB*

   # Check device permissions
   ls -la /dev/ttyUSB0

View Raw Data
-------------

.. code-block:: python

   from xsens_class import XsensReceiver
   gps = XsensReceiver()

   while True:
       print(f"Pos: {gps.get_position()}")
       print(f"RTK: {gps.get_rtk_status()}")
       time.sleep(1)

LIDAR Debugging
===============

Network Check
-------------

.. code-block:: bash

   # Check connectivity
   ping 192.168.1.201

   # Check UDP traffic
   sudo tcpdump -i eth0 port 2368

View Point Cloud
----------------

.. code-block:: python

   from sensors.lidar_interface import VelodyneLIDAR
   import open3d as o3d

   lidar = VelodyneLIDAR()
   points = lidar.get_scan()

   pcd = o3d.geometry.PointCloud()
   pcd.points = o3d.utility.Vector3dVector(points)
   o3d.visualization.draw_geometries([pcd])

Camera Debugging
================

List Cameras
------------

.. code-block:: bash

   v4l2-ctl --list-devices

Test Capture
------------

.. code-block:: python

   import cv2

   cap = cv2.VideoCapture(0)
   ret, frame = cap.read()
   cv2.imwrite('test_capture.jpg', frame)
   cap.release()
