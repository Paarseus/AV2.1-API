=============
Custom Routes
=============

Create and use custom waypoint routes.

.. contents:: Contents
   :local:
   :depth: 2

Creating Waypoint Files
=======================

CSV Format
----------

Create a CSV file with waypoints:

.. code-block:: text

   # waypoints.csv
   # lat,lon
   34.0577,-117.8215
   34.0578,-117.8212
   34.0580,-117.8210
   34.0582,-117.8208

UTM Format
----------

Or use UTM coordinates directly:

.. code-block:: text

   # waypoints_utm.csv
   # x,y
   427859.2,3769432.1
   427865.5,3769440.3
   427872.1,3769448.7

Loading Custom Routes
=====================

.. code-block:: python

   import csv

   def load_waypoints(filename):
       waypoints = []
       with open(filename) as f:
           reader = csv.reader(f)
           next(reader)  # Skip header
           for row in reader:
               waypoints.append((float(row[0]), float(row[1])))
       return waypoints

   # Use in runner
   waypoints = load_waypoints('waypoints_utm.csv')
   runner.set_waypoints(waypoints)

Recording Routes
================

Record a driven path for later replay:

.. code-block:: python

   class RouteRecorder:
       def __init__(self, filename):
           self.file = open(filename, 'w')
           self.writer = csv.writer(self.file)
           self.writer.writerow(['lat', 'lon'])

       def record(self, lat, lon):
           self.writer.writerow([lat, lon])

       def close(self):
           self.file.close()
