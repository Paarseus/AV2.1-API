===============
Troubleshooting
===============

Common issues and solutions for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 1

Quick Reference
===============

+---------------------------+----------------------------------+
| Symptom                   | First Check                      |
+===========================+==================================+
| No GPS position           | Antenna sky view, wait for fix   |
+---------------------------+----------------------------------+
| LIDAR no data             | Ethernet connection, IP config   |
+---------------------------+----------------------------------+
| Vehicle doesn't respond   | Teensy connection, drive mode    |
+---------------------------+----------------------------------+
| Steering oscillates       | Reduce controller gain           |
+---------------------------+----------------------------------+
| Cuts corners              | Increase lookahead distance      |
+---------------------------+----------------------------------+

Detailed Guides
===============

.. toctree::
   :maxdepth: 2

   common-issues
   sensor-debugging
