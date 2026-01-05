============
Dependencies
============

Complete reference of all software dependencies for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Python Version
==============

- **Required**: Python 3.8+
- **Recommended**: Python 3.10

Core Dependencies
=================

Scientific Computing
--------------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - numpy
     - >= 1.20.0
     - Numerical arrays and mathematical operations
   * - scipy
     - >= 1.7.0
     - Signal processing, interpolation, optimization
   * - matplotlib
     - >= 3.4.0
     - 2D plotting and visualization

Computer Vision
---------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - opencv-python
     - >= 4.5.0
     - Image processing, camera interfaces
   * - opencv-contrib-python
     - >= 4.5.0
     - Additional OpenCV modules

Deep Learning
-------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - torch
     - >= 1.9.0
     - PyTorch deep learning framework
   * - torchvision
     - >= 0.10.0
     - Computer vision models and transforms

.. note::
   Install PyTorch with CUDA support for GPU acceleration:

   .. code-block:: bash

      pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

Navigation & Mapping
--------------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - osmnx
     - >= 1.1.0
     - OpenStreetMap data retrieval and routing
   * - networkx
     - >= 2.6.0
     - Graph algorithms for path planning
   * - pyproj
     - >= 3.0.0
     - Coordinate system transformations (WGS84 ↔ UTM)
   * - shapely
     - >= 1.8.0
     - Geometric operations and collision detection

3D Visualization
----------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - open3d
     - >= 0.13.0
     - 3D point cloud visualization and processing

Hardware Interfaces
-------------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - pyserial
     - >= 3.5
     - Serial communication (Teensy, Xsens)
   * - velodyne-decoder
     - >= 2.0.0
     - Velodyne LIDAR packet parsing
   * - xsensdeviceapi
     - (SDK)
     - Xsens MTi sensor interface
   * - pyrealsense2
     - >= 2.50.0
     - Intel RealSense camera interface

Configuration
-------------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Package
     - Version
     - Purpose
   * - pyyaml
     - >= 5.4.0
     - YAML configuration file parsing

requirements.txt
================

.. code-block:: text

   # UTM Navigator Dependencies
   # Install with: pip install -r requirements.txt

   # Core scientific
   numpy>=1.20.0
   scipy>=1.7.0
   matplotlib>=3.4.0

   # Computer vision
   opencv-python>=4.5.0
   opencv-contrib-python>=4.5.0

   # Deep learning (install with CUDA separately if needed)
   # torch>=1.9.0
   # torchvision>=0.10.0

   # Navigation
   osmnx>=1.1.0
   networkx>=2.6.0
   pyproj>=3.0.0
   shapely>=1.8.0

   # Visualization
   open3d>=0.13.0

   # Hardware
   pyserial>=3.5
   velodyne-decoder>=2.0.0
   pyrealsense2>=2.50.0

   # Configuration
   pyyaml>=5.4.0

System Dependencies
===================

These must be installed via apt (Ubuntu/Debian):

.. code-block:: bash

   # Build tools
   sudo apt install build-essential cmake git

   # Python development
   sudo apt install python3-dev python3-pip python3-venv

   # OpenCV system libraries
   sudo apt install libopencv-dev

   # Serial port access
   sudo apt install libserial-dev

   # OpenGL (for Open3D)
   sudo apt install libgl1-mesa-dev libglu1-mesa-dev

   # Spatial indexing (for OSMnx/Shapely)
   sudo apt install libspatialindex-dev

Optional Dependencies
=====================

For development and testing:

.. code-block:: bash

   pip install pytest pytest-cov    # Testing
   pip install black flake8         # Code formatting
   pip install sphinx sphinx-rtd-theme  # Documentation
