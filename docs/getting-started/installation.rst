============
Installation
============

Complete installation guide for the UTM Navigator software stack.

.. contents:: Contents
   :local:
   :depth: 2

System Requirements
===================

Operating System
----------------

- **Ubuntu 20.04 LTS** (recommended) or Ubuntu 22.04 LTS
- Other Linux distributions may work but are not officially supported

Hardware
--------

+------------------+----------------------------+--------------------------------+
| Component        | Minimum                    | Recommended                    |
+==================+============================+================================+
| CPU              | Intel i5 / AMD Ryzen 5     | Intel i7 / AMD Ryzen 7         |
+------------------+----------------------------+--------------------------------+
| RAM              | 8 GB                       | 16+ GB                         |
+------------------+----------------------------+--------------------------------+
| GPU              | NVIDIA GTX 1050            | NVIDIA GTX 1060+               |
+------------------+----------------------------+--------------------------------+
| Storage          | 50 GB SSD                  | 256+ GB NVMe SSD               |
+------------------+----------------------------+--------------------------------+
| USB Ports        | 3x USB 3.0                 | 4x USB 3.0                     |
+------------------+----------------------------+--------------------------------+
| Ethernet         | 1x Gigabit                 | 1x Gigabit                     |
+------------------+----------------------------+--------------------------------+

Step 1: System Packages
=======================

Update your system and install required packages:

.. code-block:: bash

   sudo apt update && sudo apt upgrade -y

   # Core development tools
   sudo apt install -y build-essential cmake git wget curl

   # Python development
   sudo apt install -y python3-dev python3-pip python3-venv

   # OpenCV dependencies
   sudo apt install -y libopencv-dev python3-opencv

   # Serial communication
   sudo apt install -y libserial-dev

   # Open3D dependencies (for 3D visualization)
   sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev

Step 2: NVIDIA Drivers & CUDA
=============================

For GPU-accelerated perception (YOLOPv2):

.. code-block:: bash

   # Add NVIDIA repository
   sudo add-apt-repository ppa:graphics-drivers/ppa
   sudo apt update

   # Install recommended driver
   sudo ubuntu-drivers autoinstall

   # Reboot
   sudo reboot

After reboot, verify the driver:

.. code-block:: bash

   nvidia-smi

Install CUDA Toolkit:

.. code-block:: bash

   # Install CUDA 11.8 (adjust version as needed)
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
   sudo dpkg -i cuda-keyring_1.0-1_all.deb
   sudo apt update
   sudo apt install -y cuda-toolkit-11-8

   # Add to PATH (add to ~/.bashrc)
   echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
   echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc

Step 3: Python Environment
==========================

Create and activate a virtual environment:

.. code-block:: bash

   cd ~/utm-navigator
   python3 -m venv venv
   source venv/bin/activate

   # Upgrade pip
   pip install --upgrade pip setuptools wheel

Step 4: Python Dependencies
===========================

Install all Python packages:

.. code-block:: bash

   pip install -r requirements.txt

Or install manually:

.. code-block:: bash

   # Core scientific stack
   pip install numpy scipy matplotlib

   # Computer vision
   pip install opencv-python opencv-contrib-python

   # Deep learning (with CUDA support)
   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

   # Navigation and mapping
   pip install osmnx networkx pyproj shapely

   # Visualization
   pip install open3d

   # Hardware interfaces
   pip install pyserial velodyne-decoder pyrealsense2

   # Configuration
   pip install pyyaml

Step 5: Xsens SDK Installation
==============================

The Xsens MTi SDK requires manual installation:

1. Download the MT Software Suite from `Xsens <https://www.xsens.com/software-downloads>`_
2. Extract and run the installer:

.. code-block:: bash

   tar -xzf MT_Software_Suite_linux-x64_2022.0.tar.gz
   cd MT_Software_Suite_linux-x64_2022.0
   sudo ./mtsdk_linux-x64_2022.0.run

3. Install the Python bindings:

.. code-block:: bash

   pip install xsensdeviceapi

4. Add user to dialout group for serial access:

.. code-block:: bash

   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect

Step 6: Verify Installation
===========================

Run the verification script:

.. code-block:: bash

   python scripts/verify_installation.py

Expected output:

.. code-block:: text

   Checking Python version... OK (3.10.12)
   Checking NumPy... OK (1.24.3)
   Checking OpenCV... OK (4.8.0)
   Checking PyTorch... OK (2.0.1+cu118)
   Checking CUDA availability... OK (CUDA 11.8)
   Checking OSMnx... OK (1.5.0)
   Checking Xsens SDK... OK
   Checking Velodyne decoder... OK

   All dependencies installed successfully!

Troubleshooting
===============

CUDA not detected
-----------------

Ensure NVIDIA drivers are installed:

.. code-block:: bash

   nvidia-smi

If the command fails, reinstall drivers and reboot.

Permission denied on serial ports
---------------------------------

Add your user to the dialout group:

.. code-block:: bash

   sudo usermod -a -G dialout $USER

Log out and log back in.

OSMnx import error
------------------

OSMnx requires additional system libraries:

.. code-block:: bash

   sudo apt install -y libspatialindex-dev
   pip install rtree

Next Steps
==========

- :doc:`dependencies` - Detailed dependency reference
- :doc:`first-run` - Running the system for the first time
- :doc:`../configuration/sensor-calibration` - Calibrate your sensors
