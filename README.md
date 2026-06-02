<div align="center">

# Juno Augmented Navigation

**Autonomous driving & augmented navigation stack for the Shell Eco-marathon**
Developed by **[H2politO](https://areeweb.polito.it/didattica/h2polito/)** — Politecnico di Torino

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![Build](https://img.shields.io/badge/build-colcon-blue)](https://colcon.readthedocs.io/)

</div>

---

## Overview

**Juno Augmented Navigation** is a ROS 2-based software stack that powers the autonomous
navigation system of the H2politO Shell Eco-marathon vehicle. It fuses camera-based
perception, road segmentation, and GNSS localization into a planning and control pipeline
that drives steering and braking actuators in real time.

The system perceives the environment with a **Stereolabs ZED 2i** camera, detects stop
signs with a **YOLOv12m** object detector, segments the drivable road with **TwinLiteNet+**,
projects the scene into a **bird's-eye view (BEV)** for planning, and commands **Trinamic
TMCL** stepper motors for steering and braking over a CAN bus. **u-blox** GNSS receivers
provide global positioning.

---

## Features

| Subsystem | Description |
| --- | --- |
| **Stop-sign detection** | Real-time detection with a YOLOv12m model (Ultralytics) running on ZED 2i RGB frames via OpenCV. |
| **Road segmentation** | Drivable-area / lane segmentation using TwinLiteNet+. |
| **Bird's-eye view (BEV)** | Perspective transform of the segmented scene into a top-down view for geometric reasoning. |
| **Path planning & obstacle avoidance** | Trajectory generation and Nav2-based obstacle avoidance built on the BEV segmentation output. |
| **GNSS localization** | Global positioning from u-blox GPS receivers. |
| **Steering & braking actuation** | Trinamic TMCL stepper motors controlled over CAN (`python-can` + `pytrinamic`). |
| **Modular ROS 2 nodes** | Perception, planning, and control implemented as independent, launch-configurable ROS 2 nodes. |

---

## Supported Environment

| Component | Version |
| --- | --- |
| Ubuntu | 22.04 LTS (Jammy Jellyfish) |
| ROS 2 | Humble Hawksbill |
| Python | 3.10.12 |
| Build system | colcon |
| Compute platform | NVIDIA Jetson |

> **Note:** Use Ubuntu 22.04 LTS to match the supported ROS 2 Humble environment.

---

## Installation

Follow the steps **in order**. The commands assume an existing ROS 2 workspace (`ros2_ws`).

### 1. Clone the repository

```bash
git clone https://github.com/AutonomousDrivingH2politO/AD-SEM
```

### 2. Install Python dependencies

From the repository root:

```bash
pip install -r requirements.txt
```

> **PyTorch / CUDA:** If you need CUDA-enabled PyTorch, install `torch` and `torchvision`
> using the command from the [official PyTorch selector](https://pytorch.org/get-started/locally/)
> instead of the generic CPU wheels.

> **Jetson:** On NVIDIA Jetson, install `torch` and `torchvision` from NVIDIA's prebuilt
> Jetson wheels matching your JetPack version, not the generic PyPI wheels (which are not
> CUDA-enabled for Jetson).

### 3. Install ROS 2 dependencies

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
# From inside the repository folder:
rosdep install --from-paths .. --ignore-src -r -y
# Or, from the workspace root:
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Install the ZED SDK Python API

The ZED 2i camera requires the **ZED SDK** and its Python API (`pyzed.sl`), which are **not**
available on PyPI. Install them separately by following the
[Stereolabs documentation](https://www.stereolabs.com/docs/). This enables:

```python
import pyzed.sl as sl
```

### 5. Clone external repositories

Clone the following packages into your workspace `src/` directory at the pinned commits:

| Package | Repository |
| --- | --- |
| TwinLiteNetPlus | [chequanghuy/TwinLiteNetPlus @ `e052192`](https://github.com/chequanghuy/TwinLiteNetPlus/tree/e0521920e110b374ae5f5343020ef2fc09bfa0eb) |
| ublox | [AutonomousDrivingH2politO/ublox @ `6659731`](https://github.com/AutonomousDrivingH2politO/ublox/tree/665973146687b5e3a9fc85b48e60ec27c1c64202) |
| zed-ros2-examples | [stereolabs/zed-ros2-examples @ `3ee3f69`](https://github.com/stereolabs/zed-ros2-examples/tree/3ee3f69449b93413cb8f201e97fbe0bb19554709) |
| zed-ros2-wrapper | [AutonomousDrivingH2politO/zed-ros2-wrapper @ `21f67d7`](https://github.com/AutonomousDrivingH2politO/zed-ros2-wrapper/tree/21f67d717e58987af8baf72f0a60159836b34a9e) |
| ntrip_client | [AutonomousDrivingH2politO/ntrip_client @ `ca4de0d`](https://github.com/AutonomousDrivingH2politO/ntrip_client/tree/ca4de0dad0fae12fa6d8e9856a993eec5007ce89) |
| rtabmap | [introlab/rtabmap @ `d11aa0a`](https://github.com/introlab/rtabmap/tree/d11aa0a88149a2956973ef3c74cc98183a5c2767) |
| rtabmap_ros | [AutonomousDrivingH2politO/rtabmap_ros @ `ede1acb`](https://github.com/AutonomousDrivingH2politO/rtabmap_ros/tree/ede1acb90a62823b2e1092867db7577b72ba2e7c) |

### 6. Build the workspace

```bash
cd ~/AD-SEM
colcon build                              # build everything
colcon build --packages-select <package>  # build a single package
```

### 7. Source the workspace

```bash
source install/setup.bash
```

> Re-source ROS 2 and the workspace (`setup.bash`) in every new terminal before running commands.

---

## Usage

Launch a package:

```bash
ros2 launch <package_name> <launch_file>.launch.py
```

Run a single node:

```bash
ros2 run <package_name> <node_name>
```

> Replace `<package_name>`, `<launch_file>`, and `<node_name>` with the actual targets from this workspace.

---

## Configuration

Before running the full pipeline, make sure the hardware interfaces are configured:

- **ZED 2i camera** — Install the ZED SDK and verify the camera is detected
  (e.g. `ZED_Explorer`). Camera and depth parameters are exposed through the
  `zed-ros2-wrapper` configuration files.
- **CAN bus (TMCL steppers)** — Bring up the CAN interface used by the steering and braking
  motors, for example:
  ```bash
  sudo ip link set <can_interface> up type can bitrate 500000
  ```
  Replace `<can_interface>` (e.g. `can0`) and the bitrate with the values matching your setup.
- **u-blox GNSS** — Confirm the receiver's serial port and baud rate match the `ublox`
  package configuration. NTRIP corrections are handled by `ntrip_client`.
- **Model weights** — Place the YOLOv12m and TwinLiteNet+ weights where the perception
  nodes expect them.
- **Jetson GPIO** — The system runs on an NVIDIA Jetson; install `Jetson.GPIO` on the board
  for the GPIO-driven peripherals (not needed on a development workstation).

> Hardware-specific parameters (CAN channel, serial ports, model paths, ROS parameters)
> depend on your vehicle setup.

---

## Contributing

Contributions from team members and collaborators are welcome.

1. Create a feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
2. Make your changes and ensure the workspace still builds:
   ```bash
   colcon build
   ```
3. Use clear, descriptive commit messages.
4. Open a Pull Request describing **what** changed and **why**.

For major changes, architecture updates, or project-specific questions, contact your manager
or the project maintainer before starting work.

---

## Contact

- **Website:** [H2politO](https://areeweb.polito.it/didattica/h2polito/)
- **LinkedIn:** [team-h2polito](https://www.linkedin.com/company/team-h2polito/)
- **Instagram:** [@h2polito](https://www.instagram.com/h2polito/)
- **Email:** guidaautonomah2polito@gmail.com