# LQR Path Tracking Controller for F1TENTH Autonomous Racing

**Author:** Mohamed Elgouhary
**Affiliation:** iCPS Lab, West Virginia University
**Project Type:** ROS 2 Autonomous Racing Controller

This repository contains a ROS 2 Python implementation of a **Linear Quadratic Regulator (LQR)** controller for path tracking in F1TENTH-style autonomous racing.

The controller follows a precomputed raceline using a lateral kinematic vehicle model. It computes steering and speed commands based on lateral error, heading error, velocity error, and local track curvature.

## Overview

Linear Quadratic Regulator control is a classical optimal-control method that is widely used for stabilizing dynamic systems. In autonomous racing, LQR can be used as a model-based path-tracking controller that balances tracking accuracy and control effort.

This repository provides an LQR-based controller that:

* Loads raceline waypoints from CSV files
* Computes lateral and heading errors relative to the reference path
* Uses a discrete-time LQR formulation for steering and speed control
* Publishes Ackermann drive commands
* Supports simulation and real-car topic configurations
* Publishes visualization markers for RViz
* Includes multiple raceline CSV files for different racing tracks

## Main Features

* ROS 2 Python implementation
* F1TENTH-compatible Ackermann drive output
* Lateral kinematic vehicle model
* Riccati-equation-based LQR gain computation
* Raceline tracking using nearest-waypoint projection
* Speed tracking based on reference raceline velocity
* RViz visualization of waypoints and vehicle front-axle point
* Support for multiple track CSV files

## Repository Structure

```text
LQR/
├── README.md
└── src/
    ├── csv_data/
    │   ├── Austin.csv
    │   ├── Austin_fast.csv
    │   ├── Catalunya_fast.csv
    │   ├── Montreal_fast.csv
    │   ├── Monza_fast.csv
    │   ├── Spa_fast.csv
    │   ├── YasMarina_fast.csv
    │   └── ...
    └── lqr_pkg/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        ├── test/
        └── lqr_pkg/
            ├── __init__.py
            ├── lqr.py
            └── utils.py
```

## Main Files

| File                           | Description                                                       |
| ------------------------------ | ----------------------------------------------------------------- |
| `src/lqr_pkg/lqr_pkg/lqr.py`   | Main ROS 2 LQR controller node                                    |
| `src/lqr_pkg/lqr_pkg/utils.py` | Utility functions for waypoint processing and angle normalization |
| `src/lqr_pkg/setup.py`         | ROS 2 Python package setup file                                   |
| `src/lqr_pkg/package.xml`      | ROS 2 package metadata and dependencies                           |
| `src/csv_data/`                | Raceline CSV files for different tracks                           |

## Controller Description

The controller uses a lateral kinematic vehicle model with the following state-error representation:

```text
x = [e_l, e_l_dot, e_theta, e_theta_dot, e_v]
```

where:

* `e_l` is the lateral error
* `e_l_dot` is the lateral-error rate
* `e_theta` is the heading error
* `e_theta_dot` is the heading-error rate
* `e_v` is the velocity error

The LQR controller computes feedback commands using a discrete-time state-space model:

```text
u = Kx
```

where the control output includes:

* Steering correction
* Speed correction

The final steering command combines:

* LQR feedback steering
* Curvature-based feedforward steering

This allows the vehicle to track the raceline while accounting for the local path curvature.

## ROS 2 Topics

The controller uses the following topics by default:

| Type   | Topic                         | Description                                    |
| ------ | ----------------------------- | ---------------------------------------------- |
| Input  | `/ego_racecar/odom`           | Vehicle odometry in simulation                 |
| Input  | `/pf/viz/inferred_pose`       | Estimated pose for real-car mode               |
| Input  | `/pf/pose/odom`               | Particle-filter odometry for real-car velocity |
| Output | `/drive`                      | Ackermann steering and speed command           |
| Output | `/visualization_marker_array` | RViz visualization markers                     |

## Dependencies

This package expects a ROS 2 environment with the following dependencies:

* `rclpy`
* `std_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `nav_msgs`
* `ackermann_msgs`
* `visualization_msgs`
* `tf2_ros`

Python dependencies include:

* `numpy`
* `scipy`

Depending on your setup, the package may also use:

* `cvxpy`
* `numba`

## Installation

Clone the repository into the `src` folder of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ICPS-LAB-WVU/LQR.git
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the Controller

After building and sourcing the workspace, run:

```bash
ros2 run lqr_pkg lqr
```

You should see output similar to:

```text
Lateral Kinematic LQR Initialized
```

The node will subscribe to odometry, compute LQR-based steering and speed commands, and publish them to:

```text
/drive
```

## Using a Different Track

The controller loads raceline CSV files from:

```text
src/csv_data/
```

The default map name in the code is:

```text
Montreal_fast
```

This means the controller expects:

```text
src/csv_data/Montreal_fast.csv
```

To use another track, modify the `map_name` variable in `lqr.py`, for example:

```python
self.map_name = 'Catalunya_fast'
```

Available track files include examples such as:

```text
Austin_fast.csv
Catalunya_fast.csv
Hockenheim_fast.csv
Melbourne_fast.csv
MexicoCity_fast.csv
Montreal_fast.csv
Monza_fast.csv
Spa_fast.csv
YasMarina_fast.csv
```

## Simulation vs Real Vehicle

The controller includes a flag for simulation or real-car operation:

```python
self.is_real = False
```

For simulation, the controller uses:

```text
/ego_racecar/odom
```

For real-car operation, the controller can use:

```text
/pf/viz/inferred_pose
/pf/pose/odom
```

Before running on a real vehicle, carefully verify:

* Topic names
* Vehicle coordinate frames
* Wheelbase
* Speed limits
* Steering limits
* Raceline scaling
* Odometry direction
* Emergency stop behavior

## Suggested Workflow

1. Start the F1TENTH simulator or real-car localization stack.
2. Confirm that odometry or pose messages are being published.
3. Select the desired raceline CSV file.
4. Build and source the ROS 2 workspace.
5. Run the LQR controller.
6. Visualize waypoints and vehicle tracking behavior in RViz.
7. Tune the LQR cost matrices if needed.
8. Test at low speed before increasing the velocity profile.

## Parameters to Tune

Important parameters inside the controller include:

| Parameter   | Description                 |
| ----------- | --------------------------- |
| `dt`        | Controller time step        |
| `wheelbase` | Vehicle wheelbase           |
| `Q`         | State-error cost matrix     |
| `R`         | Control-effort cost matrix  |
| `map_name`  | Selected raceline CSV file  |
| `is_real`   | Simulation or real-car mode |

The LQR cost matrices are defined in the `LQRSolver` class:

```python
self.Q = np.diag([1.5, 0.5, 1.0, 0.5, 0.5])
self.R = np.diag([1, 1])
```

Increasing values in `Q` generally makes the controller more aggressive in reducing tracking error. Increasing values in `R` generally penalizes control effort and can make the controller smoother.

## Notes on CSV Format

The raceline CSV files are expected to contain waypoint information used by the controller, including:

* x-position
* y-position
* heading
* curvature
* reference velocity

The current implementation reads waypoint columns directly from the CSV file. If you use a custom raceline, make sure the column order matches the format expected in `lqr.py`.

## Safety Notes

This controller is intended for research and development. Before using it on physical hardware:

* Test first in simulation
* Start with low speeds
* Use an emergency stop
* Confirm steering direction
* Confirm odometry direction
* Verify the raceline is aligned with the map
* Check all ROS topic names
* Avoid running near people or obstacles during initial testing


## Author

This repository is developed and maintained by:

**Mohamed Elgouhary**
PhD Student and Graduate Research Assistant
Lane Department of Computer Science and Electrical Engineering
West Virginia University

## Acknowledgment

This repository is associated with autonomous vehicle and F1TENTH research at the iCPS Lab, West Virginia University.


## Contact

For questions or collaboration, please contact:

**Mohamed Elgouhary**
West Virginia University
Email: [mae00018@mix.wvu.edu](mailto:mae00018@mix.wvu.edu)
