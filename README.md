# humanoid_mk1

## Overview

`humanoid_mk1` is a humanoid robotics development repository that combines ROS2-based control software, simulation assets, and project documentation in a single workspace.

## Hardware

- Humanoid robot platform
- Motor control interfaces
- Embedded firmware targets
- Sensors and bring-up peripherals

## Software stack

- ROS2 Humble
- Isaac Sim 5.1.0
- C++
- Python

## Repository structure

- `control/`: Control-related resources and utilities
- `docker/`: Container environment setup
- `docs/`: Project documentation and development notes
- `experiments/`: Experimental code and prototypes
- `firmware/`: Embedded and low-level firmware sources
- `ros2_ws/`: ROS2 workspace
- `simulation/`: Simulation assets, worlds, robots, and tasks
- `tools/`: Helper scripts and tooling

## Development workflow

1. Add or update robot code under `ros2_ws/src/`.
2. Keep simulation assets organized under `simulation/`.
3. Record project decisions and hardware details under `docs/`.
4. Build and test ROS2 packages from `ros2_ws/`.
5. Track experimental work separately before merging it into stable packages.
