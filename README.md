# px4_ros_project

A software-in-the-loop (SITL) simulation of an autonomous quadrotor flown by a
custom Guidance, Navigation and Control stack written in ROS 2, running with
PX4 and Gazebo.

The goal is to progressively take ownership of the control cascade from PX4 —
starting with PX4 flying itself and ending with this stack producing thrust and
torque commands directly, leaving PX4 responsible only for control allocation
and the safety state machine.

**Status:** Under Construction.
 
## Repository layout

This repository is a colcon workspace. Clone it and build in place.

```
px4_ros_project/
├── src/
│   ├── gnc_bringup/     launch files, params, world config
│   └── gnc_offboard/    offboard control node
├── tools/               log analysis and plotting (plain Python, not ROS)
├── docs/                derivations, plots, design decisions
└── deps.repos           pins px4_msgs to a known commit
```

## Prerequisites

Developed and tested on:

| Component | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS (WSL2) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (`gz-sim` 8.14.0) |
| PX4-Autopilot | `v1.18.0-beta1-34-g5f63c0698e` |
| Micro XRCE-DDS Agent | any recent release, on `PATH` as `MicroXRCEAgent` |

ROS 2 Jazzy and Gazebo Harmonic are the pairing PX4 v1.16+ expects. Substituting
either is likely to require changes.

### PX4-Autopilot

PX4 is an external dependency and is **not** vendored here — its `.git` is over
1.5 GB with 35 submodules, and it uses its own build system rather than colcon.

Clone and build it separately:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
git checkout 5f63c0698e9cc839b7bc35a5532b88d3fd944583
make submodulesclean
make px4_sitl
```

### Micro XRCE-DDS Agent

The bridge between PX4's uXRCE-DDS client and the ROS 2 DDS graph. Install per
the [PX4 documentation](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
so that `MicroXRCEAgent` is on your `PATH`. The launch file starts it on UDP
port 8888, matching PX4's client default.

### Python tooling

Used by the log analysis scripts in `tools/`, outside the ROS build:

```bash
pip install --user pyulog matplotlib numpy pandas
```

## Build

```bash
git clone https://github.com/maxh768/px4_ros_project.git
cd px4_ros_project
vcs import src < deps.repos          # fetches pinned px4_msgs
colcon build --symlink-install
source install/setup.bash
```

`vcs` comes from `python3-vcstool` (`sudo apt install python3-vcstool`).
`--symlink-install` lets launch files and Python nodes be edited without
rebuilding.

Remaining system dependencies can be resolved with:

```bash
rosdep install --from-paths src --ignore-src -y
```

## License

MIT. See `LICENSE` in each package.