# AMP Autonomous Software Stack v2

[![build checks](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/ci.yaml/badge.svg)](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/ci.yaml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/Autonomous-Motorsports-Purdue/AMP_ASSv2/master.svg)](https://results.pre-commit.ci/latest/github/Autonomous-Motorsports-Purdue/AMP_ASSv2/master)

Second iteration of the Autonomous Software Stack (ASS) for the AMP go-kart.
Go to the [wiki](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/wiki)
to learn more about this repo and code standards.

It is dependent on [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Developing

The best way to work with the codebase is within a Ubuntu 20.04 environment, be
it on baremetal or on a VM. To do so, clone this repo and install all the
dependencies and build the project.

An alternate build system,
`[colcon](https://colcon.readthedocs.io/en/released/user/installation.html)`
can also be used.

```bash
# Install dependencies
rosdep update
rosdep install --from-paths src -iry

# Build the packages
catkin_make

# If using colcon, use the line below
# colcon build
```

## Formatting

In order to run the `pre-commit` formatter, install `pre-commit` (via [PyPI](https://pypi.org/project/pre-commit/)) and `clang` (via your distro's package manager) on your Linux Machine and run:

```
pre-commit install
```

This will make it so that a format check is executed whenever you attempt to commit changes.

In order to pass the formatting check, run the following before every commit:

```
pre-commit run -a
```

The formatter was borrowed from [MoveIt's ROS Repo](https://github.com/ros-planning/moveit2).
