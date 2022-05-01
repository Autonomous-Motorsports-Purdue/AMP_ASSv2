# AMP Autonomous Software Stack v2
[![AMP-CLI CI](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/cli.yml/badge.svg)](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/cli.yml)     [![AMP_ASSv2 CI](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/ci.yml/badge.svg)](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/actions/workflows/ci.yml)

Second iteration of the Autonomous Software Stack (ASS) for the AMP go-kart.
Go to the [wiki](https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv2/wiki) to learn more about this repo and code standards.

# Developing
For most people, it is recommended to work with the code base from within a virtual ROS environment in a docker container. To do so, the `amp-cli` will help you set up all you'll need.
* To build the amp-cli tool, run:
```
sudo pip install .
```
* Run `amp-cli` to view the available options.
Run the `devel` option in order to have a good consistent environment.

# Formatting
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
