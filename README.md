# colcon-build-in-container

colcon verb extension to build everything inside a container and transfert the restults back to the host

## How it works

`colcon build-in-container` performs the following steps to build your workspace:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Upload your workspace inside the container
- Install all your rosdep dependencies (make sure to keep your `package.xml` updated!)
- Build you workspace inside the container
- Download the `install/` directory of the built workspace on your host under: `colcon-build-in-container/`

## Usage

Basic usage:
```
colcon build-in-container
```

Advanced usage:
```
colcon build-in-container --log-level=info --ros-distro humble --colcon-build-args "--cmake-args -DCMAKE_BUILD_TYPE=Release" --debug
```

Usage help:
```
$ colcon build-in-container --help

usage: colcon build-in-container [-h] [--ros-distro ROS_DISTRO] [--colcon-build-args *] [--debug] [--paths [PATH [PATH ...]]]

call a colcon command inside a fresh container.

optional arguments:
  -h, --help            show this help message and exit
  --ros-distro ROS_DISTRO
                        ROS version, can also be set by the environment variable ROS_DISTRO.
  --colcon-build-args *
                        Pass arguments to the colcon build command
  --debug               Shell into the environment at the end of the build or if there is an error

Discovery arguments:
  --paths [PATH [PATH ...]]
                        The paths to check for a package. Use shell wildcards (e.g. `src/*`) to select all direct subdirectories (default: .)
```

By default, `build-in-container` uses the ROS version from the `ROS_DISTRO` environment variable.
This can be overwritten with the option `--ros-distro` allowing one to compile for a different ROS distribution than the one associated with the host OS.

## Use cases
The colcon `build-in-container` can be used to:
- Build a ROS 2 package in a clean environment before releasing it
- Build a ROS 2 package for a different ROS distribution
- Make sure that your `package.xml` are up to date
- Build a ROS 2 workspace with a ROS 2 version you haven't installed
- And more!

## Troubleshooting
If you have issues with pylxd and openssl:

Pylxd doesn't work from the apt debian or the pip package. You [must be installed from source](https://discuss.linuxcontainers.org/t/5-0-2-raises-connection-reset-by-peer-exception-on-pylxds-container-execute/16292)
