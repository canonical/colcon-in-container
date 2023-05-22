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
colcon build-in-container --help
```

By default `build-in-container` uses the ROS version from the `ROS_DISTRO` environment variable. This can be overwritten with the option `--ros-distro`

## Use cases
The colcon `build-in-container` can be used to:
- Build a ROS 2 pacakge before releasing it
- Make sure that your `pacakge.xml` are up to date
- Build a ROS 2 workspace with a ROS 2 version you haven't installed
- Anything you can imagine!

## Troubleshooting
If you have issues with pylxd and openssl:

Pylxd doesn't work from the apt debian or the pip package. You [must be installed from source](https://discuss.linuxcontainers.org/t/5-0-2-raises-connection-reset-by-peer-exception-on-pylxds-container-execute/16292)
