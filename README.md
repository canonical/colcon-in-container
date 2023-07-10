# colcon-in-container

colcon verb extension to build and test inside a container and transfert the restults back to the host

## How it works

### colcon build-in-container

`colcon build-in-container` performs the following steps to build your workspace:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Upload your workspace inside the container
- Install all your rosdep dependencies (make sure to keep your `package.xml` updated!)
- Build you workspace inside the container
- Download the `build/` and `install/` directories of the built workspace on your host under: `build_in_container` and `install_in_container/`

### colcon test-in-container

`colcon test-in-container` performs the following steps to build your workspace:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Upload your workspace as well as your `build_in_container` and `install_in_container/` directories inside the container
- Install all your rosdep dependencies (make sure to keep your `package.xml` updated!)
- Test you workspace inside the container
- Download the test results directory of the built workspace on your host under: `test_in_container/`

## Usage

### colcon build-in-container

Basic usage:
```
colcon build-in-container
```

Advanced usage:
```
colcon --log-level=info build-in-container --ros-distro humble --colcon-build-args "--cmake-args -DCMAKE_BUILD_TYPE=Release" --debug
```

Usage help:
```
$ colcon build-in-container --help

usage: colcon build-in-container [-h] [--ros-distro ROS_DISTRO] [--colcon-build-args *] [--debug] [--paths [PATH [PATH ...]]]

Call a colcon build command inside a fresh container.

options:
  -h, --help            show this help message and exit
  --ros-distro ROS_DISTRO
                        ROS version, can also be set by the environment variable ROS_DISTRO.
  --colcon-build-args *
                        Pass arguments to the colcon build command
  --debug               Shell into the environment in case the build fails.
  --shell-after         Shell into the environment at the end of the build or if there is an error. This flag includes "--debug".
  --provider            Environment provider.
```

By default, `build-in-container` uses the ROS version from the `ROS_DISTRO` environment variable.
This can be overwritten with the option `--ros-distro` allowing one to compile for a different ROS distribution than the one associated with the host OS.

### colcon test-in-container

Basic usage:
```
colcon test-in-container
```

Advanced usage:
```
colcon --log-level=info test-in-container --ros-distro humble --colcon-test-args "--cmake-args -DCMAKE_BUILD_TYPE=Release" --debug
```

Usage help:
```
$ colcon test-in-container --help

usage: colcon test-in-container [-h] [--ros-distro ROS_DISTRO] [--colcon-test-args *] [--debug] [--paths [PATH [PATH ...]]]

Call a colcon test command inside a fresh container.

options:
  -h, --help            show this help message and exit
  --ros-distro ROS_DISTRO
                        ROS version, can also be set by the environment variable ROS_DISTRO.
  --colcon-test-args *  Pass arguments to the colcon test command. Arguments matching other
                        options must be prefixed by a space.
  --debug               Shell into the environment in case the build fails.
  --shell-after         Shell into the environment at the end of the build or if there is an
                        error. This flag includes "--debug".
  --provider {lxd}      Environement provider.
```

By default, buil and test `in-container` use the ROS version from the `ROS_DISTRO` environment variable.
This can be overwritten with the option `--ros-distro` allowing one to compile for a different ROS distribution than the one associated with the host OS.

## Use cases
The colcon `in-container` extension can be used to:
- Build and test a ROS 2 package in a clean environment before releasing it
- Build and test a ROS 2 package for a different ROS distribution
- Make sure that your `package.xml` are up to date
- Build and test a ROS 2 workspace with a ROS 2 version you haven't installed
- And more!

## Troubleshooting
If you have issues with pylxd and openssl:

Pylxd doesn't work from the apt debian or the pip package. You [must be installed from source](https://discuss.linuxcontainers.org/t/5-0-2-raises-connection-reset-by-peer-exception-on-pylxds-container-execute/16292)
