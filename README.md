# colcon-in-container

colcon verb extension to build and test inside a fresh and isolated ROS environment and transfer the results back to the host.

With this extension, developers can build ROS packages for any ROS 2 distributions directly from colcon independently of the host. 
With it, one can validate 'builds' and 'tests' making sure all the dependencies are properly listed in their `package.xml` on any ROS 2 distribution. 
Validating packages and workspace in an isolated and ephemeral environment is key for distributing and packaging software.

## Quickstart

- Install the tool:
```
pip3 install -U git+https://github.com/canonical/colcon-in-container
```
- Install and initialize LXD:
```
sudo snap install lxd
lxd init --auto
```
- Then call colcon with the build-in-container verb:
```
colcon --log-level=info build-in-container --ros-distro jazzy
```

See the [usage](#usage) section for advanced information on installation and tool usage.

## How it works

### colcon build-in-container

`colcon build-in-container` performs the following steps to build your workspace:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Upload your workspace inside the container
- Install your build time rosdep dependencies (make sure to keep your `package.xml` updated!)
- Build you workspace inside the container
- Download the `build/` and `install/` directories of the built workspace on your host under: `build_in_container` and `install_in_container/`

### colcon test-in-container

`colcon test-in-container` performs the following steps to build your workspace:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Upload your workspace as well as your `build_in_container` and `install_in_container/` directories inside the container
- Install your test time your rosdep dependencies (make sure to keep your `package.xml` updated!)
- Test you workspace inside the container
- Download the test results directory of the built workspace on your host under: `test_results_in_container/`

### colcon release-in-container

`colcon release-in-container` performs the following steps to release your workspace's packages:
- Start an ephemeral `LXD` container
- Install a fresh ROS 2
- Install the necessary software to generate a Debian file
- Upload your workspace inside the container
- Install your build time rosdep dependencies (make sure to keep your `package.xml` updated!)
- Generate the Debian for each of the selected packages.
- Download the results of the releases on your host under: `release_in_container/`

## Usage
### Installation
#### colcon in-container
To use the extension you will need to install it and also install and initialize LXD. Both steps are described below.

This being a colcon extension, make sure to have [colcon installed](https://colcon.readthedocs.io/en/released/user/installation.html).

The extension can be installed via `pip` using the URL with the following command:

```
pip3 install -U git+https://github.com/canonical/colcon-in-container
```
Due to potential conflict with the python dependencies `cryptography` and `pyopenssl` between a ROS install and the tool, a [python virtual environment](https://docs.python.org/3/library/venv.html) is recommended.
#### LXD

`LXD` can be installed with snap:

```
sudo snap install lxd
```

Alternative installation methods are available in the [`LXD` documentation](https://documentation.ubuntu.com/lxd/en/latest/installing/).

Initialize `LXD` with:

```
lxd init --auto
```

### colcon build-in-container

Basic usage:
```
colcon build-in-container
```

Advanced usage:
```
colcon --log-level=info build-in-container --ros-distro jazzy --colcon-build-args "--cmake-args -DCMAKE_BUILD_TYPE=Release" --debug
```

Usage help:
```
$ colcon build-in-container --help

usage: colcon build-in-container [-h] [--ros-distro ROS_DISTRO] [--colcon-build-args *] [--debug] [--shell-after] [--paths [PATH [PATH ...]]]

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
colcon --log-level=info test-in-container --ros-distro jazzy --colcon-test-args "--cmake-args -DCMAKE_BUILD_TYPE=Release" --debug
```

Usage help:
```
$ colcon test-in-container --help

usage: colcon test-in-container [-h] [--ros-distro ROS_DISTRO] [--colcon-test-args *] [--debug] [--shell-after] [--paths [PATH [PATH ...]]]

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
  --provider {lxd}      Environment provider.
```

By default, buil and test `in-container` use the ROS version from the `ROS_DISTRO` environment variable.
This can be overwritten with the option `--ros-distro` allowing one to compile for a different ROS distribution than the one associated with the host OS.

### colcon release-in-container

Basic usage:
```
colcon release-in-container
```

Advanced usage:
```
colcon --log-level=info release-in-container --ros-distro jazzy --bloom-generator rosdebian --debug
```

Usage help:
```
$ colcon release-in-container --help

usage: colcon release-in-container [-h] [--ros-distro ROS_DISTRO] [--bloom-generator {debian,rosdebian}] [--debug] [--shell-after] [--paths [PATH [PATH ...]]]

Call a colcon build command inside a fresh container.

options:
  -h, --help            show this help message and exit
  --ros-distro ROS_DISTRO
                        ROS version, can also be set by the environment variable ROS_DISTRO.
  --bloom-generator {debian,rosdebian}
                        Pass arguments to the bloom-generate command
                        Pass arguments to the colcon build command
  --debug               Shell into the environment in case the build fails.
  --shell-after         Shell into the environment at the end of the build or if there is an error. This flag includes "--debug".
  --provider            Environment provider.
```

By default, `release-in-container` uses the ROS version from the `ROS_DISTRO` environment variable.
This can be overwritten with the option `--ros-distro` allowing one to compile for a different ROS distribution than the one associated with the host OS.

## Use cases
The colcon `in-container` extension can be used to:
- Build, test and release ROS 2 package in a clean environment
- Build, test and release a ROS 2 package for a different ROS distribution
- Make sure that your `package.xml` are up to date
- Build, test and release a ROS 2 workspace with a ROS 2 version you haven't installed
- And more!

## Troubleshooting
If you have issues with pylxd and openssl:

Pylxd doesn't work from the apt debian or the pip package. You [must be installed from source](https://discuss.linuxcontainers.org/t/5-0-2-raises-connection-reset-by-peer-exception-on-pylxds-container-execute/16292)

Please open a [GitHub issue](https://github.com/canonical/colcon-in-container/issues) if you face any issue with the tool.
