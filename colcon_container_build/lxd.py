# Copyright (C) 2023 Canonical, Ltd.

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from functools import partial
import logging
import os
from platform import system
import shutil
import subprocess

from colcon_container_build.helper \
    import get_ubuntu_distro, host_architecture
from colcon_core.logging import colcon_logger
from pylxd import Client, exceptions


logger = colcon_logger.getChild(__name__)


class LXDClient(object):
    """LXD client interacting with the LXD socket."""

    def __init__(self, ros_distro):  # noqa: D107
        if system() != 'Linux':
            raise Exception('LXDClient is only supported on Linux')

        try:
            self.lxd_client = Client()
        except exceptions.ClientConnectionFailed as e:
            raise Exception(
                'Failed to initialized LXD client. '
                'Make sure LXD is installed (sudo snap install lxd) and '
                f'initialised (lxd init --auto): {e}'
            )

        self.container_name = 'colcon-container-build'
        self.ros_distro = ros_distro
        ubuntu_distro = get_ubuntu_distro(self.ros_distro)
        self.source_ros_install = f'. /opt/ros/{self.ros_distro}/setup.bash'

        # Handler to remove line breaks
        handler = logging.StreamHandler()
        handler.terminator = ''
        logger.addHandler(handler)

        config = {
            'name': self.container_name,
            'source': {
                'type': 'image',
                'protocol': 'simplestreams',
                'server': 'https://images.linuxcontainers.org',
                'alias': f'ubuntu/{ubuntu_distro}/{host_architecture()}',
            },
            'ephemeral': True,
        }

        if self.lxd_client.instances.exists(self.container_name):
            previous_instance = self.lxd_client.instances.get(self.container_name)
            if previous_instance.status == 'Running':
                previous_instance.stop(wait=True)

        self.instance = self.lxd_client.instances.create(config, wait=True)
        self.instance.start(wait=True)
        if self._install_ros().exit_code:
            raise Exception('Failed to install ROS in the container')

    def __del__(self):  # noqa: D105
        if self.instance:
            if self.instance.status == 'Running':
                self.instance.stop(wait=True)

    def _install_ros(self):
        # handle the different versions of ros2
        commands = [
            'apt-get update',
            'apt-get install -y software-properties-common curl',
            'curl -sSL '
            'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key '
            '-o /usr/share/keyrings/ros-archive-keyring.gpg',
            'echo "deb [arch=$(dpkg --print-architecture) '
            'signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] '
            'http://packages.ros.org/ros2/ubuntu '
            '$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | '
            'tee /etc/apt/sources.list.d/ros2.list > /dev/null',
            'apt-get update',
            'apt-get upgrade -y',
            f'apt-get install -y ros-{self.ros_distro}-ros-base',
            'apt-get install -y ros-dev-tools',
        ]

        return self._execute_commands(commands)

    def _execute_command(self, command):
        return self.instance.execute(
            command, stdout_handler=logger.info, stderr_handler=logger.error
        )

    def _execute_commands(self, commands):
        commands_to_run = '#!/bin/bash\n'
        commands_to_run += '\n'.join(commands)
        self.instance.files.put('/tmp/script', commands_to_run)
        return self._execute_command(['bash','-xe',  '/tmp/script'])

    def _call_rosdep(self):
        # initialize and call rosdep over our repository
        logger.info('installing and calling rosdep')
        commands = [
            self.source_ros_install,
            'rosdep init',
            'rosdep update',
            'rosdep install --from-paths /ws/src --ignore-src -y',
        ]

        return self._execute_commands(commands)

    def _build(self, colcon_build_args):
        logger.info('building workspace')
        print(colcon_build_args)
        commands = [
            self.source_ros_install,
            'cd /ws',
            f'colcon build {colcon_build_args}',
        ]
        return self._execute_commands(commands)

    def _download_results(self):
        logger.info('downloading install/ on host')
        if os.path.exists(self.container_name):
            shutil.rmtree(self.container_name, ignore_errors=True)

        return self.instance.files.recursive_get('/ws/install', self.container_name)

    def upload_package(self, package_name, package_path):
        """Upload package to container workspace."""
        logger.info(f'uploading {package_path} into the container /ws/src/')
        container_package_path = f'/ws/src/{package_name}'
        self._execute_command(['mkdir', '-p', container_package_path])
        self.instance.files.recursive_put(package_path, container_package_path)

    def build(self, colcon_build_args):
        """Build the workspace.

        Pull build-time dependencies, build the workspace and download the
        result build directory.
        """
        commands = [self._call_rosdep,
                    partial(self._build, colcon_build_args),
                    self._download_results]
        for command in commands:
            ret = command()
            if ret.exit_code:
                return

    def shell(self):
        """Shell into the container."""
        subprocess.run(['lxc', 'exec', self.container_name,
                        '--cwd', '/ws',
                        '--','bash'])
