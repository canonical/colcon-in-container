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
import jinja2
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
                'alias': f'ubuntu/{ubuntu_distro}/cloud/{host_architecture()}',
            },
            'ephemeral': True,
            'config': {},
        }

        config_directory = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 'config')
        cloud_init_file = os.path.join(
            config_directory, 'cloud-init.yaml')
        with open(cloud_init_file, 'r') as f:
            environment = jinja2.Environment()
            template = environment.from_string(f.read())
            user_data : dict
            user_data = template.render(
                v1 = {
                    'machine': host_architecture(),
                    'distro_release': get_ubuntu_distro(self.ros_distro),
                    'ros_release': self.ros_distro,
                }
            )

            config['config']['user.user-data'] = f'{user_data}'

        if self.lxd_client.instances.exists(self.container_name):
            previous_instance = self.lxd_client.instances.get(self.container_name)
            if previous_instance.status == 'Running':
                previous_instance.stop(wait=True)

        logger.info('Downloading the image then creating the LXD instance')
        self.instance = self.lxd_client.instances.create(config, wait=True)
        self.instance.start(wait=True)
        logger.info('Waiting for ROS 2 to be installed')
        self._execute_command(["cloud-init", "status", "--wait"])

    def __del__(self):  # noqa: D105
        if self.instance:
            if self.instance.status == 'Running':
                self.instance.stop(wait=True)

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
                    partial(self._build, colcon_build_args)]
        for command in commands:
            ret = command()
            if ret.exit_code:
                return

        self._download_results()

    def shell(self):
        """Shell into the container."""
        subprocess.run(['lxc', 'exec', self.container_name,
                        '--cwd', '/ws',
                        '--','bash'])
