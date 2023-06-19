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
import os
from platform import system
import shutil
import subprocess
from typing import Any, Callable, Dict, List

from colcon_build_in_container.helper \
    import get_ubuntu_distro, host_architecture
from colcon_core.logging import colcon_logger
from pylxd import Client, exceptions
from pylxd.models.instance import _InstanceExecuteResult


logger = colcon_logger.getChild(__name__)


def _is_lxd_installed():
    return shutil.which('lxd') is not None


class LXDClient(object):
    """LXD client interacting with the LXD socket."""

    def __init__(self, ros_distro):  # noqa: D107
        if system() != 'Linux':
            raise SystemError('LXDClient is only supported on Linux')

        if not _is_lxd_installed():
            raise SystemError('LXD is not installed. Please run '
                              '`sudo snap install lxd`')

        try:
            self.lxd_client = Client()
        except exceptions.ClientConnectionFailed as e:
            raise SystemError(
                'Failed to initialized LXD client. '
                f'Make sure LXD is properly installed and running: {e}'
            )

        if not self._is_lxd_initialised():
            raise SystemError('LXD is not initialised. Please run '
                              '`lxd init --auto')

        self.container_name = 'colcon-build-in-container'
        self.host_install_folder = 'install_in_container'
        self.ros_distro = ros_distro
        ubuntu_distro = get_ubuntu_distro(self.ros_distro)

        self.logger_container = logger.getChild('container')

        config: Dict[str, Any] = {
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
            config['config']['user.user-data'] = f.read().replace(
                '{{ ros_version }}',
                f'{self.ros_distro}')

        if self.lxd_client.instances.exists(self.container_name):
            previous_instance = self.lxd_client.instances.get(
                self.container_name)
            if previous_instance.status == 'Running':
                previous_instance.stop(wait=True)
            else:
                previous_instance.delete(wait=True)

        logger.info('Downloading the image then creating the LXD instance')
        self.instance = self.lxd_client.instances.create(config, wait=True)
        self.instance.start(wait=True)
        logger.info('Waiting for ROS 2 to be installed')
        self._execute_command(['cloud-init', 'status', '--wait'])

    def __del__(self):  # noqa: D105
        if hasattr(self, 'instance') and self.instance:
            if self.instance.status == 'Running':
                self.instance.stop(wait=True)

    def _is_lxd_initialised(self):
        devices = self.lxd_client.profiles.get('default').devices
        return bool(devices)

    def _execute_command(self, command):
        return self.instance.execute(
            command, stdout_handler=self.logger_container.info,
            stderr_handler=self.logger_container.info, cwd='/ws'
        )

    def _execute_commands(self, commands):
        commands_to_run = '#!/bin/bash\n'
        commands_to_run += '\n'.join(commands)
        self.instance.files.put('/tmp/script', commands_to_run)
        return self._execute_command(['bash', '-xei', '/tmp/script'])

    def _call_rosdep(self):
        # initialize and call rosdep over our repository
        logger.info('Initialising and calling rosdep')
        commands = [
            'rosdep init',
            'rosdep update',
            # Avoid rosdep/apt interactive shell error message
            'export DEBIAN_FRONTEND=noninteractive',
            'rosdep install --from-paths /ws/src --ignore-src -y '
            f'--rosdistro={self.ros_distro}',
            '--dependency-types=build',
            '--dependency-types=buildtool',
            '--dependency-types=build_export',
            '--dependency-types=buildtool_export',
        ]

        return self._execute_commands(commands)

    def _build(self, colcon_build_args):
        logger.info(f'building workspace with args: {colcon_build_args}')
        return self._execute_commands([
            f'colcon --log-level={logger.getEffectiveLevel()} '
            f'build {colcon_build_args}'])

    def _download_results(self):
        logger.info('downloading install/ on host')
        if os.path.exists(self.host_install_folder):
            shutil.rmtree(self.host_install_folder, ignore_errors=True)
        try:
            self.instance.files.recursive_get('/ws/install',
                                              self.host_install_folder)
        except exceptions.NotFound:
            logger.warn('/ws/install was empty. '
                        'Are you sure you built packages?')

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
        commands: List[Callable[[], _InstanceExecuteResult]] = [
            self._call_rosdep,
            partial(self._build, colcon_build_args)]
        for command in commands:
            exit_code = command().exit_code
            if exit_code:
                return exit_code

        self._download_results()
        return 0

    def shell(self):
        """Shell into the container."""
        subprocess.run(['lxc', 'exec', self.container_name, '--', 'bash'])
