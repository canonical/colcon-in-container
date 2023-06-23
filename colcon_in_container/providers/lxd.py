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

import os
from platform import system
import shutil
import subprocess
from typing import Any, Dict

from colcon_in_container.logging import logger
from colcon_in_container.providers._helper \
    import host_architecture
from colcon_in_container.providers.provider import Provider
from pylxd import Client, exceptions


def _is_lxd_installed():
    return shutil.which('lxd') is not None


class LXDClient(Provider):
    """LXD client interacting with the LXD socket."""

    def __init__(self, ros_distro):  # noqa: D107
        super().__init__(ros_distro)
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

        config: Dict[str, Any] = {
            'name': self.instance_name,
            'source': {
                'type': 'image',
                'protocol': 'simplestreams',
                'server': 'https://images.linuxcontainers.org',
                'alias': f'ubuntu/{self.ubuntu_distro}/cloud/'
                f'{host_architecture()}',
            },
            'ephemeral': True,
            'config': {},
        }
        config_directory = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'config')
        cloud_init_file = os.path.join(
            config_directory, 'cloud-init.yaml')
        with open(cloud_init_file, 'r') as f:
            config['config']['user.user-data'] = f.read()

        if self.lxd_client.instances.exists(self.instance_name):
            previous_instance = self.lxd_client.instances.get(
                self.instance_name)
            if previous_instance.status == 'Running':
                previous_instance.stop(wait=True)
            else:
                previous_instance.delete(wait=True)

        logger.info('Downloading the image then creating the LXD instance')
        self.instance = self.lxd_client.instances.create(config, wait=True)
        self.instance.start(wait=True)
        logger.info('Waiting for ROS 2 to be installed')
        self.execute_command(['cloud-init', 'status', '--wait'])

    def _clean_instance(self):
        if hasattr(self, 'instance') and self.instance:
            if self.instance.status == 'Running':
                self.instance.stop(wait=True)

    def _is_lxd_initialised(self):
        devices = self.lxd_client.profiles.get('default').devices
        return bool(devices)

    def execute_command(self, command):
        """Execute the given command inside the instance."""
        return self.instance.execute(
            command, stdout_handler=self.logger_instance.info,
            stderr_handler=self.logger_instance.info, cwd='/ws'
        ).exit_code

    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        try:
            self.instance.files.recursive_get(instance_path,
                                              host_path)
        except exceptions.NotFound:
            raise FileNotFoundError

    def _write_in_instance(self, *, instance_file_path, lines):
        """Copy data from the instance to the host."""
        self.instance.files.put(instance_file_path, lines)

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the instance to the host."""
        self.instance.files.recursive_put(host_path, instance_path)

    def shell(self):
        """Shell into the instance."""
        subprocess.run(['lxc', 'exec', self.instance_name, '--', 'bash'])
