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

from abc import ABC, abstractmethod
import os
import shutil

from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers._helper import get_ubuntu_distro


class Provider(ABC):
    """Provider client."""

    def __init__(self, ros_distro):  # noqa: D107
        self.instance_name = 'colcon-in-container'
        self.ros_distro = ros_distro
        self.ubuntu_distro = get_ubuntu_distro(self.ros_distro)
        self.logger_instance = logger.getChild('instance')

    def __del__(self):  # noqa: D105
        self._clean_instance()

    @abstractmethod
    def _clean_instance(self):
        """Clean the created instance."""
        pass

    def wait_for_install(self):
        """Wait for installation to be done."""
        logger.info('Waiting for ROS 2 to be installed')
        cloud_init_exit_code = \
            self.execute_command(['cloud-init', 'status', '--wait'])
        if cloud_init_exit_code:
            raise exceptions.CloudInitError(
                'Failed to run cloud-init with error: '
                f'{cloud_init_exit_code}.'
            )

    @abstractmethod
    def execute_command(self, command) -> int:
        """Execute the given command inside the instance."""
        pass

    def execute_commands(self, commands):
        """Execute multiple commands inside the instance."""
        commands_to_run = '#!/bin/bash\n'
        commands_to_run += '\n'.join(commands)
        self._write_in_instance(instance_file_path='/tmp/script',
                                lines=commands_to_run)
        return self.execute_command(['bash', '-xei', '/tmp/script'])

    @abstractmethod
    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        pass

    def download_result(self, *,
                        result_path_in_instance,
                        result_path_on_host):
        """Download result from the instance on the host."""
        logger.info(f'downloading {result_path_in_instance} on host')
        if os.path.exists(result_path_on_host):
            shutil.rmtree(result_path_on_host, ignore_errors=True)
        try:
            self._copy_from_instance_to_host(
                instance_path=result_path_in_instance,
                host_path=result_path_on_host
            )
        except FileNotFoundError:
            logger.error(f'{result_path_in_instance} was empty. '
                         'Are you sure you executed the right command?')
            raise exceptions.FileNotFoundInInstanceError(
                result_path_in_instance)

    @abstractmethod
    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the instance to the host."""
        pass

    @abstractmethod
    def _write_in_instance(self, *, instance_file_path, lines):
        """Copy data from the instance to the host."""
        pass

    def upload_package(self, package_path):
        """Upload package to instance workspace."""
        package_name = os.path.basename(package_path)
        instance_package_path = f'/ws/src/{package_name}'
        self.upload_directory(host_path=package_path,
                              instance_path=instance_package_path)

    def upload_directory(self, *, host_path, instance_path):
        """Upload package to instance workspace."""
        logger.info(f'uploading {host_path} into the instance {instance_path}')
        if not os.path.isdir(host_path):
            raise exceptions.FileNotFoundInHostError(host_path)

        self.execute_command(['mkdir', '-p', instance_path])
        self._copy_from_host_to_instance(
            host_path=host_path,
            instance_path=instance_path
        )

    @abstractmethod
    def shell(self):
        """Shell into the instance."""
        pass
