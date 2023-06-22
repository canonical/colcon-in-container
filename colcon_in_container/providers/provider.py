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
from colcon_in_container.providers._helper import get_ubuntu_distro


class Provider(ABC):
    """LXD client interacting with the LXD socket."""

    def __init__(self, ros_distro):  # noqa: D107
        self.container_name = 'colcon-build-in-container'
        self.ros_distro = ros_distro
        self.ubuntu_distro = get_ubuntu_distro(self.ros_distro)
        self.logger_container = logger.getChild('container')

    def __del__(self):  # noqa: D105
        self._clean_instance()

    @abstractmethod
    def _clean_instance(self):
        """Clean the created instance."""
        pass

    @abstractmethod
    def execute_command(self, command) -> int:
        """Execute the given command inside the container."""
        pass

    def execute_commands(self, commands):
        """Execute multiple commands inside the container."""
        commands_to_run = '#!/bin/bash\n'
        commands_to_run += '\n'.join(commands)
        self._write_in_container(container_file_path='/tmp/script',
                                 lines=commands_to_run)
        return self.execute_command(['bash', '-xei', '/tmp/script'])

    @abstractmethod
    def _copy_from_container_to_host(self, *, container_path, host_path):
        """Copy data from the container to the host."""
        pass

    def download_result(self, *,
                        result_path_in_container,
                        result_path_on_host):
        """Download result from the container on the host."""
        logger.info(f'downloading {result_path_in_container} on host')
        if os.path.exists(result_path_on_host):
            shutil.rmtree(result_path_on_host, ignore_errors=True)
        try:
            self._copy_from_container_to_host(
                container_path=result_path_in_container,
                host_path=result_path_on_host
            )
        except FileNotFoundError as e:
            logger.warn(f'{result_path_in_container} was empty. '
                        'Are you sure you executed the right command?')
            raise e

    @abstractmethod
    def _copy_from_host_to_container(self, *, host_path, container_path):
        """Copy data from the container to the host."""
        pass

    @abstractmethod
    def _write_in_container(self, *, container_file_path, lines):
        """Copy data from the container to the host."""
        pass

    def upload_package(self, package_path):
        """Upload package to container workspace."""
        package_name = os.path.basename(package_path)
        logger.info(f'uploading {package_path} into the container /ws/src/')
        container_package_path = f'/ws/src/{package_name}'
        self.execute_command(['mkdir', '-p', container_package_path])
        self._copy_from_host_to_container(
            host_path=package_path,
            container_path=container_package_path
        )

    @abstractmethod
    def shell(self):
        """Shell into the container."""
        pass
