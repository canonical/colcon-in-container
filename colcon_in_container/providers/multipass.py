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
from platform import processor
import shutil
import subprocess
from typing import Any, Dict, List
import jinja2

from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers.provider import Provider


def _get_multipass_path():
    return shutil.which('multipass')


class MultipassClient(Provider):
    """Multipass client interacting with the Multipass socket."""

    def __init__(self, ros_distro):  # noqa: D107
        super().__init__(ros_distro)

        self.multipass_path = _get_multipass_path()
        if not self.multipass_path:
            raise exceptions.ProviderNotInstalledOnHostError(
                'Multipass is not installed. Please run `sudo snap install multipass`')

        config_directory = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'config')
        cloud_init_file = os.path.join(
            config_directory, 'cloud-init.yaml')
        with open(cloud_init_file, 'r') as f:
            config = f.read()

        template = jinja2.Environment().from_string(source=config)
        cloud_init_content = template.render({'v1': {'machine': processor(), 'distro_release': self.ubuntu_distro}})
        self.rendered_cloud_init_path = '.colcon-in-container-cloud-init.yaml'
        with open(self.rendered_cloud_init_path, 'w') as f:
            written = f.write(cloud_init_content)
            if not written:
                raise exceptions.ProviderNotConfiguredError(
                    'Failed to write cloud-init file '
                    f'in {self.rendered_cloud_init_path}')

        if self._run(['info', self.instance_name]).returncode == 0:
           self._clean_instance()

        logger.info('Downloading the image then creating the Multipass instance')
        self._run(['launch', self.ubuntu_distro,
              '--name', self.instance_name,
              '--cloud-init', self.rendered_cloud_init_path,
              '--timeout', '1000'], check=True)
        self.execute_command(['cloud-init', 'status', '--wait'])


    def _run(self, command: List[str], **kwargs):
        """Execute a multipass command.

        Returns the return code from the command.
        """
        command = [str(self.multipass_path), *command]

        logger.debug(f'Executing on host: {" ".join(command)}')
        return subprocess.run(command, **kwargs)

    def _clean_instance(self):
        self._run(['delete', '--purge', self.instance_name])

    def execute_command(self, command: List[str]):
        """Execute the given command inside the instance."""
        completed_process = self._run(['exec', self.instance_name,
                         '--working-directory', '/ws',
                         *command],
                         capture_output=True)
                         #check=True)
        self.logger_instance.debug(completed_process.stdout.strip())
        return completed_process

    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        return_code = self._run(['transfer', '--recursive',
                                 f'{self.instance_name}:{instance_path}',
                                 host_path], check=True)
        if return_code:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def _write_in_instance(self, *, instance_file_path, lines):
        """Write file in the instance."""
        command = [self.multipass_path, 'transfer', '-', instance_file_path]
        with subprocess.Popen(
            command, stdin=subprocess.PIPE, stderr=subprocess.PIPE
        ) as proc:
            assert proc.stdin is not None
            assert proc.stderr is not None

            for line in lines:
                proc.stdin.write(line)

            # Close stdin before reading stderr, otherwise read() will hang
            # because process is waiting for more data.
            proc.stdin.close()

            # Take one read of stderr in case there is anything useful
            # for debugging an error.
            stderr = proc.stderr.read()

        if proc.returncode != 0:
            raise exceptions.ProviderClientError(
                f'Failed to write data to destination {instance_file_path}. '
                f'Failed withe return code: {proc.returncode} and stderr: {stderr}'
            )

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the instance to the host."""
        return_code = self._run(['transfer', '--recursive',
                                 host_path,
                                 f'{self.instance_name}:{instance_path}'],
                                 check=True)
        if return_code:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def shell(self):
        """Shell into the instance."""
        self._run(['exec', self.instance_name, '--', 'bash'])
