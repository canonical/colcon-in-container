# Copyright (C) 2024 Canonical, Ltd.

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
import shutil
import subprocess
from typing import List

from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers.provider import Provider


def _get_multipass_path():
    return shutil.which('multipass')


class MultipassClient(Provider):
    """Multipass client interacting with the Multipass socket."""

    def __init__(self, ros_distro, pro_token=None):  # noqa: D107
        super().__init__(ros_distro)

        self.multipass_path = _get_multipass_path()
        if not self.multipass_path:
            raise exceptions.ProviderNotInstalledOnHostError(
                'Multipass is not installed.'
                'Please run `sudo snap install multipass`')

        self._render_and_write_jinja_template(pro_token)

        if self._run(['info', self.instance_name]).returncode == 0:
            self.clean_instance()

        cpus = os.getenv('COLCON_IN_CONTAINER_MULTIPASS_CPUS', default='2')
        mem = os.getenv('COLCON_IN_CONTAINER_MULTIPASS_MEMORY', default='4G')
        disk = os.getenv('COLCON_IN_CONTAINER_MULTIPASS_DISK', default='32G')

        logger.info('Downloading the image then '
                    'creating the Multipass instance')
        self._run(
            ['launch', self.ubuntu_distro,
             '--name', self.instance_name,
             '--cpus', cpus,
             '--memory', mem,
             '--disk', disk,
             '--cloud-init', self.rendered_cloud_init_path,
             '--timeout', '1000'], check=True)
        self.execute_command(['cloud-init', 'status', '--wait'])

    def _render_and_write_jinja_template(self, pro_token):
        cloud_init_content = self._render_jinja_template(pro_token)
        self.rendered_cloud_init_path = '.colcon-in-container-cloud-init.yaml'
        with open(self.rendered_cloud_init_path, 'w') as f:
            written = f.write(cloud_init_content)
            if not written:
                raise exceptions.ProviderNotConfiguredError(
                    'Failed to write cloud-init file '
                    f'in {self.rendered_cloud_init_path}')

    def _run(self, command: List[str], **kwargs):
        """Execute a multipass command.

        Returns the return code from the command.
        """
        command = [str(self.multipass_path), *command]

        logger.debug(f'Executing on host: {" ".join(command)}')
        return subprocess.run(command, **kwargs)

    def clean_instance(self):
        """Clean the created instance."""
        self._run(['delete', '--purge', self.instance_name])

    def execute_command(self, command: List[str]):
        """Execute the given command inside the instance."""
        completed_process = self._run(['exec', self.instance_name,
                                       '--working-directory', '/root/ws',
                                       '--', 'sudo', *command],
                                      capture_output=True)

        self.logger_instance.debug(completed_process.stdout.strip())
        return completed_process.returncode

    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        temporary_instance_path = f'/home/ubuntu/{instance_path}'

        move_return_code = self.execute_command([
            f'mkdir -p {temporary_instance_path}'])

        move_return_code &= self.execute_command([
            f'cp -r {instance_path}/* {temporary_instance_path}'])

        if move_return_code:
            raise exceptions.FileNotFoundInInstanceError(
                temporary_instance_path)

        command_result = self._run(['transfer', '--recursive', '--parents',
                                    f'{self.instance_name}:'
                                    f'{temporary_instance_path}',
                                    host_path], check=True)
        if command_result.returncode:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def _write_in_instance(self, *, instance_file_path, lines):
        """Write file in the instance."""
        command = [self.multipass_path, 'transfer', '--parents',
                   '-', f'{self.instance_name}:{instance_file_path}']
        with subprocess.Popen(
            command, stdin=subprocess.PIPE, stderr=subprocess.PIPE
        ) as proc:
            # Make mypy happy
            assert proc.stdin is not None
            assert proc.stderr is not None

            for line in lines:
                proc.stdin.write(bytes(line, 'utf-8'))

            # Close stdin before reading stderr, otherwise read() will hang
            # because process is waiting for more data.
            proc.stdin.close()

            # Take one read of stderr in case there is anything useful
            # for debugging an error.
            stderr = proc.stderr.read()

        if proc.returncode:
            raise exceptions.ProviderClientError(
                f'Failed to write data to destination {instance_file_path}. '
                f'Failed withe return code: {proc.returncode} and'
                f'stderr: {stderr.decode("utf-8")}'
            )

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the instance to the host.

        Because we cannot transfer as root in the VM without being root on the
        host, we use a trick to transfer it in a safe place then moving it.
        """
        temporary_instance_path = f'/home/ubuntu/{instance_path}'
        transfer_result = self._run(['transfer', '--recursive', '--parents',
                                    str(host_path),
                                    (f'{self.instance_name}:'
                                        f'{temporary_instance_path}')],
                                    check=True)
        if transfer_result.returncode:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

        move_return_code = self.execute_command([
            f'mv {temporary_instance_path}/* {instance_path}'])
        move_return_code &= self.execute_command([
            f'chown root:root -R {instance_path}'])

        if move_return_code:
            raise exceptions.FileNotFoundInInstanceError(
                temporary_instance_path)

    def shell(self):
        """Shell into the instance."""
        self._run(['exec', self.instance_name, '--', 'sudo', 'bash'])
