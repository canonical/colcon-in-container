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

import json
import os
from platform import system
import shutil
import subprocess


from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers._helper \
    import host_architecture
from colcon_in_container.providers.provider import Provider


def _is_lxd_installed():
    return shutil.which('lxd') is not None


class LXDClient(Provider):
    """LXD client interacting with the LXD socket."""

    def __init__(self, ros_distro, pro_token=None):  # noqa: D107
        super().__init__(ros_distro)
        if system() != 'Linux':
            raise exceptions.ProviderDoesNotSupportHostOSError(
                'LXDClient is only supported on Linux')

        if not _is_lxd_installed():
            raise exceptions.ProviderNotInstalledOnHostError(
                'LXD is not installed. Please run `sudo snap install lxd`')

        # Check if LXD is running by trying to connect to it
        try:
            subprocess.run(
                ['lxc', 'list', '--format', 'json'],
                capture_output=True,
                check=True,
                text=True
            )
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            raise exceptions.ProviderClientError(
                'Failed to initialized LXD client. '
                f'Make sure LXD is properly installed and running: {e}'
            )

        if not self._is_lxd_initialised():
            raise exceptions.ProviderNotConfiguredError(
                'LXD is not initialised. Please run `lxd init --auto`')

        if self.ubuntu_distro == 'noble':
            # necessary due to
            # https://github.com/canonical/cloud-init/issues/5223
            cloud_init_url = 'https://cloud-images.ubuntu.com/releases/'
        else:
            cloud_init_url = \
                'https://cloud-images.ubuntu.com/minimal/releases/'

        image_alias = f'{self.ubuntu_distro}/{host_architecture()}'
        cloud_init_data = self._render_jinja_template(pro_token)

        # Check if instance already exists and clean it up
        if self._instance_exists(self.instance_name):
            instance_status = self._get_instance_status(self.instance_name)
            if instance_status == 'Running':
                self._stop_instance(self.instance_name)
            else:
                self._delete_instance(self.instance_name)

        logger.info('Downloading the image then creating the LXD instance')

        # Create instance with cloud-init config
        # First, we need to write the cloud-init config to a temporary file
        import tempfile
        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml',
                delete=False) as cloud_init_file:
            cloud_init_file.write(cloud_init_data)
            cloud_init_path = cloud_init_file.name

        try:
            # Launch instance with cloud-init
            subprocess.run([
                'lxc', 'launch',
                f'{cloud_init_url}:{image_alias}',
                self.instance_name,
                '--ephemeral',
                '--config', f'user.user-data={cloud_init_data}'
            ], check=True, capture_output=True, text=True)
        finally:
            # Clean up temp file
            if os.path.exists(cloud_init_path):
                os.unlink(cloud_init_path)

    def _instance_exists(self, instance_name):
        """Check if an LXD instance exists."""
        result = subprocess.run(
            ['lxc', 'list', instance_name, '--format', 'json'],
            capture_output=True,
            check=True,
            text=True
        )
        instances = json.loads(result.stdout)
        return len(instances) > 0

    def _get_instance_status(self, instance_name):
        """Get the status of an LXD instance."""
        result = subprocess.run(
            ['lxc', 'list', instance_name, '--format', 'json'],
            capture_output=True,
            check=True,
            text=True
        )
        instances = json.loads(result.stdout)
        if instances:
            return instances[0]['status']
        return None

    def _stop_instance(self, instance_name):
        """Stop an LXD instance."""
        subprocess.run(
            ['lxc', 'stop', instance_name],
            check=True,
            capture_output=True
        )

    def _delete_instance(self, instance_name):
        """Delete an LXD instance."""
        subprocess.run(
            ['lxc', 'delete', instance_name, '--force'],
            check=True,
            capture_output=True
        )

    def clean_instance(self):
        """Clean the created instance."""
        if self._instance_exists(self.instance_name):
            instance_status = self._get_instance_status(self.instance_name)
            if instance_status == 'Running':
                self._stop_instance(self.instance_name)

    def _is_lxd_initialised(self):
        """Check if LXD is initialized by checking the default profile."""
        try:
            result = subprocess.run(
                ['lxc', 'profile', 'show', 'default'],
                capture_output=True,
                check=True,
                text=True
            )
            profile_data = result.stdout
            # Parse YAML to check if devices are configured
            # A properly initialized LXD will have devices configured
            return (
                'devices:' in profile_data
                and len(profile_data.strip()) > 20
            )
        except subprocess.CalledProcessError:
            return False

    def execute_command(self, command):
        """Execute the given command inside the instance."""
        # Execute command in the instance with working directory /ws
        result = subprocess.run(
            ['lxc', 'exec', self.instance_name, '--cwd', '/ws', '--']
            + command,
            capture_output=True,
            text=True
        )

        # Log stdout and stderr
        if result.stdout:
            self.logger_instance.debug(result.stdout)
        if result.stderr:
            self.logger_instance.debug(result.stderr)

        return result.returncode

    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        try:
            # Use lxc file pull with recursive flag
            subprocess.run(
                ['lxc', 'file', 'pull', '-r',
                 f'{self.instance_name}{instance_path}',
                 host_path],
                check=True,
                capture_output=True
            )
        except subprocess.CalledProcessError:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def _write_in_instance(self, *, instance_file_path, lines):
        """Write data to a file in the instance."""
        # Use lxc file push with stdin
        subprocess.run(
            ['lxc', 'file', 'push', '-',
             f'{self.instance_name}{instance_file_path}'],
            input=lines if isinstance(lines, bytes) else lines.encode('utf-8'),
            check=True,
            capture_output=True
        )

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the host to the instance."""
        # Use lxc file push with recursive flag
        subprocess.run(
            ['lxc', 'file', 'push', '-r', '-p',
             host_path,
             f'{self.instance_name}{instance_path}'],
            check=True,
            capture_output=True
        )

    def shell(self):
        """Shell into the instance."""
        subprocess.run(['lxc', 'exec', self.instance_name, '--', 'bash'])
