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

import glob
import json
import os
from platform import system
import shutil
import subprocess
import tempfile


from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
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
                'Failed to initialize LXD client. '
                f'Make sure LXD is properly installed and running: {e}'
            )

        if not self._is_lxd_initialised():
            raise exceptions.ProviderNotConfiguredError(
                'LXD is not initialised. Please run `lxd init --auto`')

        cloud_init_data = self._render_jinja_template(pro_token)

        # Check if instance already exists and clean it up
        if self._instance_exists(self.instance_name):
            instance_status = self._get_instance_status(self.instance_name)
            if instance_status == 'Running':
                self._stop_instance(self.instance_name)
            # Delete the instance after stopping
            self._delete_instance(self.instance_name)

        logger.info('Downloading the image then creating the LXD instance')

        # Create instance (but don't start it yet)
        subprocess.run([
            'lxc', 'init',
            'ubuntu-minimal',
            self.instance_name,
            '--ephemeral'
        ], check=True, capture_output=True, text=True)

        # Set cloud-init config using stdin to avoid
        # command line length limits and special character issues
        subprocess.run(
            ['lxc', 'config', 'set', self.instance_name,
             'user.user-data', '-'],
            input=cloud_init_data.encode('utf-8'),
            check=True,
            capture_output=True
        )

        # Start instance with cloud-init config applied
        subprocess.run(
            ['lxc', 'start', self.instance_name],
            check=True,
            capture_output=True
        )

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
        # Ignore errors - instance may not exist or may fail to delete
        subprocess.run(
            ['lxc', 'delete', instance_name, '--force'],
            capture_output=True
        )

    def clean_instance(self):
        """Clean the created instance."""
        if self._instance_exists(self.instance_name):
            instance_status = self._get_instance_status(self.instance_name)
            if instance_status == 'Running':
                self._stop_instance(self.instance_name)
            # Delete the instance after stopping
            self._delete_instance(self.instance_name)

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
            # Create a temporary directory to pull into
            with tempfile.TemporaryDirectory() as temp_dir:
                # Pull the directory - lxc will create a subdirectory with
                # the source name inside temp_dir
                subprocess.run(
                    ['lxc', 'file', 'pull', '--recursive',
                     f'{self.instance_name}/{instance_path}',
                     temp_dir],
                    check=True,
                    capture_output=True
                )

                # Get the basename of the source path
                source_basename = os.path.basename(instance_path.rstrip('/'))
                pulled_dir = os.path.join(temp_dir, source_basename)

                # Move the contents to the target location
                if os.path.exists(pulled_dir):
                    shutil.move(pulled_dir, host_path)
                else:
                    raise FileNotFoundError(
                        f'Expected directory {pulled_dir} not found')
        except subprocess.CalledProcessError:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def _write_in_instance(self, *, instance_file_path, lines):
        """Write data to a file in the instance."""
        # Create a temporary file on host and push it to the instance
        # Using stdin doesn't work reliably in CI environments
        with tempfile.NamedTemporaryFile(
                mode='w', delete=False) as temp_file:
            if isinstance(lines, str):
                content = lines
            else:
                content = lines.decode('utf-8')
            temp_file.write(content)
            temp_file_path = temp_file.name

        try:
            subprocess.run(
                ['lxc', 'file', 'push',
                 temp_file_path,
                 f'{self.instance_name}/{instance_file_path}'],
                check=True,
                capture_output=True
            )
        finally:
            # Clean up temporary file
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the host to the instance."""
        # To copy contents of host_path into instance_path (not create a
        # subdirectory), we need to push each item individually
        # Get all items in the source directory
        items = glob.glob(os.path.join(host_path, '*'))

        if not items:
            # Empty directory or doesn't exist
            return

        # Push each item to the destination
        for item in items:
            subprocess.run(
                ['lxc', 'file', 'push', '--recursive', '--create-dirs',
                 item,
                 f'{self.instance_name}/{instance_path}/'],
                check=True,
                capture_output=True
            )

    def shell(self):
        """Shell into the instance."""
        subprocess.run(['lxc', 'exec', self.instance_name, '--', 'bash'])
