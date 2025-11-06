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
import stat
import subprocess
from typing import Any, Dict


from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers._helper \
    import host_architecture
from colcon_in_container.providers.provider import Provider
from pylxd import Client, exceptions as pylxd_exceptions


def _is_lxd_installed():
    return shutil.which('lxd') is not None


def _get_lxd_client_cert():
    """Get LXD client certificate paths if they exist.

    Checks both snap and traditional LXD installation locations.
    Returns a tuple of (cert_path, key_path) or (None, None) if not found.
    """
    # Check snap location first (most common on Ubuntu)
    snap_config_dir = os.path.expanduser('~/snap/lxd/common/config')
    snap_cert_file = os.path.join(snap_config_dir, 'client.crt')
    snap_key_file = os.path.join(snap_config_dir, 'client.key')

    if os.path.exists(snap_cert_file) and os.path.exists(snap_key_file):
        return (snap_cert_file, snap_key_file)

    # Check traditional location
    config_dir = os.path.expanduser('~/.config/lxc')
    cert_file = os.path.join(config_dir, 'client.crt')
    key_file = os.path.join(config_dir, 'client.key')

    if os.path.exists(cert_file) and os.path.exists(key_file):
        return (cert_file, key_file)

    return (None, None)


def _find_remote_name_for_endpoint(endpoint):
    """Find the lxc remote name for a given endpoint URL.

    Returns the remote name if found, None otherwise.
    """
    try:
        result = subprocess.run(
            ['lxc', 'remote', 'list', '--format', 'json'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            remotes = json.loads(result.stdout)
            # Normalize endpoint for comparison (remove trailing slash)
            normalized_endpoint = endpoint.rstrip('/')
            for remote_name, remote_info in remotes.items():
                remote_addr = remote_info.get('Addr', '').rstrip('/')
                if remote_addr == normalized_endpoint:
                    return remote_name
    except (subprocess.TimeoutExpired, json.JSONDecodeError,
            FileNotFoundError):
        pass
    return None


class LXDClient(Provider):
    """LXD client interacting with the LXD socket."""

    def __init__(  # noqa: D107
        self, ros_distro, pro_token=None, remote=None
    ):
        super().__init__(ros_distro)
        if system() != 'Linux':
            raise exceptions.ProviderDoesNotSupportHostOSError(
                'LXDClient is only supported on Linux')

        if not _is_lxd_installed():
            raise exceptions.ProviderNotInstalledOnHostError(
                'LXD is not installed. Please run `sudo snap install lxd`')

        try:
            self.remote = remote
            if remote:
                logger.info(
                    f'Connecting to remote LXD server: {remote}')
                # Get client certificate for authentication
                cert_path, key_path = _get_lxd_client_cert()
                if cert_path and key_path:
                    # Use certificate authentication
                    cert = (cert_path, key_path)
                    logger.debug(
                        f'Using client certificate: {cert_path}')
                    self.lxd_client = Client(
                        endpoint=remote,
                        cert=cert,
                        verify=False  # LXD uses self-signed certs
                    )
                else:
                    # No certificate found, try without cert
                    # (server might allow unauthenticated access)
                    logger.warning(
                        'No client certificate found. '
                        'Attempting connection without certificate. '
                        'If authentication fails, ensure LXD client '
                        'certificates exist in ~/snap/lxd/common/config/ '
                        '(snap) or ~/.config/lxc/ (traditional install).'
                    )
                    self.lxd_client = Client(
                        endpoint=remote,
                        verify=False  # LXD uses self-signed certs
                    )
            else:
                self.lxd_client = Client()
        except pylxd_exceptions.ClientConnectionFailed as e:
            raise exceptions.ProviderClientError(
                'Failed to initialize LXD client. '
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

        config: Dict[str, Any] = {
            'name': self.instance_name,
            'source': {
                'type': 'image',
                'protocol': 'simplestreams',
                'server': cloud_init_url,
                'alias': f'{self.ubuntu_distro}/'
                f'{host_architecture()}',
            },
            'ephemeral': True,
            'config': {},
        }
        config['config']['user.user-data'] \
            = self._render_jinja_template(pro_token)

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

    def clean_instance(self):
        """Clean the created instance."""
        if hasattr(self, 'instance') and self.instance:
            if self.instance.status == 'Running':
                self.instance.stop(wait=True)

    def _is_lxd_initialised(self):
        devices = self.lxd_client.profiles.get('default').devices
        return bool(devices)

    def execute_command(self, command):
        """Execute the given command inside the instance."""
        return self.instance.execute(
            command, stdout_handler=self.logger_instance.debug,
            stderr_handler=self.logger_instance.debug, cwd='/ws'
        ).exit_code

    def _recursive_get(self, remote_path, local_path):
        response = self.instance.files._endpoint.get(
            params={'path': remote_path}, is_api=False)

        if 'X-LXD-type' in response.headers:
            unix_permissions = int(response.headers['X-LXD-mode'], 8)
            if response.headers['X-LXD-type'] == 'directory':
                os.mkdir(local_path, unix_permissions)
                content = json.loads(response.content)
                if 'metadata' in content and content['metadata']:
                    for file in content['metadata']:
                        self._recursive_get(
                            os.path.join(remote_path, file),
                            os.path.join(local_path, file),
                        )
            elif response.headers['X-LXD-type'] == 'file':
                fd = os.open(local_path, os.O_CREAT | os.O_WRONLY,
                             mode=unix_permissions)
                with open(fd, 'wb') as f:
                    f.write(response.content)

    def _recursive_put(self, local_path, remote_path):
        norm_src = os.path.normpath(local_path)
        if not os.path.isdir(norm_src):
            raise NotADirectoryError('src parameter must be a directory')
        idx = len(norm_src)
        dst_items = set()
        for path, dirname, files in os.walk(norm_src):
            dst_path = os.path.normpath(
                os.path.join(remote_path, path[idx:].lstrip(os.path.sep))
            )
            # create directory or symbolic link
            # (depending on what's there)
            if path not in dst_items:
                dst_items.add(path)
                unix_permissions = oct(os.stat(path).st_mode)[-3:]
                headers = self.instance.files._resolve_headers(
                    mode=unix_permissions)
                # determine what the file is:
                # a directory or a symbolic link
                file_mode = os.stat(path).st_mode
                if stat.S_ISLNK(file_mode):
                    headers['X-LXD-type'] = 'symlink'
                else:
                    headers['X-LXD-type'] = 'directory'
                self.instance.files._endpoint.post(params={'path': dst_path},
                                                   headers=headers)
            # copy files
            for f in files:
                src_file = os.path.join(path, f)
                with open(src_file, 'rb') as fp:
                    filepath = os.path.join(dst_path, f)
                    unix_permissions = oct(os.stat(src_file).st_mode)[-3:]
                    self.instance.files.put(filepath, fp.read(),
                                            mode=unix_permissions)

    def _copy_from_instance_to_host(self, *, instance_path, host_path):
        """Copy data from the instance to the host."""
        try:
            self._recursive_get(instance_path, host_path)
        except pylxd_exceptions.NotFound:
            raise exceptions.FileNotFoundInInstanceError(instance_path)

    def _write_in_instance(self, *, instance_file_path, lines):
        """Copy data from the instance to the host."""
        self.instance.files.put(instance_file_path, lines)

    def _copy_from_host_to_instance(self, *, host_path, instance_path):
        """Copy data from the instance to the host."""
        self._recursive_put(host_path, instance_path)

    def shell(self):
        """Shell into the instance."""
        if self.remote:
            # Try to find the remote name for the endpoint
            remote_name = _find_remote_name_for_endpoint(self.remote)
            if remote_name:
                # We found a matching remote, use it
                logger.info(
                    f'Connecting to remote instance using remote: '
                    f'{remote_name}'
                )
                subprocess.run([
                    'lxc', 'exec',
                    f'{remote_name}:{self.instance_name}',
                    '--', 'bash'
                ])
            else:
                # Remote not found in lxc config, provide instructions
                logger.warning(
                    'Remote LXD server detected but no matching remote '
                    'found in lxc configuration. '
                    f'Add the remote using: lxc remote add <name> '
                    f'{self.remote}'
                )
                logger.info(
                    'To shell into the remote instance, use: '
                    f'lxc exec <remote-name>:{self.instance_name} -- bash'
                )
        else:
            subprocess.run(['lxc', 'exec', self.instance_name, '--', 'bash'])
