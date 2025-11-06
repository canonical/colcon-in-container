# Copyright (C) 2023, Canonical, Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""LXD provider for colcon-in-container."""

import json
import os
from platform import system
import select
import shutil
import stat
import sys
import termios
import threading
import tty
from typing import Any, Dict

from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions
from colcon_in_container.providers._helper \
    import host_architecture
from colcon_in_container.providers.provider import Provider
from pylxd import Client, exceptions as pylxd_exceptions
from ws4py.client import WebSocketBaseClient


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

        try:
            self.lxd_client = Client()
        except pylxd_exceptions.ClientConnectionFailed as e:
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
        """Shell into the instance using pylxd interactive execute."""
        # Get websocket URLs for interactive session
        ws_info = self.instance.raw_interactive_execute(
            ['bash'],
            environment={'TERM': os.environ.get('TERM', 'xterm')},
            cwd='/ws'
        )

        # Get the base websocket URL
        websocket_url = self.lxd_client.websocket_url

        # Event to signal connection is ready
        connection_ready = threading.Event()
        connection_error = threading.Event()

        # Interactive websocket client for the shell
        class InteractiveShellClient(WebSocketBaseClient):
            """Websocket client for interactive shell sessions."""

            def __init__(self, url, ssl_options=None):
                """Initialize the interactive shell client."""
                super().__init__(url, ssl_options=ssl_options)
                self.stdin_fd = sys.stdin.fileno()
                self.old_tty_settings = None
                self.is_connected = False

            def opened(self):
                """Set terminal to raw mode when connection established."""
                try:
                    self.old_tty_settings = termios.tcgetattr(self.stdin_fd)
                    tty.setraw(self.stdin_fd)
                    self.is_connected = True
                    connection_ready.set()
                except (OSError, IOError, termios.error) as e:
                    logger.error(f'Failed to set terminal to raw mode: {e}')
                    connection_error.set()

            def received_message(self, message):
                """Write received messages from container to stdout."""
                try:
                    if message.is_binary:
                        sys.stdout.buffer.write(message.data)
                    else:
                        sys.stdout.write(message.data.decode('utf-8'))
                    sys.stdout.flush()
                except (OSError, IOError) as e:
                    logger.debug(f'Error writing to stdout: {e}')
                    self.close()

            def closed(self, code, reason=None):
                """Restore terminal settings when connection closes."""
                self.is_connected = False
                if self.old_tty_settings:
                    try:
                        termios.tcsetattr(
                            self.stdin_fd,
                            termios.TCSADRAIN,
                            self.old_tty_settings)
                    except (OSError, IOError, termios.error):
                        pass

        # Create websocket client
        ws_client = InteractiveShellClient(
            websocket_url,
            ssl_options=self.lxd_client.ssl_options if hasattr(
                self.lxd_client, 'ssl_options') else None
        )
        ws_client.resource = ws_info['ws']

        try:
            # Establish connection first
            ws_client.connect()

            # Run message loop in a separate thread
            ws_thread = threading.Thread(target=ws_client.run)
            ws_thread.daemon = True
            ws_thread.start()

            # Wait for connection to be established
            if not connection_ready.wait(timeout=5):
                if connection_error.is_set():
                    raise exceptions.ProviderClientError(
                        'Failed to establish websocket connection: '
                        'terminal setup failed'
                    )
                raise exceptions.ProviderClientError(
                    'Failed to establish websocket connection: timeout'
                )

            # Main loop: read from stdin and send to websocket
            stdin_fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(stdin_fd)

            try:
                while ws_client.is_connected and ws_thread.is_alive():
                    # Use select to wait for data on stdin
                    rlist = [stdin_fd]
                    readable, _, _ = select.select(rlist, [], [], 0.1)

                    if stdin_fd in readable:
                        # Read from stdin and send to websocket
                        try:
                            data = os.read(stdin_fd, 4096)
                            if data:
                                ws_client.send(data, binary=True)
                            else:
                                # EOF received
                                break
                        except OSError as e:
                            logger.debug(f'Error reading from stdin: {e}')
                            break

            finally:
                # Restore terminal settings before closing
                try:
                    termios.tcsetattr(
                        stdin_fd, termios.TCSADRAIN, old_settings)
                except (OSError, IOError, termios.error):
                    pass

                # Close websocket gracefully
                try:
                    ws_client.close()
                except (OSError, IOError, ConnectionError) as e:
                    logger.debug(f'Error closing websocket: {e}')

                # Wait for thread to finish
                ws_thread.join(timeout=1)

        except (OSError, IOError, ConnectionError) as e:
            logger.error(f'Failed to establish interactive shell: {e}')
            raise exceptions.ProviderClientError(
                f'Failed to establish interactive shell: {e}'
            )
