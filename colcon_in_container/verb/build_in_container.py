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
import sys
from typing import Callable, List

from colcon_core.logging import colcon_logger
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_packages
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint
from colcon_in_container.providers.lxd import LXDClient
from colcon_in_container.verb._parser import \
    add_container_argument, add_ros_distro_argument,\
    verify_ros_distro_in_parsed_args


logger = colcon_logger.getChild(__name__)


class BuildInContainerVerb(VerbExtensionPoint):
    """Call a colcon build command inside a fresh container."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
        self.host_install_folder = 'install_in_container'

    def add_arguments(self, *, parser):  # noqa: D102

        add_ros_distro_argument(parser)

        parser.add_argument(
            '--colcon-build-args',
            default='',
            metavar='*',
            type=str.lstrip,
            help='Pass arguments to the colcon build command.',
        )

        add_container_argument(parser)
        add_packages_arguments(parser)

    def _call_rosdep(self, ros_distro):
        # initialize and call rosdep over our repository
        logger.info('Initialising and calling rosdep')
        commands = [
            'rosdep init',
            'rosdep update',
            # Avoid rosdep/apt interactive shell error message
            'export DEBIAN_FRONTEND=noninteractive',
            'rosdep install --from-paths /ws/src --ignore-src -y '
            f'--rosdistro={ros_distro} '
            '--dependency-types=build '
            '--dependency-types=buildtool '
            '--dependency-types=build_export '
            '--dependency-types=buildtool_export '
            '--dependency-types=test'
        ]

        return self.provider.execute_commands(commands)

    def _colcon_build(self, colcon_build_args):
        logger.info(f'building workspace with args: {colcon_build_args}')
        return self.provider.execute_commands([
            f'colcon --log-level={logger.getEffectiveLevel()} '
            f'build {colcon_build_args}'])

    def _build(self, args):
        """Build the workspace.

        Pull build-time dependencies, build the workspace and download the
        result build directory.
        """
        commands: List[Callable[[], int]] = [
            partial(self._call_rosdep, args.ros_distro),
            partial(self._colcon_build, args.colcon_build_args)]
        for command in commands:
            exit_code = command()
            if exit_code:
                return exit_code

        try:
            self.provider.download_result(
                result_path_in_container='/ws/install',
                result_path_on_host=self.host_install_folder)
        except FileNotFoundError:
            return 1
        return 0

    def main(self, *, context):  # noqa: D102

        if not verify_ros_distro_in_parsed_args(context.args):
            sys.exit(1)

        try:
            self.provider = LXDClient(context.args.ros_distro)
        except SystemError as e:
            logger.error(f'Failed to start the LXD client: {e}')
            return sys.exit(1)

        # copy packages into the container
        decorators = get_packages(context.args, recursive_categories=('run', ))
        logger.info(f'Discovered {len(decorators)}, '
                    'uploading them in the container')
        for decorator in decorators:
            package = decorator.descriptor
            if not decorator.selected:
                continue
            self.provider.upload_package(package.path)

        build_exit_code = self._build(context.args)
        if build_exit_code and context.args.debug:
            logger.error(f'Build failed with error code {build_exit_code}.')
            logger.warn('Debug was selected, entering the container.')
            self.provider.shell()
        elif context.args.shell_after:
            logger.info('Shell after was selected, entering the container.')
            self.provider.shell()

        return build_exit_code
