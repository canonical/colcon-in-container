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


from os import getenv
import sys

from colcon_build_in_container.lxd import LXDClient
from colcon_core.logging import colcon_logger
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_packages
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


logger = colcon_logger.getChild(__name__)

ros_distro_choices = ['humble', 'foxy']


class BuildInContainerVerb(VerbExtensionPoint):
    """Call a colcon build command inside a fresh container."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102

        ros_distro_env = getenv('ROS_DISTRO')

        parser.add_argument(
            '--ros-distro',
            metavar='ROS_DISTRO',
            type=str,
            choices=ros_distro_choices,
            default=ros_distro_env,
            required=not ros_distro_env,
            help='ROS version, can also be set by the environment variable '
                 'ROS_DISTRO.'
        )
        parser.add_argument(
            '--colcon-build-args',
            default='',
            metavar='*',
            type=str.lstrip,
            help='Pass arguments to the colcon build command.',
        )
        parser.add_argument(
            '--debug',
            action='store_true',
            help='Shell into the environment in case the build fails.',
        )
        parser.add_argument(
            '--shell-after',
            action='store_true',
            help='Shell into the environment at the end of the build or if '
                 'there is an error. This flag includes "--debug".',
        )
        add_packages_arguments(parser)

    def main(self, *, context):  # noqa: D102

        if context.args.ros_distro not in ros_distro_choices:
            logger.error(f'The ROS_DISTRO={context.args.ros_distro} '
                         'environment variable is not a viable '
                         '--ros-distro argument. See --ros-distro to set '
                         'a valid ros-distro')
            sys.exit(1)

        lxd_client = LXDClient(context.args.ros_distro)

        # copy packages into the container
        decorators = get_packages(context.args, recursive_categories=('run', ))
        logger.info(f'Discovered {len(decorators)}, '
                    'uploading them in the container')
        for decorator in decorators:
            package = decorator.descriptor
            if not decorator.selected:
                continue
            lxd_client.upload_package(package.name, package.path)

        if error_code := lxd_client.build(context.args.colcon_build_args) \
                and context.args.debug:
            logger.error(f'Build failed with error code {error_code}.')
            logger.warn('Debug was selected, entering the container.')
            lxd_client.shell()
        elif context.args.shell_after:
            logger.info('Shell after was selected, entering the container.')
            lxd_client.shell()

        return 0
