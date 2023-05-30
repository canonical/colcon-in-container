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
from colcon_core.package_discovery import add_package_discovery_arguments
from colcon_core.package_discovery import discover_packages
from colcon_core.package_identification \
    import get_package_identification_extensions
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


logger = colcon_logger.getChild(__name__)

ros_distro_choices = ['humble', 'foxy']


class BuildInContainerVerb(VerbExtensionPoint):
    """call a colcon command inside a fresh container."""

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
            help='Pass arguments to the colcon build command',
        )

        parser.add_argument(
            '--debug',
            action='store_true',
            help='Shell into the environment at the end of the build or if '
                  'there is an error',
        )

        add_package_discovery_arguments(parser)

    def main(self, *, context):  # noqa: D102

        if context.args.ros_distro not in ros_distro_choices:
            logger.error(f'The ROS_DISTRO={context.args.ros_distro} '
                         'environement variable is not a viable '
                         '--ros-distro argument. See --ros-distro to set '
                         'valid ros-distro')
            sys.exit(1)

        lxd_client = LXDClient(context.args.ros_distro)

        # copy packages into the container
        extension = get_package_identification_extensions()
        discovered_packages = discover_packages(context.args, extension)
        logger.info(f'Discovered {len(discovered_packages)}, '
                    'uploading them in the container')
        for package in discovered_packages:
            lxd_client.upload_package(package.name, package.path)

        lxd_client.build(context.args.colcon_build_args)

        if context.args.debug:
            logger.info(f'Debug was selected, entering the container.')
            lxd_client.shell()

        return 0
