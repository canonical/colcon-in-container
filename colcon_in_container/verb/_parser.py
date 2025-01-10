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

from colcon_in_container.logging import logger


_ros_distro_choices = ['rolling', 'humble', 'jazzy']
_eol_ros_distro_choices = ['foxy']


def add_ros_distro_argument(parser):
    """Add the --ros-distro argument to the parser."""
    ros_distro_env = getenv('ROS_DISTRO')

    parser.add_argument(
        '--ros-distro',
        metavar='ROS_DISTRO',
        type=str,
        choices=_ros_distro_choices + _eol_ros_distro_choices,
        default=ros_distro_env,
        required=not ros_distro_env,
        help='ROS version, can also be set by the environment variable '
             'ROS_DISTRO.')


def verify_ros_distro_in_parsed_args(args):
    """Verify if the obtained ROS distro is matching the selection."""
    if args.ros_distro not in _ros_distro_choices:
        if args.ros_distro in _eol_ros_distro_choices:
            if not args.pro:
                logger.error('The ros-distro is set to the EoL '
                             'distro {args.ros_distro} '
                             'without any Ubuntu Pro token provided.'
                             'Please provide the `--pro` token argument'
                             'in order to use EoL distro.')
                return False
        else:
            logger.error(f'The ROS_DISTRO={args.ros_distro} '
                         'environment variable is not a viable '
                         '--ros-distro argument. See --ros-distro to set '
                         'a valid ros-distro')
            return False
    return True


def add_instance_argument(parser):
    """Add the instance specific arguments to the parser."""
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

    parser.add_argument(
        '--provider',
        type=str,
        choices=['lxd', 'multipass'],
        default='lxd',
        help='Environment provider.'
    )


def add_pro_arguments(parser):
    """Add the Ubuntu Pro token arguments to the parser."""
    parser.add_argument(
        '--pro',
        type=str,
        help='Ubuntu Pro token to enable inside the instance.',
    )

    parser.add_argument(
        '--auto-deps-management',
        action='store_true',
        help='Automatically manages missing dependencies.'
             'This will retrieve, install and source the '
             'ROS dependencies of the workspace not available in ROS ESM',
    )
