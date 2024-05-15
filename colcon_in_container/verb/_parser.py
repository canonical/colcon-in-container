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


_ros_distro_choices = ['rolling', 'iron', 'humble']


def add_ros_distro_argument(parser):
    """Add the --ros-distro argument to the parser."""
    ros_distro_env = getenv('ROS_DISTRO')

    parser.add_argument(
        '--ros-distro',
        metavar='ROS_DISTRO',
        type=str,
        choices=_ros_distro_choices,
        default=ros_distro_env,
        required=not ros_distro_env,
        help='ROS version, can also be set by the environment variable '
             'ROS_DISTRO.')


def verify_ros_distro_in_parsed_args(args):
    """Verify if the obtained ROS distro is matching the selection."""
    if args.ros_distro not in _ros_distro_choices:
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
