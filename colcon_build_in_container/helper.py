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

from platform import processor


_ros2_ubuntu_distro = {'rolling': 'jammy',
                       'iron': 'jammy',
                       'humble': 'jammy',
                       'foxy': 'focal'}


def get_ubuntu_distro(ros_distro):
    """Return the Ubuntu distro associated to the ROS distro."""
    return _ros2_ubuntu_distro[ros_distro]


def host_architecture():
    """Return the host CPU architecture."""
    processor_architecture = {'x86_64': 'amd64',
                              'aarch64': 'arm64'}
    host_processor = processor()
    if host_processor not in processor_architecture:
        raise SystemError(f'Architecture {host_processor} is not supported')

    return processor_architecture[host_processor]
