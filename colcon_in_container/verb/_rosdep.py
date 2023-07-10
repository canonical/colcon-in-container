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

from typing import Set

from colcon_in_container.logging import logger


def call_rosdep(provider, ros_distro, dependency_types: Set[str]):
    """Call rosdep init, update and install provided dependency_types."""
    # initialize and call rosdep over our repository
    logger.info('Initialising and calling rosdep')
    commands = [
        'rosdep init',
        'rosdep update',
        # Avoid rosdep/apt interactive shell error message
        'export DEBIAN_FRONTEND=noninteractive',
        'rosdep install --from-paths /ws/src --ignore-src -y '
        f'--rosdistro={ros_distro} '
    ]
    for dependency_type in dependency_types:
        commands[-1] += f'--dependency-types={dependency_type} '

    return provider.execute_commands(commands)