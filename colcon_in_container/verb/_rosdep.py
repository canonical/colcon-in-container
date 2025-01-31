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

from typing import Optional, Set

from colcon_in_container.logging import logger
from colcon_in_container.verb._parser import _eol_ros_distro_choices


class Rosdep(object):
    """Rosdep tool class wrapper to call rosdep in a provider."""

    def __init__(self, provider, ros_distro):
        """Initialize rosdep and call rosdep init."""
        self.provider = provider
        self.ros_distro = ros_distro
        logger.info('Initialising rosdep')
        self.provider.execute_commands(['rosdep init'])

    def update(self):
        """Call rosdep update."""
        logger.info('Updating rosdep')
        command = 'rosdep update'
        if self.ros_distro in _eol_ros_distro_choices:
            command += ' --include-eol-distros'
        return self.provider.execute_commands([command])

    def install(self,
                workspace='/root/ws/src',
                dependency_types: Optional[Set[str]] = None):
        """Call rosdep install on the provided dependency_types."""
        logger.info('Installing dependencies with rosdep')
        commands = [
            # Avoid rosdep/apt interactive shell error message
            'export DEBIAN_FRONTEND=noninteractive',
            f'rosdep install --from-paths {workspace} --ignore-src -y '
            f'--rosdistro={self.ros_distro} '
        ]
        if dependency_types:
            for dependency_type in dependency_types:
                commands[-1] += f'--dependency-types={dependency_type} '

        return self.provider.execute_commands(commands)
