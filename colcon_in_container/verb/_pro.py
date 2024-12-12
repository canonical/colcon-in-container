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


class Pro(object):
    """Pro tool class wrapper to call pro in a provider."""

    def __init__(self, provider, token):
        """Initialize rosdep and call rosdep init."""
        self.provider = provider
        logger.info('Attaching Pro token')
        #TODO We should not let people enable pro when it's not available
        #TODO We should not enable ROS 2 sources that are enabled in the cloud-init
        # Should we use a different jinja or make it more complex?
        #TODO do we want to integrate rosinstall_generator?
        
        self.provider.execute_command(['pro', 'attach', token])

    def enable(self):
        """Call Pro enable for ROS ESM."""
        logger.info('Enabling ROS ESM')
        commands = [
            'pro enable esm-infra',
            'pro enable esm-apps',
            'apt-get update',
            'apt-get upgrade -y',
            'pro enable ros --beta',
            #TODO This should depends on a flag
            'pro enable ros-updates --beta' 
            
        ]
        return self.provider.execute_commands(commands)
