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

from abc import ABC

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


class InContainer(ABC, VerbExtensionPoint):
    """InContainer verb abstract class."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
        self.host_build_in_container_folder = 'build_in_container'
        self.host_install_in_container_folder = 'install_in_container'
        self.host_test_results_folder = 'test_results_in_container'
        self.instance_workspace_path = '/ws/'
