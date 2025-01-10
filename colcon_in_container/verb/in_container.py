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
from typing import List

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from colcon_in_container.verb._pro import \
    auto_ROS_ESM_dependency_managment, \
    underlay_workspace_path


class InContainer(ABC, VerbExtensionPoint):
    """InContainer verb abstract class."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
        self.host_build_in_container_folder = 'build_in_container'
        self.host_install_in_container_folder = 'install_in_container'
        self.host_build_in_container_underlay_folder = 'build_in_container_underlay'
        self.host_install_in_container_underlay_folder = 'install_in_container_underlay'
        self.host_test_results_folder = 'test_results_in_container'
        self.host_release_in_container_folder = 'release_in_container'
        self.instance_workspace_path = '/root/ws/'

    def _upload_selected_packages(self, package_decorators) -> List[str]:
        """Upload selected packages in the instance.

        Returns the list of uploaded packages.
        """
        package_names = []
        for decorator in package_decorators:
            package = decorator.descriptor
            if not decorator.selected:
                continue
            self.provider.upload_package(package.name, package.path)
            package_names.append(package.name)
        return package_names

    def _ros_esm(self, args):
        if args.pro and args.auto_deps_managment:
            auto_ROS_ESM_dependency_managment(self.provider,
                                                self.rosdep,
                                                args.ros_distro,
                                                self.dependency_types)
            self.provider.download_result(
                result_path_in_instance=underlay_workspace_path
                + 'install',
                result_path_on_host=self.host_install_in_container_underlay_folder)
            self.provider.download_result(
                result_path_in_instance=underlay_workspace_path
                + 'build',
                result_path_on_host=self.host_build_in_container_underlay_folder)
