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

underlay_workspace_path = '/root/ws_underlay/'

def auto_ROS_ESM_dependency_managment(provider, rosdep, ros_distro, dependency_types: Optional[Set[str]] = None):
    # Create the second workspace
    def _create_underlay_workspace():
        exit_code = provider.execute_commands([
            f'ros-esm-dependencies-diff-generator --rosdistro { ros_distro } -o /root/dependencies.rosinstall --source /root/ws/src/',
            f'cat /root/dependencies.rosinstall',
            f'vcs import --shallow {underlay_workspace_path}/src < /root/dependencies.rosinstall'])
        if exit_code:
            raise SystemError('Failed to create the underlay workspace '
                              f'with exit code {exit_code}')

    def _build_dependencies():
        exit_code = rosdep.install(workspace=f'{underlay_workspace_path}/src', dependency_types=dependency_types)
        if exit_code:
            raise SystemError('Failed to rosdep install underlay workspace dependencies '
                              f'with exit code {exit_code}')
        exit_code = provider.execute_commands([
            f'cd {underlay_workspace_path}',
            f'colcon --log-level={logger.getEffectiveLevel()} '
            f'build --cmake-args -DCMAKE_BUILD_TYPE=Release'])
        if exit_code:
            raise SystemError('Failed to build underlay workspace dependencies '
                              f'with exit code {exit_code}')

    _create_underlay_workspace()
    _build_dependencies()

