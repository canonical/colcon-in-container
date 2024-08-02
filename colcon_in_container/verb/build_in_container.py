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


from functools import partial
import sys
from typing import Callable, List

from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_packages
from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions as provider_exceptions
from colcon_in_container.providers.provider_factory import ProviderFactory
from colcon_in_container.verb._parser import \
    add_instance_argument, add_ros_distro_argument,\
    verify_ros_distro_in_parsed_args
from colcon_in_container.verb._rosdep import Rosdep
from colcon_in_container.verb.in_container import InContainer


class BuildInContainerVerb(InContainer):
    """Call a colcon build command inside a fresh container."""

    def __init__(self):  # noqa: D107
        super().__init__()

    def add_arguments(self, *, parser):  # noqa: D102

        add_ros_distro_argument(parser)

        parser.add_argument(
            '--colcon-build-args',
            default='',
            metavar='*',
            type=str.lstrip,
            help='Pass arguments to the colcon build command.'
            'Arguments matching other options must be prefixed by a space.',
        )

        add_instance_argument(parser)
        add_packages_arguments(parser)

    def _colcon_build(self, colcon_build_args):
        logger.info(f'building workspace with args: {colcon_build_args}')
        return self.provider.execute_commands([
            f'colcon --log-level={logger.getEffectiveLevel()} '
            f'build {colcon_build_args}'])

    def _build(self, args):
        """Build the workspace.

        Pull build-time dependencies, build the workspace and download the
        result build directory.
        """
        commands: List[Callable[[], int]] = [
            partial(self.rosdep.install,
                    ['build',
                     'buildtool',
                     'build_export',
                     'buildtool_export',
                     'test']),
            partial(self._colcon_build, args.colcon_build_args)]
        for command in commands:
            exit_code = command()
            if exit_code:
                return exit_code

        try:
            self.provider.download_result(
                result_path_in_instance=self.instance_workspace_path
                + 'install',
                result_path_on_host=self.host_install_in_container_folder)
            self.provider.download_result(
                result_path_in_instance=self.instance_workspace_path
                + 'build',
                result_path_on_host=self.host_build_in_container_folder)
        except provider_exceptions.FileNotFoundInInstanceError:
            return 1
        return 0

    def main(self, *, context):  # noqa: D102

        if not verify_ros_distro_in_parsed_args(context.args):
            sys.exit(1)

        self.provider = ProviderFactory.create(context.args.provider,
                                               context.args.ros_distro)
        try:
            self.provider.wait_for_install()
        except provider_exceptions.CloudInitError as e:
            logger.error(e)
            # temporary until error management is migrated to exceptions
            exit_code = 1
        else:
            self.rosdep = Rosdep(self.provider, context.args.ros_distro)
            self.rosdep.update()
            # copy packages into the instance
            decorators = get_packages(context.args,
                                      recursive_categories=('run', ))
            logger.info(f'Discovered {len(decorators)} packages, '
                        'uploading them in the instance')
            for decorator in decorators:
                package = decorator.descriptor
                if not decorator.selected:
                    continue
                self.provider.upload_package(package.path)
            exit_code = self._build(context.args)

        if exit_code != 0:
            logger.error(f'Build failed with exit code {exit_code}. ')
            if context.args.debug:
                logger.warn('Debug was selected, entering the instance.')
                self.provider.shell()
            sys.exit(1)
        elif context.args.shell_after:
            logger.info('Shell after was selected, entering the instance.')
            self.provider.shell()

        logger.info('Successfully built workspace in container.')
        return exit_code
