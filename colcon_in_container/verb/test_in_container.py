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


class TestInContainerVerb(InContainer):
    """Call a colcon test command inside a fresh container."""

    __test__ = False

    def __init__(self):  # noqa: D107
        super().__init__()

    def add_arguments(self, *, parser):  # noqa: D102

        add_ros_distro_argument(parser)

        parser.add_argument(
            '--colcon-test-args',
            default='',
            metavar='*',
            type=str.lstrip,
            help='Pass arguments to the colcon test command. '
            'Arguments matching other options must be prefixed by a space.',
        )

        add_instance_argument(parser)
        add_packages_arguments(parser)

    def _colcon_test(self, colcon_test_args):
        logger.info(f'testing workspace with args: {colcon_test_args}')
        return self.provider.execute_commands([
            f'colcon --log-level={logger.getEffectiveLevel()} '
            f'test --test-result-base="test_results" {colcon_test_args}'])

    def _test(self, args):
        """Test the workspace.

        Pull test dependencies, test the workspace and download the
        result test directories.
        """
        commands: List[Callable[[], int]] = [
            partial(self.rosdep.install,
                    ['exec', 'test']),
            partial(self._colcon_test, args.colcon_test_args)]
        for command in commands:
            exit_code = command()
            if exit_code:
                return exit_code

        try:
            self.provider.download_result(
                result_path_in_instance=self.instance_workspace_path
                + 'test_results',
                result_path_on_host=self.host_test_results_folder)
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
            exit_code = 1
        else:
            self.rosdep = Rosdep(self.provider, context.args.ros_distro)
            self.rosdep.update()
            # copy packages into the instance
            decorators = get_packages(context.args, recursive_categories=('run', ))
            logger.info(f'Discovered {len(decorators)} packages, '
                        'uploading them in the instance')
            for decorator in decorators:
                package = decorator.descriptor
                if not decorator.selected:
                    continue
                self.provider.upload_package(package.path)

            # upload build and install folder
            self.provider.upload_directory(
                host_path=self.host_build_in_container_folder,
                instance_path=self.instance_workspace_path + 'build')
            self.provider.upload_directory(
                host_path=self.host_install_in_container_folder,
                instance_path=self.instance_workspace_path + 'install')

            exit_code = self._test(context.args)

        if exit_code and context.args.debug:
            logger.error(f'Test failed with error code {exit_code}.')
            logger.warn('Debug was selected, entering the instance.')
            self.provider.shell()
        elif context.args.shell_after:
            logger.info('Shell after was selected, entering the instance.')
            self.provider.shell()

        return exit_code
