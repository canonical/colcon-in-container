# Copyright (C) 2024 Canonical, Ltd.

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
import os
import sys
from typing import Callable, List

from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_packages
from colcon_in_container.logging import logger
from colcon_in_container.providers import exceptions as provider_exceptions
from colcon_in_container.providers.provider_factory import ProviderFactory
from colcon_in_container.verb._parser import \
    add_instance_argument, add_pro_arguments, \
    add_ros_distro_argument, \
    verify_ros_distro_in_parsed_args
from colcon_in_container.verb._pro import auto_ros_esm_dependency_management
from colcon_in_container.verb._rosdep import Rosdep
from colcon_in_container.verb.in_container import InContainer


class ReleaseInContainerVerb(InContainer):
    """Build a Debian package inside a fresh container."""

    def __init__(self):  # noqa: D107
        super().__init__()
        self.dependency_types = {'build',
                                 'buildtool',
                                 'build_export',
                                 'buildtool_export',
                                 'test'}
        self.packages_from_underlay_ws = []

    def add_arguments(self, *, parser):  # noqa: D102

        add_ros_distro_argument(parser)
        add_instance_argument(parser)
        add_packages_arguments(parser)
        add_pro_arguments(parser)

        parser.add_argument(
            '--bloom-generator',
            default='rosdebian',
            type=str,
            choices=['debian', 'rosdebian'],
            required=False,
            help='Pass arguments to the bloom-generate command.',
        )

        parser.add_argument(
            '--install-released-packages',
            action='store_true',
            help='Progressively install released packages '
                 'so that dependencies are satisfied.',
        )

    def _install_bloom_dependencies(self, ros_distro):
        """Install bloom dependencies in the instance.

        Install all the bloom release dependencies in the instance.

        Raise: ChildProcessError if the install fails.
        """
        exit_code = self.provider.execute_commands([
            'apt install --yes '
            'python3-bloom '
            'python3-rosdep '
            'fakeroot '
            'debhelper '
            'dh-python '
            # necessary to set ROS_DISTRO
            f'ros-{ros_distro}-ros-environment'
        ])
        if exit_code:
            raise ChildProcessError('Failed to install bloom dependencies '
                                    f'with error code {exit_code}')

    def _bloom_generate(self, package_name, generator):
        logger.info(f'Bloom generating {package_name}')
        return self.provider.execute_commands([
            f'cd {self.instance_workspace_path}/src/{package_name}',
            f'bloom-generate {generator}'])

    def _generate_binary(self, package_name):
        logger.info(f'Generating binary for {package_name}')
        return self.provider.execute_commands([
            f'cd {self.instance_workspace_path}/src/{package_name}',
            'fakeroot debian/rules binary'])

    def _save_results(self, package_name):
        logger.info(f'Saving results for {package_name}')
        return self.provider.execute_commands([
            f'mkdir -p {self.instance_workspace_path}/release/{package_name}',
            f'mv {self.instance_workspace_path}/src/{package_name}/debian '
            f'{self.instance_workspace_path}/release/{package_name}',
            f'mv {self.instance_workspace_path}/src/*.deb '
            f'{self.instance_workspace_path}/release/{package_name}',
            # not every package have .ddeb file
            f'mv {self.instance_workspace_path}/src/*.ddeb '
            f'{self.instance_workspace_path}/release/{package_name} || true'])

    def _add_colcon_ignore(self):
        """Add COLCON_IGNORE to the results.

        Add COLCON_IGNORE to the downloaded results.
        This way the results won't get detected as sources
        """
        file_path = self.host_release_in_container_folder + '/COLCON_IGNORE'
        with open(file_path, 'a'):
            os.utime(file_path)

    def _release_package(self, package_name, args):
        """Release the package.

        Run bloom-generate rosdebian|debian.

        Raise: ChildProcessError when a command fails in the container.
        """
        commands: List[Callable[[], int]] = [
            partial(self.rosdep.install,
                    dependency_types=self.dependency_types),
            partial(self._bloom_generate, package_name, args.bloom_generator),
            partial(self._generate_binary, package_name),
            partial(self._save_results, package_name)]

        for command in commands:
            exit_code = command()
            if exit_code:
                raise ChildProcessError(f'Failed to run command: {command} in '
                                        'container.'
                                        f'Error code is {exit_code}')

    def _download_packages_results(self):
        """Download the package release results.

        Download the package release results back on the host.

        Raise: FileNotFoundError when provided path doesn't exist.
        """
        try:
            self.provider.download_result(
                result_path_in_instance=self.instance_workspace_path
                + 'release',
                result_path_on_host=self.host_release_in_container_folder)
        except provider_exceptions.FileNotFoundInInstanceError as e:
            raise FileNotFoundError(str(e))

        # add colcon_ignore in the results so the source
        # from release won't get detected
        self._add_colcon_ignore()

    def _install_package(self, package_name):
        """Install the package on the system.

        Install the package on the system so it's available for
        the packages to be released.
        For now, only Debian packages are supported.

        Raise: ChildProcessError when the installation fails.
        """
        package_to_install = (f'{self.instance_workspace_path}/release/'
                              f'{package_name}/*.deb')
        logger.debug(f'Installing {package_to_install} file.')
        # We ignore the dependencies so the packages from the
        # underlay workspace are not causing a
        # missing dependency error.
        exit_code = self.provider.execute_commands([
            f'dpkg --install --force-depends {package_to_install}'])
        if exit_code:
            raise ChildProcessError('Failed to install released '
                                    f'package {package_name}: {exit_code}')

    def main(self, *, context):  # noqa: D102

        if not verify_ros_distro_in_parsed_args(context.args):
            sys.exit(1)

        self.provider = ProviderFactory.create(context.args.provider,
                                               context.args.ros_distro,
                                               context.args.pro)
        try:
            self.provider.wait_for_install()

            self.rosdep = Rosdep(self.provider, context.args.ros_distro)
            self.rosdep.update()

            try:
                self._install_bloom_dependencies(context.args.ros_distro)
            except ChildProcessError as e:
                logger.error(str(e))

            # copy packages into the instance
            decorators = get_packages(context.args,
                                      recursive_categories=('run', ))
            logger.info(f'Discovered {len(decorators)} packages, '
                        'uploading the selected ones in the instance')

            package_names = self._upload_selected_packages(decorators)
            if not package_names:
                raise FileNotFoundError('No package found for release')

            if context.args.pro and context.args.auto_deps_management:
                auto_ros_esm_dependency_management(self.provider,
                                                   self.rosdep,
                                                   context.args.ros_distro,
                                                   self.dependency_types)

            for package in package_names:
                try:
                    self._release_package(package, context.args)
                except ChildProcessError as e:
                    raise SystemError(f'Release package {package} '
                                      f'failed: {str(e)}')
                except FileNotFoundError as e:
                    raise SystemError(f'Download {package} result '
                                      f'failed: {str(e)}')
                logger.info(f'Package {package} released!')

                if context.args.install_released_packages:
                    logger.info(f'Installing {package} package.')
                    self._install_package(package)

            self._download_packages_results()

        except (ChildProcessError,
                SystemError,
                FileNotFoundError,
                provider_exceptions.CloudInitError) as e:
            logger.error(str(e))
            if context.args.debug or context.args.shell_after:
                logger.warning('Debug was selected, entering the instance')
                self.provider.shell()
                sys.exit(1)

        if context.args.shell_after:
            logger.info('Shell after was selected, entering the instance')
            self.provider.shell()

        self.provider.clean_instance()

        sys.exit(0)
