import argparse
import time
import os
from datetime import datetime
from pylxd import Client

from colcon_core.package_discovery import add_package_discovery_arguments
from colcon_core.package_discovery import discover_packages
from colcon_core.package_descriptor import PackageDescriptor, DependencyDescriptor
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint
from colcon_core.package_identification \
    import get_package_identification_extensions

from colcon_core.location import get_log_path


class ContainerBuildVerb(VerbExtensionPoint):
    """call a colcon command inside a fresh container"""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--ephemeral',
            action='store_true',
            help='An ephemeral container will be deleted after the build')
        add_package_discovery_arguments(parser)

    def execute_command(self, instance, command):
        print(f"+{command}")
        execute_result = instance.execute(command)
        print(execute_result.stdout)
        if execute_result.exit_code != 0:
            print(execute_result.stderr)
        return

    def main(self, *, context):  # noqa: D102
        extension = get_package_identification_extensions()
        lxd_client = Client()
        freshly_created = False
        container_name: str
        if context.args.ephemeral:
            date_time = datetime.today().strftime('%Y-%m-%d-%H-%M-%S')
            container_name=f"colcon-container-build-{date_time}"
        else:
            container_name="colcon-container-build"

        config = {'name': container_name, 'source':
                    {'type': 'image', 'protocol': 'simplestreams',
                    'server': 'https://images.linuxcontainers.org',
                    'alias': 'ubuntu/jammy/amd64'}}
        if not lxd_client.instances.exists(container_name):
            instance = lxd_client.instances.create(config, wait=True)
            freshly_created = True
        else:
            instance = lxd_client.instances.get(container_name)

        if not instance.status == "Running":
            instance.start(wait=True)

        if freshly_created:
            # install ROS 2
            install_ros2 = """#!/bin.bash
                            apt-get update
                            apt-get install -y software-properties-common curl
                            curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
                            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
                            apt-get update
                            apt-get upgrade -y
                            apt-get install -y ros-humble-ros-base
                            apt-get install -y ros-dev-tools"""
            instance.files.put('/tmp/install_ros2', install_ros2)
            self.execute_command(instance, ['bash', '/tmp/install_ros2'])
            self.execute_command(instance, ['mkdir', '-p', '/r2_ws/src'])

        # copy packages into the container
        discovered_packages = discover_packages(context.args, extension)
        for package in discovered_packages:
            print(f"putting {package.path} into /r2_ws/src/")
            container_package_path = f'/r2_ws/src/{package.name}'
            instance.files.mk_dir(container_package_path)
            instance.files.recursive_put(
                package.path, container_package_path)
        
        # install build deps
        install_build_dep = """#!/bin.bash
                            . /opt/ros/humble/setup.bash
                            rosdep init
                            rosdep update
                            rosdep install --from-paths /r2_ws/src --ignore-src -y
                            """
        instance.files.put('/tmp/install_build_dep', install_build_dep)
        self.execute_command(instance, ['bash', '/tmp/install_build_dep'])
        
         # install build deps
        build_ws = """#!/bin.bash
                    . /opt/ros/humble/setup.bash
                    cd /r2_ws && colcon build
                    """
        instance.files.put('/tmp/build_ws', build_ws)
        self.execute_command(instance, ['bash', '/tmp/build_ws'])

        # get results back on host
        if not os.path.exists(container_name):
            os.mkdir(container_name)

        instance.files.recursive_get("/r2_ws/install", os.path.join(container_name, 'install'))


        if context.args.ephemeral:
            instance.stop(wait=True)
            instance.delete()
        return 0
