from platform import processor

ros2_ubuntu_distro = {'humble': 'jammy',
                      'foxy': 'focal'}

def get_ubuntu_distro(ros_distro):
    return ros2_ubuntu_distro[ros_distro]


def host_architecture():
    processor_architecture = {'x86_64': 'amd64',
                              'aarch64': 'arm64'}
    host_processor = processor()
    if host_processor not in processor_architecture:
        raise SystemError(f'Architecture {host_processor} is not supported')
    
    return processor_architecture[host_processor]
