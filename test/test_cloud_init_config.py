# Copyright (C) 2025 Canonical, Ltd.

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

"""Test cloud-init configuration."""

from pathlib import Path

import jinja2


def test_cloud_init_uses_hkps_keyserver():
    """Verify that cloud-init config uses hkps:// keyserver."""
    # Get the path to the cloud-init.yaml file
    repo_root = Path(__file__).parents[1]
    cloud_init_file = repo_root / 'colcon_in_container' / 'providers' / \
        'config' / 'cloud-init.yaml'

    assert cloud_init_file.exists(), \
        f'Cloud-init file not found at {cloud_init_file}'

    # Read the template
    with open(cloud_init_file, 'r') as f:
        config = f.read()

    # Render the template without pro_token (the path that uses apt sources)
    template = jinja2.Environment().from_string(source=config)
    configuration = {
        'architecture': 'amd64',
        'distro_release': 'jammy'
    }
    rendered = template.render(configuration)

    # Verify the keyserver parameter is present and uses hkps:// with port 443
    assert 'keyserver: hkps://keyserver.ubuntu.com:443' in rendered, \
        'Cloud-init config must include hkps:// keyserver with port 443 ' \
        'to bypass firewall blocks on port 11371'

    # Also verify keyid is still present
    assert 'keyid:' in rendered, \
        'Cloud-init config must include keyid'


def test_cloud_init_pro_token_path_does_not_need_keyserver():
    """Verify that Ubuntu Pro path doesn't use apt sources with keyserver."""
    # Get the path to the cloud-init.yaml file
    repo_root = Path(__file__).parents[1]
    cloud_init_file = repo_root / 'colcon_in_container' / 'providers' / \
        'config' / 'cloud-init.yaml'

    # Read the template
    with open(cloud_init_file, 'r') as f:
        config = f.read()

    # Render the template with pro_token (the path that uses Ubuntu Pro)
    template = jinja2.Environment().from_string(source=config)
    configuration = {
        'architecture': 'amd64',
        'distro_release': 'jammy',
        'pro_token': 'test-token'
    }
    rendered = template.render(configuration)

    # Verify Ubuntu Pro is enabled and apt sources are not used
    assert 'ubuntu_pro:' in rendered, \
        'Cloud-init with pro_token should use ubuntu_pro'
    assert 'keyserver:' not in rendered, \
        'Cloud-init with pro_token should not include apt keyserver config'
    assert 'keyid:' not in rendered, \
        'Cloud-init with pro_token should not include apt keyid config'
