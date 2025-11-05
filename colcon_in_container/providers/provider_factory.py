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


from typing import Dict

from colcon_in_container.providers import exceptions
from colcon_in_container.providers.lxd import LXDClient
from colcon_in_container.providers.multipass import MultipassClient
from colcon_in_container.providers.provider import Provider


class ProviderFactory(object):
    """Provider factory singleton to make providers."""

    _instance = None
    _providers: Dict[str, Provider] = {}

    def __new__(cls):
        """Singleton creation."""
        if cls._instance is None:
            cls._instance = super(ProviderFactory, cls).__new__(cls)
        return cls._instance

    @classmethod
    def register(cls, name, provider):
        """Register a provider by its name."""
        cls._providers[name] = provider

    @classmethod
    def create(cls, name, ros_distro, pro_token=None, remote=None):
        """Make a provider based on the name."""
        provider = cls._providers.get(name)
        if not provider:
            raise exceptions.ProviderNotRegisteredError(name)
        # Only pass remote to LXD provider
        if name == 'lxd':
            return provider(ros_distro, pro_token, remote)  # type: ignore
        else:
            return provider(ros_distro, pro_token)  # type: ignore


# Register all the providers
ProviderFactory.register('lxd', LXDClient)
ProviderFactory.register('multipass', MultipassClient)
