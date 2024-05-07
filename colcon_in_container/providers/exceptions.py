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


class ProviderNotRegisteredError(Exception):
    """Exception raised when creating an unregistered provider."""


class FileNotFoundInInstanceError(Exception):
    """Exception raised when the file does not existing in the instance."""

    def __init__(self, file_name):
        """Message for a file not found."""
        super().__init__(
            f'File {file_name} is not available inside the instance')


class FileNotFoundInHostError(Exception):
    """Exception raised when the file does not existing on the host."""

    def __init__(self, file_name):
        """Message for a file not found."""
        super().__init__(
            f'File {file_name} is not available on host')


class ProviderDoesNotSupportHostOSError(Exception):
    """Exception raised when the provider is not supported on the host."""


class ProviderNotInstalledOnHostError(Exception):
    """Exception raised when the provider is not installed on the host."""


class ProviderNotConfiguredError(Exception):
    """Exception raised when the provider is not configured."""


class ProviderClientError(Exception):
    """Exception raised when the provider cannot connect to the client."""


class CloudInitError(Exception):
    """Exception raised when the cloud-init failed in the client."""
