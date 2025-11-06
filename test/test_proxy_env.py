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

import os
import unittest
from unittest.mock import patch

from colcon_in_container.providers._helper import (
    get_proxy_environment_variables,
)


class TestProxyEnvironmentVariables(unittest.TestCase):
    """Test proxy environment variable detection."""

    def test_get_proxy_env_vars_when_none_set(self):
        """Test when no proxy variables are set."""
        with patch.dict(os.environ, {}, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, {})

    def test_get_proxy_env_vars_with_http_proxy(self):
        """Test when http_proxy is set."""
        test_env = {'http_proxy': 'http://proxy.example.com:8080'}
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_with_https_proxy(self):
        """Test when https_proxy is set."""
        test_env = {'https_proxy': 'http://proxy.example.com:8443'}
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_with_no_proxy(self):
        """Test when no_proxy is set."""
        test_env = {'no_proxy': 'localhost,127.0.0.1,.example.com'}
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_with_all_vars(self):
        """Test when all proxy variables are set."""
        test_env = {
            'http_proxy': 'http://proxy.example.com:8080',
            'https_proxy': 'http://proxy.example.com:8443',
            'no_proxy': 'localhost,127.0.0.1',
        }
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_uppercase(self):
        """Test when uppercase proxy variables are set."""
        test_env = {
            'HTTP_PROXY': 'http://proxy.example.com:8080',
            'HTTPS_PROXY': 'http://proxy.example.com:8443',
            'NO_PROXY': 'localhost',
        }
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_mixed_case(self):
        """Test when both lowercase and uppercase variables are set."""
        test_env = {
            'http_proxy': 'http://proxy.example.com:8080',
            'HTTP_PROXY': 'http://proxy.example.com:8080',
            'https_proxy': 'http://proxy.example.com:8443',
            'NO_PROXY': 'localhost',
        }
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, test_env)

    def test_get_proxy_env_vars_ignores_other_vars(self):
        """Test that non-proxy variables are ignored."""
        test_env = {
            'http_proxy': 'http://proxy.example.com:8080',
            'PATH': '/usr/bin:/bin',
            'HOME': '/home/user',
        }
        expected = {'http_proxy': 'http://proxy.example.com:8080'}
        with patch.dict(os.environ, test_env, clear=True):
            result = get_proxy_environment_variables()
            self.assertEqual(result, expected)
