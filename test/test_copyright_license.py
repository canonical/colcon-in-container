# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import sys


def test_copyright_licence():
    missing = check_files([Path(__file__).parents[1]])
    assert not len(
        missing
    ), 'In some files no copyright / license line was found'


def check_files(paths):
    missing = []
    for path in paths:
        if path.is_dir():
            for p in sorted(path.iterdir()):
                if p.name.startswith('.'):
                    continue
                if p.name.endswith('.py') or p.is_dir():
                    missing += check_files([p])
        if path.is_file():
            content = path.read_text()
            if not content:
                continue
            lines = content.splitlines()
            has_copyright = any(
                line for line in lines if line.startswith('# Copyright')
            )

            gnu_v3_license_text = [
                '# it under the terms of the GNU General Public License as '
                'published by',
                '# the Free Software Foundation, either version 3 of the '
                'License, or']

            has_gnu_v3_license = all((line in lines
                                      for line in gnu_v3_license_text))

            has_apache_license = (
                '# Licensed under the Apache License, Version 2.0' in lines
            )
            if not has_copyright or not (
                has_gnu_v3_license or has_apache_license
            ):
                print(
                    'could not find copyright / license in:',
                    path,
                    file=sys.stderr,
                )
                missing.append(path)
            else:
                print('Found copyright / license in:', path)
    return missing
