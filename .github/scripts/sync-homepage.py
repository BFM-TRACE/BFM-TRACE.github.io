#!/usr/bin/env python3
"""Copy tracked homepage sources into this publishing repository."""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def git(directory, *args):
    return subprocess.check_output(['git', '-C', str(directory), *args])


def allowed(relative):
    path = Path(relative)
    return bool(relative) and not path.is_absolute() and '..' not in path.parts and path.parts[0] not in ('.git', '.github')


def main():
    target = Path(__file__).resolve().parents[2]
    source = Path(sys.argv[1]).expanduser().resolve() if len(sys.argv) == 2 else target.parent / 'BFM-Trace' / 'homepage'
    if len(sys.argv) > 2:
        raise SystemExit('Usage: python3 .github/scripts/sync-homepage.py [SOURCE_HOMEPAGE]')
    source = source.resolve()
    if source == target or not (source / 'package.json').is_file():
        raise SystemExit('Expected a separate TRACE homepage source directory containing package.json')
    files = [name.decode('utf-8') for name in git(source, 'ls-files', '-z', '--', '.').split(b'\0') if name]
    if not files or any(not allowed(name) for name in files):
        raise SystemExit('Source has no tracked files or contains reserved/unsafe paths')
    real_source = source.resolve()
    for relative in files:
        file = source / relative
        if not file.is_file() or file.is_symlink() or real_source not in file.resolve().parents:
            raise SystemExit(f'Expected a regular tracked source file: {relative}')

    record_path = target / '.github' / 'homepage-source.json'
    previous = json.loads(record_path.read_text()) if record_path.exists() else {'files': {}}
    old_files = previous['files']
    if any(not allowed(name) for name in old_files):
        raise SystemExit('Previous sync record contains reserved/unsafe paths')
    removed = sorted(set(old_files) - set(files))
    # Delete only files owned by an earlier sync. Deployment configuration is retained.
    for relative in removed:
        file = target / relative
        if file.is_symlink() or (file.exists() and not file.is_file()):
            raise SystemExit(f'Unexpected destination entry: {relative}')
        if file.exists():
            file.unlink()
    hashes = {}
    for relative in files:
        destination = target / relative
        if destination.is_symlink():
            raise SystemExit(f'Refusing to overwrite a destination symlink: {relative}')
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / relative, destination)
        hashes[relative] = hashlib.sha256(destination.read_bytes()).hexdigest()
    record = {
        'schema': 1,
        'sourceCommit': git(source, 'rev-parse', 'HEAD').decode().strip(),
        'scope': 'Tracked homepage files, including their current working-tree edits; .github is maintained by the publishing repository.',
        'files': dict(sorted(hashes.items())),
    }
    record_path.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'copiedFiles': len(files), 'removedFiles': removed, 'sourceCommit': record['sourceCommit']}, indent=2))


if __name__ == '__main__':
    main()
