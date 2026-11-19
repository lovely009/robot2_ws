#!/usr/bin/env python3
"""Build a local compatibility loader alongside the unmodified vendor SDK."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def build(library):
    library = Path(library).expanduser().absolute()
    if not library.is_file():
        raise FileNotFoundError(f'找不到原厂 SDK: {library}')
    compiler = shutil.which('cc')
    if compiler is None:
        raise RuntimeError('需要 C 编译器 cc（Ubuntu 的 build-essential 软件包）')
    output = library.with_name(f'{library.stem}_robot2.so')
    source = Path(__file__).with_name('usbcanfd_compat.c')
    fd, temporary = tempfile.mkstemp(prefix='.usbcanfd-', suffix='.so',
                                     dir=library.parent)
    os.close(fd)
    try:
        subprocess.run([
            compiler, '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
            str(source), f'-L{library.parent}', '-Wl,-z,defs',
            '-Wl,-rpath,$ORIGIN', '-Wl,--no-as-needed', f'-l:{library.name}',
            '-ldl', '-o', temporary,
        ], check=True)
        os.chmod(temporary, 0o644)
        os.replace(temporary, output)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', nargs='?',
                        default=Path.home() / '.local/lib/robot2/libcontrolcanfd.so')
    args = parser.parse_args()
    try:
        print(build(args.library))
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'{error}\n')
