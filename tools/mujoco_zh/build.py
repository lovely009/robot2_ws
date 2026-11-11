#!/usr/bin/env python3
"""Build an isolated Chinese renderer against the installed MuJoCo 3.6.0 ABI."""

import hashlib
from importlib.metadata import distribution
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
CACHE = Path.home() / '.cache/robot2-mujoco-zh'
SOURCE = CACHE / 'source-3.6.0'
BUILD = CACHE / 'build'
RUNTIME = CACHE / 'runtime'


def run(*arguments):
    subprocess.run([str(value) for value in arguments], check=True)


def original(relative):
    return subprocess.check_output(
        ['git', '-C', str(SOURCE), 'show', f'HEAD:{relative}'], text=True)


def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError(f'Unsupported upstream source near {before[:70]!r}')
    return source.replace(before, after, 1)


def build():
    installed = distribution('mujoco')
    if installed.version != '3.6.0':
        raise ValueError('This localization is pinned to MuJoCo 3.6.0')
    package = Path(installed.locate_file('mujoco'))
    if not (SOURCE / '.git').exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        run('git', 'clone', '--depth', '1', '--filter=blob:none', '--sparse',
            '--no-checkout', '--branch', '3.6.0',
            'https://github.com/google-deepmind/mujoco.git', SOURCE)
        run('git', '-C', SOURCE, 'sparse-checkout', 'set', '--no-cone',
            '/src/', '/include/', '/simulate/', '/LICENSE')
        run('git', '-C', SOURCE, 'checkout', '3.6.0')
    for header in ('mjrender.h', 'mjui.h'):
        if (SOURCE / 'include/mujoco' / header).read_bytes() != (package / 'include/mujoco' / header).read_bytes():
            raise ValueError(f'Installed rendering ABI differs at {header}')
    BUILD.mkdir(parents=True, exist_ok=True)
    RUNTIME.mkdir(parents=True, exist_ok=True)
    staging = BUILD / 'runtime'
    if staging.exists():
        shutil.rmtree(staging)
    destination = staging / 'mujoco'
    shutil.copytree(package, destination, dirs_exist_ok=True)

    # Rename only the copied library's SONAME. All physics machine code stays in this library.
    soname = b'libmujoco.so.3.6.0\x00'
    renamed = b'libmujoph.so.3.6.0\x00'
    data = (package / soname[:-1].decode()).read_bytes()
    if data.count(soname) != 1 or len(soname) != len(renamed):
        raise ValueError('Unexpected ELF SONAME layout')
    core = destination / renamed[:-1].decode()
    core.write_bytes(data.replace(soname, renamed))

    translations = json.loads((HERE / 'translations.json').read_text())
    for english, chinese in [('Geom', '几何'), ('Site', '标记点'), ('Joint', '关节'),
                             ('Tendon', '肌腱'), ('Actuator', '驱动'), ('Flex', '柔性体'),
                             ('Skin', '蒙皮'), ('Act Group', '驱动组')]:
        for index in range(6):
            translations[f'{english} {index}'] = f'{chinese} {index}'
    lines = ['static const Translation translations[] = {']
    for english, chinese in translations.items():
        lines.append(f'  {{{json.dumps(english)}, {json.dumps(chinese, ensure_ascii=False)}}},')
    lines.append('};')
    (BUILD / 'translations.h').write_text('\n'.join(lines) + '\n')

    ui = original('src/ui/ui_main.c')
    ui = replace_once(ui, '#include <math.h>', '#include "render_zh.h"\n#include <math.h>')
    ui = replace_once(ui,
        '  // add character widths\n  while (text[i]',
        '  int translated = limit < 0 ? mjzh_textwidth(text, con->charHeight) : -1;\n'
        '  if (translated >= 0) return translated;\n\n'
        '  // add character widths\n  while (text[i]')
    ui = replace_once(ui,
        '  // determine string length that fits in maxwidth',
        '  glColor3fv(rgb);\n  glRasterPos3i(x, y, 0);\n'
        '  if (mjzh_draw(txt, maxwidth, con->charHeight)) return;\n\n'
        '  // determine string length that fits in maxwidth')
    (BUILD / 'ui_main.c').write_text(ui)

    render = original('src/render/classic/render_gl2.c')
    render = replace_once(render, '#include <math.h>', '#include "render_zh.h"\n#include <math.h>')
    render = replace_once(render, '  // shadow text\n',
        '  glColor4f(r, g, b, 1);\n  glRasterPos3f(x, y, z);\n'
        '  if (mjzh_draw(txt, 1000000, font == mjFONT_BIG ? con->charHeightBig : con->charHeight)) return;\n\n'
        '  // shadow text\n')
    render = replace_once(render,
        'static int textwidth(const mjrContext* con, const char* text) {\n',
        'static int textwidth(const mjrContext* con, const char* text) {\n'
        '  int translated = mjzh_textwidth(text, con->charHeight);\n'
        '  if (translated >= 0) return translated;\n')
    (BUILD / 'render_gl2.c').write_text(render)
    visual = original('src/engine/engine_vis_init.c')
    bound = visual[visual.index('float mjv_rbound('):]
    notice = visual.split('#include', 1)[0]
    (BUILD / 'render_bound.c').write_text(notice + '#include <math.h>\n#include <mujoco/mujoco.h>\n' + bound)

    freetype = shlex.split(subprocess.check_output(['pkg-config', '--cflags', 'freetype2'], text=True))
    common = ['-O2', '-fPIC', '-fvisibility=hidden', '-D_GNU_SOURCE', '-DMUJOCO_DLL_EXPORTS',
              '-I' + str(package / 'include'), '-I' + str(SOURCE / 'src'),
              '-I' + str(HERE), '-I' + str(BUILD), *freetype]
    sources = [BUILD / 'ui_main.c', BUILD / 'render_gl2.c', BUILD / 'render_bound.c', HERE / 'render_zh.c']
    sources += [SOURCE / f'src/render/classic/{name}.c' for name in
                ('render_context', 'render_gl3', 'render_util')]
    sources += [SOURCE / 'src/render/classic/glad/glad.c', SOURCE / 'src/render/classic/glad/loader.cc']
    objects = []
    for index, source in enumerate(sources):
        target = BUILD / f'render_{index}.o'
        compiler, standard = ('g++', '-std=c++17') if source.suffix == '.cc' else ('gcc', '-std=c11')
        run(compiler, standard, *common, '-c', source, '-o', target)
        objects.append(target)
    result = BUILD / soname[:-1].decode()
    run('g++', '-shared', '-Wl,--no-undefined', '-Wl,-soname,' + soname[:-1].decode(),
        '-Wl,-rpath,$ORIGIN', *objects, '-L' + str(destination),
        '-l:' + core.name, '-lfreetype', '-ldl', '-lm', '-o', result)
    shutil.copy2(result, destination / result.name)
    shutil.copy2(SOURCE / 'LICENSE', staging / 'MUJOCO-LICENSE')
    metadata = {'version': installed.version, 'python': sys.executable,
                'base_library_sha256': hashlib.sha256(data).hexdigest(),
                'renderer_sha256': hashlib.sha256(result.read_bytes()).hexdigest(),
                'translations': len(translations)}
    (staging / 'build.json').write_text(json.dumps(metadata, indent=2) + '\n')
    bundle_hash = hashlib.sha256(data + result.read_bytes()).hexdigest()
    version = CACHE / ('runtime-' + bundle_hash[:12])
    if not version.exists():
        shutil.copytree(staging, version)
    link = CACHE / 'runtime-next'
    link.unlink(missing_ok=True)
    link.symlink_to(version.name)
    if RUNTIME.exists() and not RUNTIME.is_symlink():
        RUNTIME.rename(CACHE / f'runtime-initial-{time.time_ns()}')
    os.replace(link, RUNTIME)
    print(f'Chinese MuJoCo runtime ready at {RUNTIME} ({len(translations)} translations)')


if __name__ == '__main__':
    build()
