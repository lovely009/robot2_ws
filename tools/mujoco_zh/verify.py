#!/usr/bin/env python3
"""Check translated native UI drawing and shortcuts in an isolated EGL context."""

import ctypes
import json
import os
from pathlib import Path
import re
import subprocess

os.environ['MUJOCO_GL'] = 'egl'

import mujoco
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
CACHE = Path.home() / '.cache/robot2-mujoco-zh'
RUNTIME = CACHE / 'runtime'
SOURCE = CACHE / 'source-3.6.0'
PACKAGE = Path(mujoco.__file__).parent
assert PACKAGE.parent.resolve() == RUNTIME.resolve(), 'Run verify.py with python_zh.sh'
os.chdir(CACHE)

dictionary = json.loads((HERE / 'translations.json').read_text())
labels = set()
for path in [SOURCE / 'simulate/simulate.cc', SOURCE / 'simulate/simulate.h']:
    labels.update(re.findall(r'\{mjITEM_[A-Z0-9_]+,\s*"([^"\n]+)"', path.read_text()))
missing = sorted(label for label in labels
                 if label not in dictionary and not re.fullmatch(r'.+ [0-5]', label))
assert not missing, missing

library = CACHE / 'build/verify_native.so'
subprocess.run([
    'gcc', '-std=c11', '-O2', '-shared', '-fPIC', str(HERE / 'verify_native.c'),
    '-I' + str(PACKAGE / 'include'), '-L' + str(PACKAGE),
    '-Wl,-rpath,' + str(PACKAGE), '-Wl,--no-undefined', '-l:libmujoco.so.3.6.0',
    '-l:libmujoph.so.3.6.0',
    '-lm', '-o', str(library),
], check=True)
native = ctypes.CDLL(str(library))
native.verify_native.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_ubyte),
                                ctypes.c_int, ctypes.c_int, ctypes.c_int]
native.verify_native.restype = ctypes.c_int
model = HERE.parents[1] / 'ros2_ws/src/robot_description/models/full_body_robot/mujoco/full_body_robot.xml'
results = []
for scale in (50, 100, 150, 200, 250, 300):
    width, height = max(1440, scale * 6 + 600), 1000
    context = mujoco.GLContext(width, height)
    context.make_current()
    pixels = np.zeros((height, width, 3), dtype=np.uint8)
    result = native.verify_native(str(model).encode(), pixels.ctypes.data_as(
        ctypes.POINTER(ctypes.c_ubyte)), width, height, scale)
    context.free()
    assert result == 0, (scale, result)
    assert pixels.max() > 128 and pixels.std() > 10
    image = Image.fromarray(np.flipud(pixels))
    image.save(CACHE / f'native-ui-{scale}.png')
    results.append({'font_scale': scale, 'native_ui': 'passed', 'shortcut': 'passed'})
print(json.dumps({'static_labels': len(labels), 'checks': results}, ensure_ascii=False))
