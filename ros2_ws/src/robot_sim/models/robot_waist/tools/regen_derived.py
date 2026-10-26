#!/usr/bin/env python3
"""从预览主 URDF 生成 standalone 与 ROS 无关发布件。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    model = Path(__file__).resolve().parent.parent
    name = model.name
    main_urdf = model / 'urdf' / f'{name}.urdf'
    dist = model.parent.parent / 'dist' / f'{name}_urdf'
    root = ET.parse(main_urdf).getroot()
    prefix = f'package://robot_sim/models/{name}/'
    meshes = {}
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if not filename.startswith(prefix):
            raise ValueError(f'网格不在当前模型目录内：{filename}')
        relative = filename[len(prefix):]
        meshes[relative] = model / relative
        mesh.set('filename', relative)
    ET.indent(root, space='  ')
    outputs = {
        model / 'urdf' / f'{name}.standalone.urdf': main_urdf.read_bytes(),
        dist / f'{name}.urdf': (
            '<?xml version="1.0" encoding="utf-8"?>\n'
            '<!-- 仅作几何预览；由 tools/regen_derived.py 生成，勿手改。 -->\n'
            + ET.tostring(root, encoding='unicode') + '\n'
        ).encode('utf-8'),
    }
    for path, content in outputs.items():
        if args.check:
            if not path.is_file() or path.read_bytes() != content:
                raise ValueError(f'派生件缺失或过期：{path}')
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
    for relative, source in meshes.items():
        target = dist / relative
        if args.check:
            if not target.is_file() or source.read_bytes() != target.read_bytes():
                raise ValueError(f'发布件网格缺失或过期：{target}')
        elif not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            os.link(source, target)
    print(f'{name}: 派生件与 {len(meshes)} 个网格校验通过' if args.check
          else f'{name}: 已生成 standalone 与 dist')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
