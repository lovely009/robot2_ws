"""Names from Robot_body1 meshes; retain export whitespace only in file paths."""
from pathlib import Path
import json

MODEL = Path(__file__).resolve().parents[1]


def names():
    metadata = json.loads((MODEL / 'config/display_meshes.json').read_text())
    links = {old: Path(entry['source']).stem.strip() for old, entry in metadata['links'].items()}
    joints = {old.removesuffix('_link'): new.removesuffix('_link')
              for old, new in links.items() if old != 'base_link'}
    if len(set(links.values())) != 38 or len(set(joints.values())) != 37:
        raise ValueError('Native joint/link names must be one-to-one')
    return joints, links


def rename_tree(value, mapping):
    if isinstance(value, dict):
        return {mapping.get(k,k): rename_tree(v,mapping) for k,v in value.items()}
    if isinstance(value,list):
        return [rename_tree(v,mapping) for v in value]
    return mapping.get(value,value) if isinstance(value,str) else value
