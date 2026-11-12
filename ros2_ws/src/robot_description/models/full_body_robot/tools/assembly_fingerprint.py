"""Fingerprint assembled geometry independently of preview control bounds."""
import hashlib
import json
import xml.etree.ElementTree as ET


def geometry_fingerprint(directory, root):
    # Match the default file parser, which discards XML comments.
    root = ET.fromstring(ET.tostring(root))
    files = sorted({asset.get('file') for asset in root.findall('asset/mesh')})
    hashes = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
              for name in files}
    for tag in ('custom', 'actuator', 'keyframe'):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    for joint in root.findall('.//joint'):
        joint.attrib.pop('limited', None)
        joint.attrib.pop('range', None)
    root.find('default').remove(root.find('default/position'))
    hashes['kinematics'] = hashlib.sha256(ET.tostring(root)).hexdigest()
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
