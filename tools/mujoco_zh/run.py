"""Open the MuJoCo viewer at the model's standing keyframe when available."""

import argparse
from pathlib import Path

import mujoco
from mujoco import viewer


def load_model(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    standing = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'Standing')
    if standing >= 0:
        mujoco.mj_resetDataKeyframe(model, data, standing)
        mujoco.mj_forward(model, data)
    return model, data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mjcf', type=Path, required=True, help='Path to the MJCF model.')
    path = parser.parse_args().mjcf.expanduser()
    viewer.launch(loader=lambda: load_model(path))


if __name__ == '__main__':
    main()
