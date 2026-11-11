import numpy as np

from robot_transmission.coupled_limits import CoupledLimits


def test_motion_projection_keeps_a_valid_path_and_frame_speed():
    workspace = CoupledLimits({'pair': {
        'joints': ['a', 'b'], 'lower': [-.1, -.1], 'step': .1,
        'valid_rows': ['010', '111', '010'],
    }}, ['a', 'b'])
    limits = dict.fromkeys(['a', 'b'], (-.1, .1))
    previous = {'a': 0., 'b': 0.}
    requested = {'a': .09, 'b': .08}
    step = dict.fromkeys(['a', 'b'], .05)
    result = workspace.project_step(limits, previous, requested, step)
    workspace.validate(result)
    np.testing.assert_allclose([result['a'], result['b']], [.05, 0.])
    for name in previous:
        assert abs(result[name] - previous[name]) <= step[name]
    following = workspace.project_step(limits, result, {'a': 0., 'b': .08}, step)
    workspace.validate(following)
    np.testing.assert_allclose([following['a'], following['b']], [0., .05])
