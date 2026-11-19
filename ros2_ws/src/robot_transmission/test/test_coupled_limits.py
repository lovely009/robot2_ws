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
    # 帧间两轴会同时插值，须先收回 a，下一帧才能沿 b 移动。
    np.testing.assert_allclose([following['a'], following['b']], [0., 0.])
    for a in np.linspace(following['a'], result['a'], 11):
        for b in np.linspace(result['b'], following['b'], 11):
            workspace.validate({'a': a, 'b': b})
    next_frame = workspace.project_step(limits, following, {'a': 0., 'b': .08}, step)
    np.testing.assert_allclose([next_frame['a'], next_frame['b']], [0., .05])
