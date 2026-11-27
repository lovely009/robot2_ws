"""Simulation name migration keeps old saved commands without alias ambiguity."""
import pytest

mujoco=pytest.importorskip('mujoco')
from robot_transmission.mujoco_positions import JointPositionTargets


def make_controller(aliases=None):
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body><joint name="canonical"/>
    <geom type="sphere" size=".01"/></body></worldbody><actuator><position joint="canonical"/>
    </actuator></mujoco>''')
    config={'joints':['canonical'],'joint_limits':{'canonical':[-1.,1.]},'flanges':{},'joint_aliases':aliases or {'old':'canonical'}}
    return model,JointPositionTargets(model,config)


def test_legacy_joint_command_uses_current_joint():
    model,controller=make_controller();data=mujoco.MjData(model)
    controller.apply(data,['old'],[.25])
    assert data.ctrl[0]==pytest.approx(.25)
    assert controller.targets=={'canonical':.25}


def test_alias_and_current_name_cannot_command_one_joint_twice():
    model,controller=make_controller();data=mujoco.MjData(model)
    with pytest.raises(ValueError,match='不可重复'):
        controller.apply(data,['old','canonical'],[.1,.2])
    assert data.ctrl[0]==0


def test_alias_target_must_exist():
    with pytest.raises(ValueError,match='别名'):
        make_controller({'old':'missing'})
