import random

import os
import sys
directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(directory, '../../../ss_pybullet'))

from pybullet_tools.ikfast.utils import get_ik_limits, compute_forward_kinematics, \
    compute_inverse_kinematics, select_solution
from pybullet_tools.utils import multiply, get_link_pose, \
    link_from_name, get_joint_positions, invert, violates_limits, joint_from_name, joints_from_names, get_movable_joints, \
    get_distance

BASE_FRAME = 'linear_axis_base_link'
IK_FRAME = 'robot_tool0'
TOOL_FRAME = 'eef_tcp_frame'  #robot_tool0 | eef_tcp_frame

TRACK_JOINT = ['linear_axis_actuation_joint']
ARM_JOINTS = ['robot_joint_{}'.format(1+i) for i in range(6)]
TRACK_ARM_JOINT = TRACK_JOINT + ARM_JOINTS


def get_track_arm_joints(robot):
    return joints_from_names(robot, TRACK_ARM_JOINT)

#####################################

def get_tool_pose(robot):
    from .ikfast_abb_irb6600_track import get_fk
    ik_joints = get_track_arm_joints(robot)
    conf = get_joint_positions(robot, ik_joints)
    # TODO: this should be linked to ikfast's get numOfJoint function
    assert len(conf) == 7
    base_from_tool = compute_forward_kinematics(get_fk, conf)
    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    return multiply(world_from_base, base_from_tool)

#####################################

def is_ik_compiled():
    try:
        from .ikfast_abb_irb6600_track import ikfast_abb_irb6600_track
        return True
    except ImportError:
        return False


def get_ik_generator(robot, tool_pose, track_limits=False):
    from .ikfast_abb_irb6600_track import get_ik
    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    base_from_tool = multiply(invert(world_from_base), tool_pose)
    base_from_ik = multiply(base_from_tool, get_tool_from_ik(robot))
    sampled_limits = get_ik_limits(robot, joint_from_name(robot, *TRACK_JOINT), track_limits)
    while True:
        sampled_values = [random.uniform(*sampled_limits)]
        ik_joints = get_track_arm_joints(robot)
        confs = compute_inverse_kinematics(get_ik, base_from_ik, sampled_values)
        yield [q for q in confs if not violates_limits(robot, ik_joints, q)]


def get_tool_from_ik(robot):
    world_from_tool = get_link_pose(robot, link_from_name(robot, TOOL_FRAME))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME))
    # tool from the bare flange (6th axis)
    return multiply(invert(world_from_tool), world_from_ik)


def sample_tool_ik(robot, tool_pose, closest_only=False, get_all=False, **kwargs):
    generator = get_ik_generator(robot, tool_pose, **kwargs)
    ik_joints = get_movable_joints(robot)
    solutions = next(generator)
    if closest_only and solutions:
        current_conf = get_joint_positions(robot, ik_joints)
        solutions = [min(solutions, key=lambda conf: get_distance(current_conf, conf))]
    solutions = list(filter(lambda conf: not violates_limits(robot, ik_joints, conf), solutions))
    return solutions if get_all else select_solution(robot, ik_joints, solutions, **kwargs)
