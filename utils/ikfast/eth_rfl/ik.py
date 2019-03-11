import random

import os
import sys
directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(directory, '../../../ss_pybullet'))

from pybullet_tools.ikfast.utils import get_ik_limits, compute_forward_kinematics, \
    compute_inverse_kinematics, select_solution
from pybullet_tools.utils import multiply, get_link_pose, \
    link_from_name, get_joint_positions, invert, violates_limits, get_distance, get_movable_joints
from ...eth_rfl_utils import get_torso_arm_joints, get_torso_joint, get_tool_frame, get_arm_joint_names

IK_FRAMES = {
    'left': 'l_robot_tool0',
    'right': 'r_robot_tool0',
}

IK_BASE_FRAMES = {
    'left': 'l_gantry_xy_carriage',
    'right': 'r_gantry_xy_carriage',
}

#####################################

def get_tool_pose(robot, arm):
    from .ikfast_eth_rfl import get_fk
    ik_joints = get_torso_arm_joints(robot, arm)
    conf = get_joint_positions(robot, ik_joints)
    # TODO: this should be linked to ikfast's get numOfJoint junction
    base_from_ik = compute_forward_kinematics(get_fk, conf)
    base_from_tool = multiply(base_from_ik, invert(get_tool_from_ik(robot, arm)))
    world_from_base = get_link_pose(robot, link_from_name(robot, IK_BASE_FRAMES[arm]))
    return multiply(world_from_base, base_from_tool)

#####################################

def is_ik_compiled():
    try:
        from .ikfast_eth_rfl import ikfast_eth_rfl
        return True
    except ImportError:
        return False


def get_ik_generator(robot, arm, tool_pose, **kwargs):
    from .ikfast_eth_rfl import get_ik
    world_from_base = get_link_pose(robot, link_from_name(robot, IK_BASE_FRAMES[arm]))
    base_from_tool = multiply(invert(world_from_base), tool_pose)
    base_from_ik = multiply(base_from_tool, get_tool_from_ik(robot, arm))
    sampled_limits = get_ik_limits(robot, get_torso_joint(robot, arm), **kwargs)
    while True:
        sampled_values = [random.uniform(*sampled_limits)]
        ik_joints = get_torso_arm_joints(robot, arm)
        confs = compute_inverse_kinematics(get_ik, base_from_ik, sampled_values)
        yield [q for q in confs if not violates_limits(robot, ik_joints, q)]


def get_tool_from_ik(robot, arm):
    world_from_tool = get_link_pose(robot, link_from_name(robot, get_tool_frame(arm)))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAMES[arm]))
    # tool from the bare flange (6th axis)
    return multiply(invert(world_from_tool), world_from_ik)


def sample_tool_ik(robot, arm, tool_pose, closest_only=False, get_all=False, **kwargs):
    generator = get_ik_generator(robot, arm, tool_pose, **kwargs)
    ik_joints = get_torso_arm_joints(robot, arm)
    solutions = next(generator)
    if closest_only and solutions:
        current_conf = get_joint_positions(robot, ik_joints)
        solutions = [min(solutions, key=lambda conf: get_distance(current_conf, conf))]
    solutions = list(filter(lambda conf: not violates_limits(robot, ik_joints, conf), solutions))
    return solutions if get_all else select_solution(robot, ik_joints, solutions, **kwargs)
