import math
import numpy as np

import os
import sys
directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(directory, '../ss_pybullet'))

from pybullet_tools.pr2_utils import get_top_grasps, get_side_grasps
from pybullet_tools.utils import link_from_name, get_body_name, GraspInfo, Pose, \
    INF, Point, approximate_as_prism, multiply, point_from_pose, unit_pose, invert
from pybullet_tools.kuka_primitives import BodyGrasp

GRASP_LENGTH = 0.
MAX_GRASP_WIDTH = np.inf

GRASP_INFO = {
    # GraspInfo = namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])
    'top': GraspInfo(lambda body: get_top_grasps(body, under=True, tool_pose=Pose(),
                                                 max_width=INF,  grasp_length=0),
                     Pose(0.1*Point(z=1))),
    'side': GraspInfo(lambda body: get_side_grasps(body, under=True, tool_pose=Pose(),
                                                 max_width=INF,  grasp_length=0),
                     Pose(0.1*Point(z=1))),
    'bottom': GraspInfo(lambda body: get_bottom_grasps(body, under=True, tool_pose=Pose(),
                                                 max_width=INF,  grasp_length=0),
                     Pose(0.1*Point(z=-1)))
}

TOOL_FRAMES = {
    'abb_irb6600_track': 'eef_tcp_frame',
}


def get_bottom_grasps(body, under=False, tool_pose=Pose(), body_pose=unit_pose(),
                      max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    """used for eth_rfl_pick test"""
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, 0, 0])
    translate_z = Pose(point=[0, 0, h / 2])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]

    return grasps


def get_grasp_gen(robot, grasp_name, tool_frame_name, tool_from_tcp=None):
    """Get a grasp generator.

    Parameters
    ----------
    robot : type
    grasp_name : str
        name of the grasp class
        only supports 'top', 'side', 'botttom' for now
    tool_frame_name : str
        e.g. 'ee_link'
    tool_from_tcp : Pose
        extra ee_mount_link to tcp transformation if any, defaults to None

    Returns
    -------
    function handle
        gen(pybullet body) : BodyGrasp

    """
    # GraspInfo : ['get_grasps', 'approach_pose'])
    grasp_info = GRASP_INFO[grasp_name]
    end_effector_link = link_from_name(robot, tool_frame_name)

    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        for tool_from_body in grasp_poses:
            # approach_pose = grasp_from_approach
            if tool_from_tcp:
                tcp_from_body = multiply(invert(tool_from_tcp), tool_from_body)
            else:
                tcp_from_body = tool_from_body
            body_grasp = BodyGrasp(body, tcp_from_body, grasp_info.approach_pose,
                                   robot, end_effector_link)
            yield (body_grasp,)
    return gen
