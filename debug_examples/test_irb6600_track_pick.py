#!/usr/bin/env python

from __future__ import print_function

import os
import sys
directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(directory, '..'))
sys.path.append(os.path.join(directory, '../ss_pybullet'))

from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, \
    get_free_motion_gen, get_holding_motion_gen, BodyPath, Attach
from pybullet_tools.utils import WorldSaver, connect, dump_world, set_pose, \
    Pose, Point, stable_z, BLOCK_URDF, load_model, wait_for_interrupt, disconnect, user_input, update_state, \
    disable_real_time, load_pybullet
from pybullet_tools.utils import get_movable_joints, \
    set_joint_positions, enable_gravity, end_effector_from_body, approach_from_grasp, \
    inverse_kinematics, pairwise_collision, get_sample_fn, plan_direct_joint_motion, HideOutput, LockRenderer
from utils.ikfast.abb_irb6600_track.ik import sample_tool_ik, get_track_arm_joints, TOOL_FRAME
from utils.pick_primitives import get_grasp_gen

USE_IKFAST = True
DEBUG_FAILURE = True
ENABLE_SELF_COLLISION = False
IRB6600_TRACK_URDF = "../models/abb_irb6600_track/urdf/irb6600_track.urdf"


def get_ik_fn(robot, fixed=[], teleport=False, num_attempts=10, self_collisions=True):
    movable_joints = get_track_arm_joints(robot) if USE_IKFAST else get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    def fn(body, pose, grasp):
        obstacles = [body] + fixed
        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        for _ in range(num_attempts):
            if USE_IKFAST:
                q_approach = sample_tool_ik(robot, approach_pose)
                if q_approach is not None:
                    set_joint_positions(robot, movable_joints, q_approach)
            else:
                set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
                q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue
            conf = BodyConf(robot, joints=movable_joints)

            if USE_IKFAST:
                q_grasp = sample_tool_ik(robot, gripper_pose, closest_only=True)
                if q_grasp is not None:
                    set_joint_positions(robot, movable_joints, q_grasp)
            else:
                q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles, \
                                                self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: user_input('Approach motion failed')
                    continue
            command = Command([BodyPath(robot, path, joints=movable_joints),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], joints=movable_joints, attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn


def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top', TOOL_FRAME)
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)
    movable_joints = get_track_arm_joints(robot) if USE_IKFAST else get_movable_joints(robot)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, joints=movable_joints)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        return Command(path1.body_paths + path2.body_paths + path3.body_paths)
    return None


def main(display='execute'):  # control | execute | step
    connect(use_gui=True)
    disable_real_time()

    with HideOutput():
        root_directory = os.path.dirname(os.path.abspath(__file__))
        robot = load_pybullet(os.path.join(root_directory, IRB6600_TRACK_URDF), fixed_base=True)
    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    floor_x = 2
    set_pose(floor, Pose(Point(x=floor_x, z=0.5)))
    set_pose(block, Pose(Point(x=floor_x, y=0, z=stable_z(block, floor))))
    # set_default_camera()
    dump_world()

    saved_world = WorldSaver()
    with LockRenderer():
        command = plan(robot, block, fixed=[floor], teleport=False)
    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print('Quit?')
        wait_for_interrupt()
        disconnect()
        return

    saved_world.restore()
    update_state()
    user_input('{}?'.format(display))
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.002)
    elif display == 'step':
        command.step()
    else:
        raise ValueError(display)

    print('Quit?')
    wait_for_interrupt()
    disconnect()


if __name__ == '__main__':
    main()
