import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path + '/lib/')
from pydoc import describe
import numpy as np
import RobotDART as rd
import dartpy
import py_trees

from utils import create_grid, box_into_basket
from math_utils import damped_pseudoinverse, AdT, enforce_joint_limits
from pi_controller import PIController
from behavior_trees import PickUpBox, LeaveToBasket

dt = 0.001
simulation_time = 40.0
total_steps = int(simulation_time / dt)

# Create robot
packages = [("tiago_description", "tiago/tiago_description")]
robot = rd.Tiago(int(1. / dt), "tiago/tiago_steel.urdf", packages)

arm_dofs = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
            "arm_6_joint", "arm_7_joint", "gripper_finger_joint", "gripper_right_finger_joint"]
robot.set_positions(np.array(
    [np.pi/2., np.pi/4., 0., np.pi/2., 0., 0., np.pi/2., 0.03, 0.03]), arm_dofs)

# Control base - we make the base fully controllable
robot.set_actuator_type("servo", "rootJoint", False, True, False)

# Create position grid for the box/basket
basket_positions, box_positions = create_grid()

# Create box
box_size = [0.04, 0.04, 0.04]
# Random cube position
box_pt = np.random.choice(len(box_positions))
box_pose = [0., 0., 0., box_positions[box_pt][0],
            box_positions[box_pt][1], box_size[2] / 2.0]
box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [
                          0.9, 0.1, 0.1, 1.0], "box_" + str(0))

# Create basket
basket_packages = [("basket", dir_path + "/lib/urdfs/models/basket")]
basket = rd.Robot(dir_path + "/lib/urdfs/models/basket/basket.urdf", basket_packages, "basket")
# Random basket position
basket_pt = np.random.choice(len(basket_positions))
basket_z_angle = 0.
basket_pose = [0., 0., basket_z_angle, basket_positions[basket_pt]
               [0], basket_positions[basket_pt][1], 0.0008]
basket.set_positions(basket_pose)
basket.fix_to_world()

# Create Graphics
gconfig = rd.gui.Graphics.default_configuration()
gconfig.width = 1280
gconfig.height = 960
graphics = rd.gui.Graphics(gconfig)


# Create simulator object
simu = rd.RobotDARTSimu(dt)
simu.set_collision_detector("bullet")
simu.set_control_freq(100)
simu.set_graphics(graphics)
graphics.look_at((0., 4.5, 2.5), (0., 0., 0.25))
simu.add_checkerboard_floor()
simu.add_robot(robot)
simu.add_robot(box)
simu.add_robot(basket)

finish_counter = 0

py_trees.logging.level = py_trees.logging.Level.DEBUG # Behavior Tree
root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne()) # Create tree root
sequence = py_trees.composites.Sequence(name="Sequence") # Create sequence node (for sequential targets)

rotate_to_box = PickUpBox(robot, [[0, 0, 0], [0, 0, 0]], dt, box, basket, 'base_link', name="rotate")
sequence.add_child(rotate_to_box)

grab_box = PickUpBox(robot, [[0, 0, 0], [0, 0, np.pi/2]], dt, box, basket, 'gripper_link', name="grab_box")
sequence.add_child(grab_box)

lift_box = PickUpBox(robot, [[0, 0, -.5], [0, 0, np.pi/2]], dt, box, basket, 'gripper_link', name="lift_box")
sequence.add_child(lift_box)

rotate_to_basket = LeaveToBasket(robot, [[0, 0, 0], [0, 0, 0]], dt, box, basket, 'base_link', name="rotate")
sequence.add_child(rotate_to_basket)

leave_box = LeaveToBasket(robot, [[0.0, 0, -.5], [0, 0, 0]], dt, box, basket, 'gripper_link', name="go_to_basket")
sequence.add_child(leave_box)

root.add_child(sequence)

while finish_counter<50:
    if (simu.schedule(simu.control_freq())):
       # Do something
        box_translation = box.base_pose().translation()
        basket_translation = basket.base_pose().translation()
        if box_into_basket(box_translation, basket_translation, basket_z_angle) or np.linalg.norm(basket_translation - box_translation,ord=2) < 0.1:
            finish_counter += 1
            simu.remove_robot(box)
            simu.remove_robot(basket)
            box_pt = np.random.choice(len(box_positions))
            box_pose = [0., 0., 0., box_positions[box_pt][0],
                        box_positions[box_pt][1], box_size[2] / 2.0]
            if np.abs(box_pose[3] - robot.base_pose().translation()[0]) < .21 or np.abs(box_pose[4] - robot.base_pose().translation()[1]) < .21:
                continue

            box = rd.Robot.create_box(box_size, box_pose, "free", 0.1, [
                          0.9, 0.1, 0.1, 1.0], "box_" + str(0))

            basket = rd.Robot(dir_path + "/lib/urdfs/models/basket/basket.urdf", basket_packages, "basket")
            basket_pt = np.random.choice(len(basket_positions))
            basket_z_angle = 0.
            basket_pose = [0., 0., basket_z_angle, basket_positions[basket_pt]
                        [0], basket_positions[basket_pt][1], 0.0008]
            basket.set_positions(basket_pose)
            basket.fix_to_world()
            simu.add_robot(box)
            simu.add_robot(basket)
            for seq in range(5) :
                root.children[0].children[seq].box = box
                root.children[0].children[seq].basket = basket

    root.tick_once()
    if (simu.step_world()):
        break