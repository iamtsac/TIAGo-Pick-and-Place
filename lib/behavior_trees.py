import dartpy
import numpy as np
import py_trees
from pi_controller import PIController
from math_utils import damped_pseudoinverse, AdT, enforce_joint_limits


class PickUpBox(py_trees.behaviour.Behaviour):
    def __init__(self, robot, offset, dt, box, basket, link_name, name="PickUpBox"):
        super(PickUpBox, self).__init__(name)
        self.robot = robot # robot
        self.box = box
        self.basket = basket
        self.link_name = link_name # Link name to controll
        self.dt = dt # dt
        self.offset = offset
        self.update_once = False

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        self.logger.debug("%s.setup()->does nothing" % (self.__class__.__name__))

    def set_tf_desired(self, target_robot):
        # Face target
        if self.name == 'rotate':
            xy = np.subtract(self.robot.body_pose("base_link").translation()[:-1], target_robot.base_pose().translation()[:-1])
            theta = np.arctan2(xy[1],xy[0]) + np.pi
            self.tf_desired = dartpy.math.Isometry3()
            self.tf_desired.set_translation(self.robot.base_pose().translation())
            self.tf_desired.set_rotation(dartpy.math.eulerXYZToMatrix([0, 0, theta]))

        # Set desired position 
        else:
            self.tf_desired = dartpy.math.Isometry3()
            goal_robot_trans = target_robot.base_pose().translation()
            goal_robot_trans[goal_robot_trans == 0] = 1
            self.tf_desired.set_translation(
                np.add(
                    target_robot.base_pose().translation(),
                    -1*(np.sign(goal_robot_trans))*self.offset[0]
                )
            )
            self.tf_desired.set_rotation(dartpy.math.eulerXYZToMatrix(self.offset[1]))

        return self.tf_desired

    def close_gripper(self):
        if(self.robot.body_pose("gripper_tool_link").translation()[2] - self.box.base_pose().translation()[2])**2 < 0.027:
            return True
        return False

    def initialise(self):
        self.logger.debug("%s.initialise()->init controller" % (self.__class__.__name__))
        self.Kp = 1.8 
        self.Ki = 0.01
        self.controller = PIController(self.set_tf_desired(self.box), self.dt, self.Kp, self.Ki)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        if not self.update_once:
            # update in case box falls.
            self.controller.set_target(self.set_tf_desired(self.box))
        if self.name == 'lift_box':
            self.update_once = True
        sorted_dofs_by_index = sorted(self.robot.dof_map().items(), key=lambda x: x[1])
        
        del sorted_dofs_by_index[24] # del finger joint
        if self.name == 'rotate': # only manipulate base
            self.robot.set_commands([0],['rootJoint_pos_x'])
            self.robot.set_commands([0],['rootJoint_pos_y'])
            del sorted_dofs_by_index[3:]
        else:
            del sorted_dofs_by_index[5] # del pos_z
        del sorted_dofs_by_index[:2]
        dofs_of_interest = [dof[0] for dof in sorted_dofs_by_index]

        # control the robot
        tf = self.robot.body_pose(self.link_name)
        vel = self.controller.update(tf)
        jac = self.robot.jacobian(self.link_name, dofs_of_interest) # this is in world frame
        jac_pinv = damped_pseudoinverse(jac) # get pseudo-inverse
        cmd = jac_pinv @ vel
        self.robot.set_commands(cmd, dofs_of_interest) # apply cmds to specific dofs

        # if error too small, report success
        err = np.linalg.norm(self.controller.error(tf))

        if self.close_gripper(): # if box between finger, close
            self.robot.set_commands([-.2], ['gripper_finger_joint']) 


        if err < 1 * 1e-1 and self.name == 'rotate': # for rotation
            new_status = py_trees.common.Status.SUCCESS
        elif err < 1.4 * 1e-1 and self.name != 'rotate': # for box pickup
            new_status = py_trees.common.Status.SUCCESS


        if new_status == py_trees.common.Status.SUCCESS:
            self.controller.reset_sum_error() #reset controller
            self.update_once = False # renable continuous desired pos update
            self.feedback_message = "Reached target"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            self.feedback_message = "Error: {0}".format(err)
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class LeaveToBasket(py_trees.behaviour.Behaviour):
    def __init__(self, robot, offset, dt, box, basket, link_name, name="LeaveToBasket"):
        super(LeaveToBasket, self).__init__(name)
        self.robot = robot # robot
        self.box = box
        self.basket = basket
        self.link_name = link_name # Link name to controll
        self.dt = dt # dt
        self.offset = offset
        self.update_once = False

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        self.logger.debug("%s.setup()->does nothing" % (self.__class__.__name__))

    def set_tf_desired(self, target_robot):
        xy = np.subtract(self.robot.body_pose("base_link").translation()[:-1], target_robot.base_pose().translation()[:-1])
        theta = np.arctan2(xy[1],xy[0]) + np.pi

        if self.name == 'rotate':
            self.tf_desired = dartpy.math.Isometry3()
            self.tf_desired.set_translation(self.robot.base_pose().translation())
            self.tf_desired.set_rotation(dartpy.math.eulerXYZToMatrix([0, 0, theta]))

        else:
            self.tf_desired = dartpy.math.Isometry3()
            goal_robot_trans = target_robot.base_pose().translation()
            goal_robot_trans[goal_robot_trans == 0] = 1
            self.tf_desired.set_translation(
                np.add(
                    target_robot.base_pose().translation(),
                    -1*(np.sign(goal_robot_trans))*self.offset[0]
                )
            )
            self.tf_desired.set_rotation(
                dartpy.math.eulerXYZToMatrix([0, 0, theta]))

        return self.tf_desired

    def close_gripper(self):
        if(self.robot.body_pose("gripper_tool_link").translation()[2] - self.box.base_pose().translation()[2])**2 < 0.03:
            return True
        return False

    def initialise(self):
        self.logger.debug("%s.initialise()->init controller" % (self.__class__.__name__))
        self.Kp = [0.5, 1, 1.2 ,1.5, 1.5, 1.5] 
        self.Ki = 0.01
        self.controller = PIController(self.set_tf_desired(self.basket), self.dt, self.Kp, self.Ki)

    def update(self):
        new_status = py_trees.common.Status.RUNNING

        sorted_dofs_by_index = sorted(self.robot.dof_map().items(), key=lambda x: x[1])
        del sorted_dofs_by_index[24]
        if self.name == 'rotate' :
            self.robot.set_commands([0],['rootJoint_pos_x']) # remove previous commands
            self.robot.set_commands([0],['rootJoint_pos_y'])
            del sorted_dofs_by_index[3:]
        # To not overrun basket
        elif np.linalg.norm((self.robot.body_pose("base_link").translation()[:-1] - self.basket.base_pose().translation()[:-1]), ord=2) < 0.5: 
            self.robot.set_commands([0],['rootJoint_pos_x']) # remove previous commands
            self.robot.set_commands([0],['rootJoint_pos_y'])
            del sorted_dofs_by_index[3:6]
        else:
            del sorted_dofs_by_index[5] # remove pos_z
        del sorted_dofs_by_index[:2]
        dofs_of_interest = [dof[0] for dof in sorted_dofs_by_index]

        # control the robot
        tf = self.robot.body_pose(self.link_name)
        vel = self.controller.update(tf)
        jac = self.robot.jacobian(self.link_name, dofs_of_interest) # this is in world frame
        jac_pinv = damped_pseudoinverse(jac) # get pseudo-inverse
        cmd = jac_pinv @ vel
        self.robot.set_commands(cmd, dofs_of_interest) # apply cmds to specific dofs

        # if error too small, report success
        err = np.linalg.norm(self.controller.error(tf))

        if err < 1.2 * 1e-1: # release box
            self.robot.set_commands([.2], ['gripper_finger_joint'])
        if err < 1.5 * 1e-1 and self.name == 'rotate':
            new_status = py_trees.common.Status.SUCCESS

        if self.box.base_pose().translation()[2] < .04: # if fallen open grip
            self.robot.set_commands([.2], ['gripper_finger_joint'])
            new_status = py_trees.common.Status.SUCCESS # multiple successes to restart tree
        elif self.box.base_pose().translation()[2] > .04 and err > 1.2 * 1e-1:
            self.robot.set_commands([-.5], ['gripper_finger_joint']) # if still in hand hold tight

        if new_status == py_trees.common.Status.SUCCESS:
            self.controller.reset_sum_error() # reset controller
            self.feedback_message = "Reached target"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            self.feedback_message = "Error: {0}".format(err)
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))