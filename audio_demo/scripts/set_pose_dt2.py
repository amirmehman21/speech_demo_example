#!/usr/bin/env python
from __future__ import division


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rospy
from std_msgs.msg import String,Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from std_msgs.msg import Int16
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import random
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand
import franka_gripper.msg
import actionlib

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveIt_Commander(object):
  """MoveIt_Commander"""
  def __init__(self):
    super(MoveIt_Commander, self).__init__()


    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_tool_pose=moveit_commander.MoveGroupCommander(group_name)
    hand_grp=moveit_commander.MoveGroupCommander("hand")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    request_subscriber = rospy.Subscriber('/request_detection', Int16, self.callback_request)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "ee link is:"
    print "============ End effector: %s" % eef_link


    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()


    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.grasp_client  = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    self.generic_grasp_client =  rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=10)
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.hand_grp=hand_grp
    self.tool_grp=group_tool_pose
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    self.width = 0.000
    self.angle = 0
    self.lift = 0

  def callback_cmd(self,data):

    if True:

        cartesian_plan, fraction = self.plan_linear_z(-0.161)
        self.execute_plan(cartesian_plan)
        self.close_hand(100,self.width)

        if self.lift:
            cartesian_plan, fraction = self.plan_linear_z(0.1)
            self.execute_plan(cartesian_plan)
            cartesian_plan, fraction = self.plan_linear_z(-0.1)
            self.execute_plan(cartesian_plan)
            self.open_hand()
            cartesian_plan, fraction = self.plan_linear_z(0.1)
            self.execute_plan(cartesian_plan)
        if not self.lift:
            self.open_hand()
        self.go_to_joint_state((0.00018762090386758654, -0.7771156373465307, 0.0005407026658047056, -2.365469873129632, -0.00020200796294576732, 1.5704722326730955, 0.7845368039521746))
    else:
        self.last_msg_idx = data.data[0]
        self.detections = np.asarray([9999,9999,9999,9999,9999])



  def callback_request(self, msg):

      if msg.data==1:
          self.width = 0.0039
          self.angle = 0
          self.lift = True
	  cartesian_plan, fraction = self.plan_linear_z(0.1)
	  self.execute_plan(cartesian_plan)
	  cartesian_plan, fraction = self.plan_linear_z(-0.1)
	  self.execute_plan(cartesian_plan)
	  self.close_hand(100,self.width)
	  self.open_hand()


      elif msg.data==2:
          self.width = 0.0039
          self.angle = 0
          self.lift = True
	  cartesian_plan, fraction = self.plan_linear_y(0.1)
	  self.execute_plan(cartesian_plan)
	  cartesian_plan, fraction = self.plan_linear_y(-0.1)
	  self.execute_plan(cartesian_plan)
	  self.close_hand(100,self.width)
	  self.open_hand()

  def gripper_move(self,cmd):

    group = self.hand_grp
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = cmd[0]
    joint_goal[1] = cmd[1]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.hand_grp.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state(self,cmd):

    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = cmd[0]
    joint_goal[1] = cmd[1]
    joint_goal[2] = cmd[2]
    joint_goal[3] = cmd[3]
    joint_goal[4] = cmd[4]
    joint_goal[5] = cmd[5]
    joint_goal[6] = cmd[6]
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, target):
    # continue here target added to args
    group = self.group
    current=group.get_current_pose().pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = target.pose.orientation.x
    pose_goal.orientation.y = target.pose.orientation.y
    pose_goal.orientation.z = target.pose.orientation.z
    pose_goal.orientation.w = target.pose.orientation.w
    pose_goal.position.x = target.pose.position.x
    pose_goal.position.y = target.pose.position.y
    pose_goal.position.z = target.pose.position.z
    print pose_goal
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def get_current_pose(self):

    group = self.group
    wpose = group.get_current_pose().pose

    return wpose

  def plan_linear_x(self, dist):


    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def plan_linear_y(self, dist):

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def plan_linear_z(self, dist):

    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += dist
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,
                                       0.01,
                                       0.0)
    return plan, fraction

  def execute_plan(self, plan):

    group = self.group
    group.execute(plan, wait=True)

  def fix_angle(self,rotcmd):

      ps = listener_tf.lookupTransform('/camera_link', '/panda_link8', rospy.Time(0))

      my_point=PoseStamped()
      my_point.header.frame_id = "camera_link"
      my_point.header.stamp = rospy.Time(0)
      my_point.pose.position.x = ps[0][0]
      my_point.pose.position.y = ps[0][1]
      my_point.pose.position.z = ps[0][2]

      theta = rotcmd / 180 * pi
      quat_rotcmd = tf.transformations.quaternion_from_euler(theta, 0, 0)
      quat = tf.transformations.quaternion_multiply(quat_rotcmd, ps[1])

      my_point.pose.orientation.x = quat[0]
      my_point.pose.orientation.y = quat[1]
      my_point.pose.orientation.z = quat[2]
      my_point.pose.orientation.w = quat[3]

      ps=listener_tf.transformPose("panda_link0",my_point)
      self.go_to_pose_goal(ps)

  def open_hand(self, wait=True):

        joint_goal = self.hand_grp.get_current_joint_values()
        joint_goal[0] = 0.0399

        self.hand_grp.set_goal_joint_tolerance(0.001)
        self.hand_grp.go(joint_goal, wait=wait)

        self.hand_grp.stop()

        current_joints = self.hand_grp.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


  def close_hand(self, force, width, speed=20.0, epsilon_inner=0.002, epsilon_outer=0.002):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        print("""
            Grasping with
            {}
            {}
        """.format(force, width))
        self.grasp_client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(
                    epsilon_inner,
                    epsilon_outer
                ),
                speed,
                force
            )
        )
        self.grasp_client.wait_for_result()
        return self.grasp_client.get_result()



def main():

  try:

    Commander = MoveIt_Commander()
    Commander.go_to_joint_state((0.00018762090386758654, -0.7771156373465307, 0.0005407026658047056, -2.365469873129632, -0.00020200796294576732, 1.5704722326730955, 0.7845368039521746))
    rospy.spin()
    print "============ Python Commander demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':

    rospy.init_node('move_group_python_commander',
                    anonymous=True)
    listener_tf = tf.TransformListener()
    main()


















