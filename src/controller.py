#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
import random
import time
from tf.transformations import quaternion_from_euler


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

#The main controller class for the robot
class FanucInterface(object):
  """Fanuc Interface Class"""
  def __init__(self):
    super(FanucInterface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints). 
    #group_name = "manipulator"
    self.group_name = "fanuc_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.move_group.get_planning_frame()
    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.move_group.get_end_effector_link()
    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    # Misc variables
    self.box_name = ''
    self.box_hand_name = ''
    self.base_name = ''
    #x, y, z, (position), then x, y, z, w (orientation)
    #self.zero_pt = [1.0484, -0.082704, 1.41, 0.017675, -0.70689, 0.017675, 0.70689]
    self.zero_pt = [-0.67132, 0.67648, 2.1999, 0.64962, -0.27916, 0.64972, 0.27915]
    self.y_pos_lim = []
    self.x_pos_lim = []
    self.z_pos_lim = []
    self.x_o_lim = []
    self.y_o_lim = []
    self.z_o_lim = []
    self.w_o_lim = []
    self.printRobotInfo()

 
  #Main function for controlling robot in joint coordinates
  def go_to_joint_state(self,joint_goal):

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def randomJointMove(self):
    joints = self.move_group.get_random_joint_values()
    self.move_group.go(joints, wait=True)
    self.move_group.stop()
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joints, current_joints, 0.01)
 
  #Returns robot to zero point in joint coordinates
  def zeroOut(self):
    joint_goal = [0.0, 0.0, 0.0, 0.0 ,0.0, 0.0]
    self.go_to_joint_state(joint_goal)


  #Prints the point (wpose) passed in, as well as a user messsage
  def printPt(self, wpose, message):
      print(message)
      print("POSITION")
      print(wpose.position.x)
      print(wpose.position.y)
      print(wpose.position.z)
      print("ORIENTATION")
      print(wpose.orientation.x)
      print(wpose.orientation.y)
      print(wpose.orientation.z)
      return

  #Main function for cartesian coordinates. Takes 3 points (position only, use previous orientation), 6 points (position and orientation as rpy)
  #Or 7 points (position and quaternion data)
  def go_to_pose_goal(self, points):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    numPoints = len(points)
    current_pose = self.move_group.get_current_pose().pose
    #If we were not given a correct amount of coordinates
    if (numPoints != 3 and numPoints != 6 and numPoints != 7):
       print("Sufficient point info not provided. Please supply 3, 6, or 7 points")
       return
   
    #We always get at least the x, y, z coordinates
    pose_goal.position.x = points[0]
    pose_goal.position.y = points[1]
    pose_goal.position.z = points[2]

    #If we got only 3 points, set the orientation to the same as the previous orientation
    if (numPoints == 3):
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w

    #Otherwise, If we were  given xyz orientation data as roll, pitch, yaw: 
    if (numPoints == 6):
      #Convert rpy to quaternion
      q= quaternion_from_euler(points[3], points[4], points[5])
      #Set it to the pose
      pose_goal.orientation.x = q[0]
      pose_goal.orientation.y = q[1]
      pose_goal.orientation.z = q[2]
      pose_goal.orientation.w = q[3]
    #Otherwise we were given the straight quaternion data
    elif numPoints == 7:
      pose_goal.orientation.x = points[3]
      pose_goal.orientation.y = points[4]
      pose_goal.orientation.z = points[5]
      pose_goal.orientation.w = points[6]

    #Make this the pose target
    self.move_group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  #Sends the robot to a position xyz coordinate with any orientation
  def positionAnyOrientation(self, pos):
    self.move_group.set_position_target(pos)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    #return all_close(pos, current_pose, 0.01)

  #Sends the robot to an orientation xyzw coordinate with any position
  def orientationAnyPosition(self, ori):
    self.move_group.set_orientation_target(ori) 
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets() 
    current_pose = self.move_group.get_current_pose().pose
    #return all_close(ori, current_pose, 0.01)

  def rpyOrientationAnyPosition(self, ori):
    self.move_group.set_rpy_target(ori) 
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets() 
    current_pose = self.move_group.get_current_pose().pose
    #return all_close(ori, current_pose, 0.01)

  def goToRandomPose(self):
    pose = self.move_group.get_random_pose()
    self.move_group.set_pose_target(pose)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets() 
    current_pose = self.move_group.get_current_pose().pose


  #Set the goal tolerance of the robot's orientation values
  def setOrientationTolerance(self, tolerance):
      print("Old Orientation Tolerance")
      print(self.move_group.get_goal_orientation_tolerance())
      print("New Orientation Tolerance")
      self.move_group.set_goal_orientation_tolerance(tolerance)
      print(self.move_group.get_goal_orientation_tolerance())

 #Set the goal tolerance of the robot's orientation values
  def setPositionTolerance(self, tolerance):
      print("Old Tolerance")
      print(self.move_group.get_goal_position_tolerance())
      print("New Tolerance")
      self.move_group.set_goal_position_tolerance(tolerance)
      print(self.move_group.get_goal_position_tolerance())

  #Function for visualizing a trajectory
  def display_trajectory(self, plan):
    #robot = self.robot
    #display_trajectory_publisher = self.display_trajectory_publisher

    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)

  #Actually executing the cartesian path we planned
  def execute_plan(self, plan):
    #move_group = self.move_group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    self.move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  #This makes sure everything makes it to the scene (or out of the scene)
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=60):
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  #Adds the table to the scene
  def add_table(self, timeout=4):
    #This is the collision buffer to be a little more sure that the robot won't annihilate the table
    buffer = 0.1 #20 cm -- all measurements are in meters
    table_x_size = 1.53
    table_y_size = 0.615
    table_z_size = 0.68 
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.00 #0.66 
    box_pose.pose.position.y = 0.915  #0.61
    box_pose.pose.position.z = (table_z_size+buffer)/2.0 #0.00
    box_pose.pose.orientation.z = 0.00 #-0.4
    table_name = "table"
    self.scene.add_box(table_name, box_pose, size=(table_x_size+buffer, table_y_size+buffer, table_z_size+buffer)) #1.53, 0.615, 0.68 with 2cm margin of error. 
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  #Base of the robot (pedestal it's on)
  def add_base(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0 #0.04 
    box_pose.pose.position.y = 0.0  #0.6
    box_pose.pose.position.z = 0.45 #0.00
    box_pose.pose.orientation.z = 0.0
    base_name = "base"
    self.scene.add_box(base_name, box_pose, size=(.61, 0.61, 0.9))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  #Floor of the workspace
  def add_floor(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0 #0.04 
    box_pose.pose.position.y = 0.0  #0.6
    box_pose.pose.position.z = 0.00 #0.00
    box_pose.pose.orientation.z = -0.4
    floor_name = "floor"
    self.scene.add_box(floor_name, box_pose, size=(5, 5, 0.01))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  #Add hand mesh
  def add_hand_mesh(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "link_6"
    box_pose.pose.orientation.w = 0.0
    box_pose.pose.orientation.z = 1.0
    box_pose.pose.position.x = -0.015
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    hand_box_name = "hand_box"
    self.scene.add_mesh(hand_box_name, box_pose, '/home/zach1804/catkin_ws/src/fanuc_demo/src/mesh/New_Hand_Mesh_V3.stl')
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_hand_mesh(self, timeout=4): #changed timeout from 8 to 4
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'eef'
    touch_links = self.robot.get_link_names(group=grasping_group)
    touch_links.append('flange')
    touch_links.append('link_6')
    touch_links.append('link_5')
   
    hand_box_name = "hand_box"

    #self.scene.attach_box(self.eef_link, self.box_hand_name, touch_links=touch_links)
    self.scene.attach_mesh('link_6', hand_box_name, touch_links=touch_links)
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  #box for robot hand
  def add_hand_box(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "link_6"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -0.14
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    hand_box_name = "hand_box"
    self.scene.add_box(hand_box_name, box_pose, size=(0.28, 0.2, 0.2))
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
   #Attach the hand box to the physical robot (on link_6) 
  def attach_hand_box(self, timeout=4): #changed timeout from 8 to 4
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'eef'
    touch_links = self.robot.get_link_names(group=grasping_group)
    touch_links.append('flange')
    touch_links.append('link_6')
    touch_links.append('link_5')
   
    hand_box_name = "hand_box"

    #self.scene.attach_box(self.eef_link, self.box_hand_name, touch_links=touch_links)
    self.scene.attach_box('link_6', hand_box_name, touch_links=touch_links)
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  #Sets up the robot scene 
  def setupScene(self, hand=True, timeout=4): #Change timeout from 8 to 4
       self.add_table()
       if (hand==True):
          # self.add_hand_box()
          # self.attach_hand_box()
          self.add_hand_mesh()
          self.attach_hand_mesh()
       self.add_base()
       self.add_floor()

  #Take down the robot scene (the objects the robot's environment and attached to the robot that aren't the robot itself)
  def destructScene(self, hand=True, timeout=4):
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    self.scene.remove_world_object("table")
    self.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=timeout)

    self.scene.remove_world_object("base")
    self.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=timeout)

    self.scene.remove_world_object("floor")
    self.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=timeout)
    if (hand == True):
      self.scene.remove_attached_object("link_6", name="hand_box")
      self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
      self.scene.remove_world_object("hand_box")
      self.wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=timeout)
    # We wait for the planning scene to update.
    return 

  #Get the joint values of the robot
  def my_get_current_joint_values(self):
    current_joints = self.move_group.get_current_joint_values()
    return current_joints 

  #Print the robot's info
  def printRobotInfo(self):
      print("============ Planning frame: %s" % self.planning_frame)
      # We can also print the name of the end-effector link for this group:
      print("============ End effector link: %s" % self.eef_link)
      # We can get a list of all the groups in the robot:
      print("============ Available Planning Groups:", self.robot.get_group_names())
      # Sometimes for debugging it is useful to print the entire state of the
      # robot:
      print("============ Printing robot state")
      print(self.robot.get_current_state())
      print("")
      return
