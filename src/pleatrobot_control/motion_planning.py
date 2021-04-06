#!/usr/bin/env python
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
import roslib
from rviz_tools import RvizMarkers
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL



# Initialize the ROS Node













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



class Cartesian_planning(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(Cartesian_planning, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    
    markers = RvizMarkers('/world', 'visualization_marker')

    point1 = geometry_msgs.msg.Point(-2,1,0)
    point2 = geometry_msgs.msg.Point(2,1,0) 
    width = 0.5
    markers.publishLine(point1, point2, 'green', width)# point1, point2, color, width, lifetime

    
    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "group4_complete"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print(" Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

   
    # Publish a line between two ROS Point Msgs


  def plan_cartesian_path(self, xInput=0, yInput =0, zInput=0, scale=1):
    

    move_group = self.move_group
    waypoints = []

    wpose = move_group.get_current_pose().pose


    wpose.position.z = scale * float(zInput) 
    wpose.position.y = scale * float(yInput)  
    wpose.position.x = scale * float(xInput) 
    waypoints.append(copy.deepcopy(wpose))


    # path = []
    # wpose2 = move_group.get_current_pose().pose
    # wpose2.position.z = scale * float(zInput) 
    # wpose2.position.y = scale * float(yInput)  
    # wpose2.position.x = scale * float(xInput) 
    # path.append(copy.deepcopy(wpose2.position))
    # width = 0.02
    # markers.publishPath(path, 'orange', width, 5.0) # path, color, width, lifetime


    #  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * float(yInput) # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
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
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):

    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


class Gripper_control(object):

  def __init__(self):
    super(Gripper_control, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
      
    # while not rospy.is_shutdown():

      # point1 = geometry_msgs.msg.Point(0.3, 0.3, 0.6)
      # point2 = geometry_msgs.msg.Point(-0.3, 0.4, 0.5)
      # width = 0.05
      # markers.publishLine(point1, point2, 'green', width, 5.0) # point1, point2, color, width, lifetime
      # rospy.Rate(1).sleep() 

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "group3_gripper"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state1(self):

    move_group = self.move_group

    ## Planning to a Joint Goal

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.05
    joint_goal[1] = -pi/4
    joint_goal[2] = pi/4

    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_joint_state2(self):

    move_group = self.move_group

    ## Planning to a Joint Goal

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = pi/4
    joint_goal[2] = -pi/4

    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

class Gantry_control(object):

  def __init__(self):
    super(Gantry_control, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "group1_gantry"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

 
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def plan_cartesian_path(self, xInput=0, yInput =0, scale=1):
    
    move_group = self.move_group
    waypoints = []

    wpose = move_group.get_current_pose().pose
    
    wpose.position.z -= scale * 0  # First move up (z)
    wpose.position.y = scale * float(yInput)  # and sideways (y)
    wpose.position.x = scale * float(xInput) 
    waypoints.append(copy.deepcopy(wpose))

     # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * float(yInput) # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

      ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
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
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):

    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

class Kuka_control(object):

  def __init__(self):
    super(Kuka_control, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "group2_kuka"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self, joint0=0, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0):

    move_group = self.move_group

    ## Planning to a Joint Goal

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = float(joint0)
    joint_goal[1] = float(joint1)
    joint_goal[2] = float(joint2)
    joint_goal[3] = float(joint3)
    joint_goal[4] = float(joint4)
    joint_goal[5] = float(joint5)
    joint_goal[6] = float(joint6)

    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


def main():



  try:

    while True:
      
      
      print("----------------------------------------------------------")
      print("Please select an option")
      print("----------------------------------------------------------")

      user_choice = input("\n Press 1 to move tool_tip in Cartesian path \n Press 2 to Control the Gripper \n Press 3 to Control Gantry Joint position \n Press 4 to Control Kuka Joint Position \n Press q to exit  ")
      

      if user_choice == "1":
        print("")
        print("----------------------------------------------------------")
        print("Cartesian Planning")
        print("----------------------------------------------------------")
        print("")
        input("Enter to begin the control setup...")
        cartesian = Cartesian_planning()

        xInput = input("Enter x ")
        yInput = input("Enter y ")
        zInput = input("Enter z ")
        input("Enter to plan a cartesian path")
        cartesian_plan, fraction = cartesian.plan_cartesian_path(xInput, yInput, zInput)

        input("Enter to confirm the path")
        cartesian.display_trajectory(cartesian_plan)

        input("Enter to exceute the path")
        cartesian.execute_plan(cartesian_plan)

        print("---------------------Trajctory has been planned---------------")


      if user_choice == "2":

        print("----------------------------------------------------------")
        print("Gripper Control")
        print("----------------------------------------------------------")
        print("")
        input("Enter to begin the control setup...")
        vimee_gripper = Gripper_control()

        input("Enter to open the gripper")
        vimee_gripper.go_to_joint_state1()

        input("Enter to close the gripper")
        vimee_gripper.go_to_joint_state2()

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        print("-----------------------Gripper motion exceuted-------------------")



      if user_choice == "3":

        print("")
        print("----------------------------------------------------------")
        print("Gantry Position Planning")
        print("----------------------------------------------------------")
        print("")
        input("Enter to begin the control setup...")
        cartesian = Gantry_control()

        xInput = input("Enter x in meters (limit +-0.4m) ")
        yInput = input("Enter y in meters (limit +-0.4m) ")

        input("Enter to plan a cartesian path")
        cartesian_plan, fraction = cartesian.plan_cartesian_path(xInput, yInput)

        input("Enter to confirm the path")
        cartesian.display_trajectory(cartesian_plan)

        input("Enter to exceute the path")
        cartesian.execute_plan(cartesian_plan)

        print("---------------------Trajctory has been planned---------------")



      if user_choice == "4":

        print("----------------------------------------------------------")
        print("Kuka Control")
        print("----------------------------------------------------------")
        print("")
        input("Enter to begin the control setup...")
        kuka = Kuka_control()

        joint0 = input("Enter joint0 in radians (limit +-2.95) ")
        joint1 = input("Enter joint1 in radians (limit +-2.09) ")
        joint2 = input("Enter joint2 in radians (limit +-2.96) ")
        joint3 = input("Enter joint3 in radians (limit +-2.09) ")
        joint4 = input("Enter joint4 in radians (limit +-2.96) ")
        joint5 = input("Enter joint5 in radians (limit +-2.09) ")
        joint6 = input("Enter joint6 in radians (limit +-3.05) ")

        input("Enter to move the arm joints")
        kuka.go_to_joint_state(joint0, joint1, joint2, joint3, joint4, joint5, joint6)
         # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        print("-----------------------Kuka motion exceuted-------------------")


      if user_choice == "q":
        break


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()