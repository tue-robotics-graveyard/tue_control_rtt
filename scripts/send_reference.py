#! /usr/bin/python

import sys
import rospy
from actionlib import SimpleActionClient, GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from diagnostic_msgs.msg import DiagnosticArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tue_manipulation_msgs.msg import GraspPrecomputeGoal, GraspPrecomputeAction
from tue_manipulation_msgs.msg import GripperCommandGoal, GripperCommandAction
from tue_msgs.msg import GripperCommand

def main():
    rospy.init_node("send_reference")

    ref_args = sys.argv[1:]

    if not ref_args:
        rospy.logwarn("Usage: send_reference JOINT_NAME_1 POSITION_1 [ JOINT_NAME_2 POSITION 2 ... ]")        
        return 1

    ac_joint_traj = SimpleActionClient("/amigo/body/joint_trajectory_action", FollowJointTrajectoryAction)

    while not ac_joint_traj.wait_for_server(timeout=rospy.Duration(1.0)) and not rospy.is_shutdown():
        rospy.logwarn("Waiting for server...")

    if rospy.is_shutdown():
        rospy.logwarn("No reference sent")        
        return 1

    rospy.loginfo("Connected to server")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # Even items in ref_args are joint_names
    joint_names = ref_args[0::2]

    # Odd items in ref_args are set points
    ref_positions = [float(i) for i in ref_args[1::2]]

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    points = [JointTrajectoryPoint(positions=ref_positions, time_from_start=rospy.Duration(0))]

    joint_trajectory = JointTrajectory(joint_names=joint_names, points=points)
    goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory, goal_time_tolerance=rospy.Duration(1000))

    ac_joint_traj.send_goal(goal)
    ac_joint_traj.wait_for_result(rospy.Duration(1000))

    res = ac_joint_traj.get_result()
    if res.error_string:
        rospy.logerr(res.error_string)

if __name__ == "__main__":
    sys.exit(main())