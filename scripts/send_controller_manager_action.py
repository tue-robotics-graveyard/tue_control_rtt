#! /usr/bin/python

import sys
import rospy
from tue_control_rtt_msgs.msg import ControllerManagerAction, ControllerAction

rospy.init_node("send_controller_action")

args = sys.argv[1:]
actions = dict(zip(args[0::2],args[1::2]))

if not actions:
    rospy.logwarn("Usage: send_controller_action CONTROLLER_NAME_1 CONTROLLER_ACTION_1 [CONTROLLER_NAME_2 CONTROLLER_ACTION_2]") 
    sys.exit(1)

pub = rospy.Publisher("action", ControllerManagerAction, queue_size=10)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - -

rospy.loginfo("Sending the the following actions to the controller manager (topic=action):")

msg = ControllerManagerAction()
for name, action in actions.iteritems():
    rospy.loginfo("%s --> %s" % (name, action))
    msg.actions.append(ControllerAction(name=name, action=action, parameters=[]))

rospy.sleep(0.5)
pub.publish(msg)
