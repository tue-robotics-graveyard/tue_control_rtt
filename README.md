# tue_control_rtt
RTT Control component implementation based on the tue_control package

To test:

    # In one terminal:
    roscore

    # Other terminal 
    roscd tue_control_rtt
    rosrun rtt_ros deployer test/test_amigo.ops

    # Other terminal
    rostopic echo /joint_states

    # Other terminal
    rosrun tue_control_rtt send_reference.py shoulder_yaw_joint_left -1
