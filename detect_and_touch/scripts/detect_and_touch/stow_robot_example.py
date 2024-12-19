import rospy
from stretch_srvs.srv import MoveJoints, MoveJointsRequest, MoveJointsResponse

# This code is example code for controlling the arm through funmap.
# To prepare the robot, start up both the drivers and funmap as usual.
# Then calibrate the robot. Execute this code to stow the robot arm by
# providing joint positions for a set of specified joints in the arm

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('cluster_centroid_publisher')

    # Wait for the service to be available
    rospy.wait_for_service('/funmap/move_joints')
    
    try:
        # Create a service proxy
        move_arm = rospy.ServiceProxy('/funmap/move_joints', MoveJoints)
        
        # Valid joint names include:
        #   - joint_lift
        #   - joint_head_pan
        #   - joint_wrist_pitch
        #   - joint_wrist_roll
        #   - joint_head_tilt
        #   - joint_wrist_yaw
        #   - joint_gripper_finger_left
        #   - joint_arm
        #   - joint_arm_l3
        #   - joint_arm_l2
        #   - joint_arm_l1
        #   - joint_arm_l0
        #   - wrist_extension
        #   - gripper_aperture
        #   - joint_gripper_finger_right

        pose=MoveJointsRequest()        
        pose.joint_names=['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        pose.positions=[1, 1, 2]

        # Prepare the request
        response = move_arm(pose)
                
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

