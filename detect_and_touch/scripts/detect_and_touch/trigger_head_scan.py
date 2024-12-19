#!/usr/bin/env python3

import rospy
from stretch_srvs.srv import TriggerHS

def perform_headscan():
    """
    Trigger a head scan service to detect new points.
    Logs success or failure based on the service response.
    """
    rospy.loginfo("Initiating head scan...")
    
    # Wait until the service is available
    rospy.wait_for_service('/funmap/trigger_head_scan')

    try:
        # Create a service proxy
        trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', TriggerHS)

        # Call the service
        response = trigger_head_scan()
        
        # Process the response
        if response.success:
            rospy.loginfo("Head scan completed successfully.")
            rospy.loginfo(f"Service response: {response.message}")
            return True
        else:
            rospy.logwarn("Head scan failed. No points detected or unknown failure.")
            rospy.logwarn(f"Service response: {response.message}")
            return False

    except rospy.ServiceException as e:
        rospy.logerr(f"Head scan service call failed: {e}")
        return False

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('headscan_trigger_node')

    rospy.loginfo("Head scan trigger node started.")
    
    # Perform the head scan
    success = perform_headscan()

    # Final log based on success
    if success:
        rospy.loginfo("Head scan operation completed successfully.")
    else:
        rospy.logerr("Head scan operation failed. Please check the system.")
