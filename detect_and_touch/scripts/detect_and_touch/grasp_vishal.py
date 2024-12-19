#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from stretch_srvs.srv import GetCluster, GetClusterRequest, MoveArm, MoveArmRequest, MoveJoints, MoveJointsRequest
import numpy as np
import sys

class CaptureAndGrasp:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('capture_and_grasp_node', anonymous=True)

        # Initialize CVBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscriber for the RGB camera
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Service proxies
        rospy.wait_for_service('/get_top1_cluster')
        self.get_cluster_service = rospy.ServiceProxy('/get_top1_cluster', GetCluster)

        rospy.wait_for_service('/funmap/move_arm')
        self.move_arm_service = rospy.ServiceProxy('/funmap/move_arm', MoveArm)

        rospy.wait_for_service('/funmap/move_joints')
        self.move_joints_service = rospy.ServiceProxy('/funmap/move_joints', MoveJoints)

        # To store the latest captured image
        self.latest_image = None

    def image_callback(self, msg):
        """
        Callback function to store the latest captured image from the camera.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Image captured successfully.")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {str(e)}")

    def get_cluster_and_grasp(self, query, num_points=50):
        """
        Query the object cluster and grasp it.
        """
        try:
            # Call the cluster service
            rospy.loginfo("Calling /get_top1_cluster service...")
            response = self.get_cluster_service(GetClusterRequest(main_query=query, criterion='mean', num_points=num_points))

            if len(response.pts) > 0:
                rospy.loginfo("Cluster received successfully.")
                points = np.array([[pt.x, pt.y, pt.z] for pt in response.pts])
                centroid = points.mean(axis=0)

                rospy.loginfo(f"Centroid of the cluster: {centroid}")

                # Open the gripper before moving to the centroid
                self.open_gripper()

                # Move the arm to the centroid
                if self.move_to_centroid(centroid):
                    # Close the gripper to grasp after reaching the centroid
                    self.close_gripper()
                else:
                    rospy.logwarn("Failed to move to the centroid. Gripper action aborted.")
            else:
                rospy.logwarn("No clusters found for the given query. Exiting...")
                sys.exit(0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def move_to_centroid(self, centroid):
        """
        Move the robot arm to the specified centroid location.
        """
        try:
            rospy.loginfo("Calling /funmap/move_arm service...")
            move_request = MoveArmRequest()
            move_request.pt = Point(x=centroid[0], y=centroid[1], z=centroid[2])

            response = self.move_arm_service(move_request)

            if response.success:
                rospy.loginfo("Arm successfully moved to the centroid.")
                return True
            else:
                rospy.logwarn("Failed to move arm: " + response.message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
            return False

    def open_gripper(self):
        """
        Open the gripper before moving to the object.
        """
        try:
            rospy.loginfo("Calling /funmap/move_joints service to open gripper...")
            gripper_request = MoveJointsRequest()
            gripper_request.joint_names = ['gripper_aperture']
            gripper_request.positions = [1]  # Open position

            response = self.move_joints_service(gripper_request)

            if response.success:
                rospy.loginfo("Gripper successfully opened.")
            else:
                rospy.logwarn("Failed to open gripper: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

    def close_gripper(self):
        """
        Close the gripper to grasp the object.
        """
        try:
            rospy.loginfo("Calling /funmap/move_joints service to close gripper...")
            gripper_request = MoveJointsRequest()
            gripper_request.joint_names = ['gripper_aperture']
            gripper_request.positions = [-1]  # Close position

            response = self.move_joints_service(gripper_request)

            if response.success:
                rospy.loginfo("Gripper successfully closed.")
            else:
                rospy.logwarn("Failed to close gripper: " + response.message)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")

if __name__ == '__main__':
    # Initialize the CaptureAndGrasp class
    capture_and_grasp = CaptureAndGrasp()

    # Allow some time to capture an image
    rospy.sleep(2)

    # Check for command-line arguments
    if len(sys.argv) < 2:
        rospy.logerr("No query provided. Usage: ./capture_and_grasp.py <query>")
        sys.exit(1)

    # Get the query from the command-line arguments
    object_query = sys.argv[1]

    # Perform the grasping task
    capture_and_grasp.get_cluster_and_grasp(object_query)

    rospy.spin()
