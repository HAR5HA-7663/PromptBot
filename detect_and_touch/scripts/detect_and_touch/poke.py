#!/usr/bin/env python3

import rospy
import argparse
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from stretch_srvs.srv import GetCluster, GetClusterRequest, GetClusterResponse, MoveArm, MoveArmResponse, MoveArmRequest
import numpy as np
import pdb


def get_cluster_points(main_query):
    """Get all cluster points for a specific query."""
    rospy.wait_for_service('/get_top1_cluster')
    
    try:
        # Create a service proxy
        get_cluster = rospy.ServiceProxy('/get_top1_cluster', GetCluster)
        
        pdb.set_trace()
        
        # Prepare the request
        response = get_cluster(main_query=main_query, criterion='', num_points=10)
        if response.success and response.pts:
            rospy.loginfo(f"Retrieved {len(response.pts)} points for query '{main_query}'.")
            return [np.array([pt.x, pt.y, pt.z]) for pt in response.pts]
        else:
            rospy.logwarn("No clusters found or no points returned.")
            return []
    except rospy.ServiceException as e:
        rospy.logerr(f"Cluster service call failed: {e}")
        return []

def move_arm_to_point(point):
    """Move the robot's arm to a specific point."""
    rospy.wait_for_service('/funmap/move_arm')
    try:
        move_arm = rospy.ServiceProxy('/funmap/move_arm', MoveArm)
        response = move_arm(Point(x=point[0], y=point[1], z=point[2]))
        if response.success:
            rospy.loginfo(f"Arm moved successfully to {point}.")
            return True
        else:
            rospy.logwarn(f"Failed to move arm to {point}.")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Move arm service call failed: {e}")
        return False

def adjust_point(point, offsets):
    """Apply small offsets to a point."""
    for offset in offsets:
        adjusted_point = point + np.array(offset)
        rospy.loginfo(f"Trying adjusted point: {adjusted_point}")
        if move_arm_to_point(adjusted_point):
            rospy.loginfo(f"Successfully moved to adjusted point: {adjusted_point}")
            return True
    return False

def select_best_point(points, criterion="mean"):
    """Select the best point based on the given criterion."""
    if not points:
        return None

    if criterion == "mean":
        return np.mean(points, axis=0)
    elif criterion == "max":
        return points[np.argmax([np.linalg.norm(pt) for pt in points])]
    elif criterion == "min":
        return points[np.argmin([np.linalg.norm(pt) for pt in points])]
    else:
        rospy.logwarn(f"Unknown criterion '{criterion}', defaulting to mean.")
        return np.mean(points, axis=0)

def fallback_random_point(points):
    """Select a random point from the cluster if all other methods fail."""
    if points:
        random_point = random.choice(points)
        rospy.loginfo(f"Fallback to random point: {random_point}")
        return random_point
    return None

def main():
    rospy.init_node('cluster_centroid_publisher')

    # Get parameters
    main_query = rospy.get_param('~main_query', 'object_name')
    parser = argparse.ArgumentParser(description="Call /get_top1_cluster service with custom queries")
    parser.add_argument('main_query', type=str, help="Main query for the service")

    args = parser.parse_args()
    adjustment_offsets = [(0.05, 0, 0), (-0.05, 0, 0), (0, 0.05, 0), (0, -0.05, 0), (0, 0, 0.05)]

    # Retrieve cluster points
    points = get_cluster_points(args.main_query)
    if not points:
        rospy.logerr("No points retrieved. Aborting.")
        return

    # Select the best point based on the criterion
    best_point = select_best_point(points)
    if best_point is None:
        rospy.logerr("No valid point selected. Aborting.")
        return

    # Attempt to move the arm to the selected point
    success = move_arm_to_point(best_point)
    if not success:
        rospy.logwarn("Initial attempt failed. Trying other points.")
        pdb.set_trace()
        for point in points:
            success = move_arm_to_point(point)
            if success:
                rospy.loginfo(f"Successfully moved to an alternative point: {point}")
                break
        else:
            rospy.logwarn("All direct attempts failed. Applying adjustments.")
            for point in points:
                success = adjust_point(point, adjustment_offsets)
                if success:
                    break
            if not success:
                rospy.logwarn("All adjustments failed. Falling back to a random point.")
                random_point = fallback_random_point(points)
                if random_point and move_arm_to_point(random_point):
                    rospy.loginfo(f"Successfully moved to fallback random point: {random_point}")
                else:
                    rospy.logerr("All attempts to move arm failed. Aborting.")

if __name__ == '__main__':
    main()
