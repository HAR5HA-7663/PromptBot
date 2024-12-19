#!/usr/bin/env python3

import rospy
import argparse
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from stretch_srvs.srv import GetCluster, GetClusterRequest, GetClusterResponse, MoveArm, MoveArmResponse, MoveArmRequest, TriggerHS
import numpy as np
import pdb


def get_cluster_points(main_query):
    """Get all cluster points for a specific query."""
    rospy.wait_for_service('/get_top1_cluster')
    try:
        get_cluster = rospy.ServiceProxy('/get_top1_cluster', GetCluster)
        response = get_cluster(main_query=main_query, criterion='')
        pdb.set_trace()
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
            rospy.logwarn(f"Failed to move arm to {point}. Obstruction likely.")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Move arm service call failed: {e}")
        return False


def perform_headscan():
    """Trigger head scan to detect updated points."""
    rospy.loginfo("Performing head scan to detect new points.")
    rospy.wait_for_service('/funmap/trigger_head_scan')
    try:
        trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', TriggerHS)
        response = trigger_head_scan()
        if response.success:
            rospy.loginfo("Head scan completed successfully.")
            return True
        else:
            rospy.logwarn("Head scan failed.")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Head scan service call failed: {e}")
        return False


def drive_to_scan():
    """Drive to scan location before performing head scan."""
    rospy.wait_for_service('/funmap/trigger_drive_to_scan')
    try:
        drive_to_scan_service = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', TriggerHS)
        response = drive_to_scan_service()
        if response.success:
            rospy.loginfo("Drive to scan location successful.")
            return True
        else:
            rospy.logwarn("Failed to drive to scan location.")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Drive to scan service call failed: {e}")
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


def fallback_random_points(points, retries=3):
    """Select random points and attempt movement multiple times."""
    for attempt in range(retries):
        random_point = random.choice(points)
        rospy.loginfo(f"Fallback attempt {attempt + 1}: Trying random point {random_point}.")
        if move_arm_to_point(random_point):
            rospy.loginfo(f"Successfully moved to fallback random point: {random_point}")
            return True
    rospy.logerr("All fallback random point attempts failed.")
    return False


def handle_obstruction(points, main_query):
    """Handle obstruction by driving to scan location, performing a head scan, and retrying approaches."""
    rospy.logwarn("Handling obstruction: Attempting to drive to scan location before performing head scan.")
    drive_success = drive_to_scan()
    if drive_success:
        rospy.loginfo("Drive to scan location was successful. Proceeding with head scan.")
    else:
        rospy.logwarn("Drive to scan location was unsuccessful. Proceeding with head scan regardless.")
        # TODO: Check if the location matches the start location to avoid unnecessary head scan.
    
    if perform_headscan():
        rospy.loginfo("Head scan completed. Retrying with updated points.")
        updated_points = get_cluster_points(main_query)
        if not updated_points:
            rospy.logerr("No new points found after head scan.")
            return False
        for criterion in ["mean", "max", "min"]:
            best_point = select_best_point(updated_points, criterion)
            if best_point is not None:
                rospy.loginfo(f"Retrying with {criterion} point: {best_point}")
                if move_arm_to_point(best_point):
                    rospy.loginfo(f"Successfully moved to {criterion} point after head scan.")
                    return True
        rospy.logwarn("All retry approaches after head scan failed. Trying fallback random points.")
        return fallback_random_points(updated_points)
    else:
        rospy.logerr("Head scan failed. Unable to resolve obstruction.")
        return False


def main():
    rospy.init_node('cluster_centroid_publisher')
    parser = argparse.ArgumentParser(description="Call /get_top1_cluster service with custom queries")
    parser.add_argument('main_query', type=str, help="Main query for the service")
    args = parser.parse_args()

    # Retrieve initial cluster points once
    points = get_cluster_points(args.main_query)
    if not points:
        rospy.logerr("No points retrieved. Exiting.")
        return

    # Try each criterion and handle obstruction if necessary
    for criterion in ["mean", "max", "min"]:
        best_point = select_best_point(points, criterion)
        if best_point is not None:
            rospy.loginfo(f"Trying {criterion} point: {best_point}")
            if move_arm_to_point(best_point):
                rospy.loginfo(f"Successfully moved to {criterion} point.")
                return
            else:
                rospy.logwarn("Obstruction detected. Performing drive to scan and retries.")
                if handle_obstruction(points, args.main_query):
                    return

    # Fallback to random points if all else fails
    rospy.logwarn("All criteria attempts failed. Falling back to random points.")
    if fallback_random_points(points):
        return

    rospy.logerr("All movement attempts failed. Exiting.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cluster centroid publisher node terminated.")
