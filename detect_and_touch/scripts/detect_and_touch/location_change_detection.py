import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math
import pdb  # Import pdb for debugging

class OdomMovementChecker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odom_movement_checker', anonymous=True)
        
        # Initialize variables
        self.initial_position = None
        self.previous_position = None
        self.current_position = None

        # Movement threshold (e.g., 0.1 meters for position change)
        self.position_threshold = 0.1  # Adjust this based on required sensitivity

        # Variance threshold for stability check
        self.variance_threshold = 0.0001

        rospy.loginfo("Odom Movement Checker initialized. Ready to check position...")

    def subscribe_once(self):
        """ Subscribe to odometry once, get value, and unsubscribe """
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Subscribed to /odom topic for a single message...")
        rospy.sleep(1.0)  # Allow some time for the callback to process
        self.odom_sub.unregister()  # Unsubscribe after processing
        rospy.loginfo("Unsubscribed from /odom topic.")

    def odom_callback(self, msg):
        """ Callback to process odometry messages """
        position = msg.pose.pose.position
        self.current_position = Point(position.x, position.y, position.z)

        # Debug: Log the updated position
        rospy.loginfo(f"Current position received: x={self.current_position.x}, y={self.current_position.y}")

        if self.initial_position is None:
            self.initial_position = self.current_position
            rospy.loginfo(f"Initial position set to: x={self.initial_position.x}, y={self.initial_position.y}")

    def has_robot_moved(self):
        """ Check if the robot has moved beyond the threshold distance """
        if self.initial_position is None or self.current_position is None:
            rospy.logwarn("Positions are not set. Cannot determine movement.")
            return False

        # Calculate Euclidean distance from the initial position
        distance = self.calculate_distance(self.initial_position, self.current_position)
        rospy.loginfo(f"Distance from initial position: {distance:.6f}")
        return distance > self.position_threshold

    @staticmethod
    def calculate_distance(pos1, pos2):
        """ Calculate Euclidean distance between two positions """
        return math.sqrt(
            (pos1.x - pos2.x) ** 2 +
            (pos1.y - pos2.y) ** 2
        )

    def check_movement(self):
        """ Perform movement check with manual subscription control """
        self.subscribe_once()  # Get the initial position
        rospy.loginfo("Checking if robot has moved...")
        pdb.set_trace
        self.subscribe_once()  # Get updated position

        if self.has_robot_moved():
            rospy.loginfo("Robot has moved from its initial position!")
        else:
            rospy.loginfo("Robot has not moved significantly.")

if __name__ == '__main__':
    try:
        checker = OdomMovementChecker()
        while not rospy.is_shutdown():
            checker.check_movement()
            rospy.sleep(2)  # Wait before checking movement again
    except rospy.ROSInterruptException:
        rospy.loginfo("Odom Movement Checker node terminated.")
