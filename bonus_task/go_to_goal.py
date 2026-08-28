import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_msgs.msg import Bool
from math import sqrt, atan2, pi

class GetPointsSubscriber(Node):
    def __init__(self):
        super().__init__('get_points_subscriber')
        
        ########################## DO NOT MODIFY THIS LINE ###########################
        self.subscription = self.create_subscription(Point, 'goal_point', self.point_callback, 10)
        ##############################################################################

        
        ########################### FILL IN THE CODE FOR THE MENTIONED COMMENT####################
        
        # Write a subscriber to get turtle pose, topic name '/turtle1/pose' and message type 'Pose'
        
        # Write a publisher for topic '/turtle1/cmd_vel' message type is 'Twist'
        
        ##########################################################################################

        
        ########################## DO NOT MODIFY THESE LINES ###########################
        self.goal_reached_publisher = self.create_publisher(Bool, 'goal_reached', 10)
        
        # Client for the SetPen service, to move the pen up and down
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetPen service...')

        self.move_to_goal = self.create_timer(0.1, self.move_to_goal)
        ################################################################################

    def pose_callback(self, msg):
        """Callback function to update the turtle's current position."""
        #################COMPLETE THIS FUNCTION ##################
        
        
    def point_callback(self, point):
        """Callback function to receive a new goal point."""
        self.goal = point
        self.goal_in_progress = True
        self.get_logger().info(f'Received new goal: ({point.x}, {point.y}, {point.z})')


    ##################################### WRITE THE LOGIC ###############################
    def move_to_goal(self):
        if self.goal is None or self.current_pose is None:
            return

        # Calculate distance and angle to the goal
        current_x, current_y = self.current_pose.x, self.current_pose.y
        goal_x, goal_y = self.goal.x, self.goal.y
        
        ##################### COMPLETE THIS ######################
        # Find distance 
        # Find angle 

        # Normalize angle to [-pi, pi]

        # Calculate velocity commands
        # Determines how much the robot should move forward 
        # and how much it should rotate to move towards the goal.
        # linear_gain 
        # angular_gain 
    
        
        twist = Twist()
        twist.linear.x = 
        twist.angular.z = 
        ##########################################################

        # Publishes velocity commands
        self.velocity_publisher.publish(twist)

        if # Write the condition to stop when near goal 
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            
            ########################## DO NOT MODIFY THESE LINES ###########################
            self.get_logger().info('Reached the goal!')
            self.goal_in_progress = False  # Reset the flag

            # Publish that the goal is reached
            self.goal_reached_publisher.publish(Bool(data=True))

            # Set pen down if not already down
            if not self.pen_down:
                self.set_pen(True)  # Call method to lower the pen
                self.pen_down = True  # Update flag to prevent repeated pen-down calls
            ################################################################################

    ########################## DO NOT MODIFY THESE LINES ###########################
    def set_pen(self, pen_down):
        """Method to set pen state: down if pen_down is True, otherwise up."""
        request = SetPen.Request()
        request.r = 255
        request.g = 0
        request.b = 0
        request.width = 2
        request.off = not pen_down 
        self.set_pen_client.call_async(request)
        self.get_logger().info('Pen set down.')
    #################################################################################
    
def main(args=None):
    rclpy.init(args=args)
    subscriber = GetPointsSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
