############################################################################################################################################
########################################################### DO NOT MODIFY THIS FILE ########################################################
############################################################################################################################################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class GetPointsPublisher(Node):
    def __init__(self):
        super().__init__('get_points_publisher')
        self.publisher = self.create_publisher(Point, 'goal_point', 10)
        self.subscription = self.create_subscription(Bool, 'goal_reached', self.goal_reached_callback, 10)

        self.vertices = [
            (5.0, 5.0),   # Bottom left
            (10.0, 5.0),  # Bottom right
            (10.0, 10.0), # Top right
            (5.0, 10.0)   # Top left
        ]
        self.current_vertex = 0
        self.next_vertex = 1
        self.step_size = 0.2
        self.current_point = Point(x=self.vertices[0][0], y=self.vertices[0][1], z=0.0)
        self.loop_complete = False

        # Publish the first point
        self.publish_point()

    def goal_reached_callback(self, msg):
        # Only proceed to publish the next point if the goal was reached
        if msg.data and not self.loop_complete:
            self.publish_point()

    def publish_point(self):
        # Current and next vertices
        current = self.vertices[self.current_vertex]
        next_ = self.vertices[self.next_vertex]

        # Direction and distance to the next vertex
        dx, dy = next_[0] - current[0], next_[1] - current[1]
        distance = (dx**2 + dy**2) ** 0.5

        # If close to the next vertex, snap to it and update vertices
        if abs(self.current_point.x - next_[0]) < self.step_size and abs(self.current_point.y - next_[1]) < self.step_size:
            self.current_point.x = next_[0]
            self.current_point.y = next_[1]
            self.publisher.publish(self.current_point)
            
            # Move to the next vertex
            self.current_vertex = self.next_vertex
            self.next_vertex = (self.next_vertex + 1) % len(self.vertices)
            
            # Check if completed one loop
            if self.current_vertex == 0:
                self.loop_complete = True

        else:
            # Normalize direction, step to the next point
            unit_dx, unit_dy = dx / distance, dy / distance
            self.current_point.x += unit_dx * self.step_size
            self.current_point.y += unit_dy * self.step_size
            self.publisher.publish(self.current_point)

def main(args=None):
    rclpy.init(args=args)
    publisher = GetPointsPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


############################################################################################################################################
########################################################### DO NOT MODIFY THIS FILE ########################################################
############################################################################################################################################