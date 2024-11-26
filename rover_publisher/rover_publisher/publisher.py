import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        # TODO: Create a publisher of type Twist
        self.publisher = self.create_publisher(Twist, 'topic', 10)

        self.get_logger().info('Publisher node has been started.')

        # TODO: Create a loop here to ask users a prompt and send messages accordingly
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.cmd_acquisition)


     # Function that prompts user for a direction input, and sends the command
    def cmd_acquisition(self):
        command = input("Enter command (w/a/s/d/t/y - max 2 characters): ")
        # TODO: Complete the function to transform the input into the right command.
        msg = Twist()
        if len(command) <= 2:
            if command.__contains__("w") or command.__contains__("s"):
                msg.linear.x = 1.0 if command.__contains__("w") else -1.0
            if command.__contains__("a") or command.__contains__("d"):
                msg.linear.z = 1.0 if command.__contains__("d") else -1.0
            if command.__contains__("t") or command.__contains__("y"):
                msg.angular.y = 1.0 if command.__contains__("t") else -1.0
        if msg.linear.x != 0.0 or msg.angular.y != 0.0 or msg.linear.z != 0.0:
            self.publisher.publish(msg)
            self.get_logger().info(f'Publised: linear \nx: {msg.linear.x}\ny: {msg.linear.y}\nz: {msg.linear.z}\nangular:\nx: {msg.angular.x}\ny: {msg.angular.y}\nz: {msg.angular.z}')
        pass

def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()