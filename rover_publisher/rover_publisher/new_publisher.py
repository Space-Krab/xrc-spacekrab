import rclpy
from rclpy.node import Node
from custom_msg.msg import Controller
import pygame


class TrajectoryPublisher2(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        pygame.joystick.init()
        
        self.joystick = pygame.joystick.Joystick(0)
        
        self.publisher = self.create_publisher(Controller, 'topic', 2)

        self.get_logger().info('Publisher node has been started.')

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.cmd_acquisition)


     # Function that prompts user for a direction input, and sends the command
    def cmd_acquisition(self):
        msg = Controller()
        msg.x = self.joystick.get_axis(0)
        msg.y = self.joystick.get_axis(1)
        print(f'Published: x = {msg.x}, y = {msg.y}')
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: x = {msg.x}, y = {msg.y}')


def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher2()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()
    pygame.joystick.quit()


if __name__ == '__main__':
    main()