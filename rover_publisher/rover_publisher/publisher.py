import rclpy
from rclpy.node import Node
import curses
from curses import wrapper
from custom_msg.msg import Command
        

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.publisher = self.create_publisher(Command, 'topic', 10)

        self.get_logger().info('Publisher node has been started.')
        
        self.stdscr = curses.initscr()
        self.curses.noecho()
        self.curses.cbreak()
        self.stdscr.keypad(True)
        
        self.prev_msg = Command()
        
        begin_x = 20; begin_y = 7
        height = 5; width = 40
        self.win = curses.newwin(height, width, begin_y, begin_x)

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.cmd_acquisition)


     # Function that prompts user for a direction input, and sends the command
    def cmd_acquisition(self):
        msg = Command()
        msg.x = self.prev_msg.x
        c = self.stdscr.getch()
        if c == ord('z'):
            msg.x = 1
        if c == ord('s'):
            msg.x = 0
        if c == ord('a'):
            msg.x = -1
        if c == ord('e'):
            msg.speed = 1
        if c == ord('d'):
            msg.speed = -1
        self.publisher.publish(msg)
        self.prev_msg = msg
        self.get_logger().info(f'Published: x = {msg.x}, speed = {msg.speed}')


def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()
    #curses.endwin()


if __name__ == '__main__':
    wrapper(main())