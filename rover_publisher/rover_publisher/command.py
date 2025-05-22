import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Header
from rclpy.time import Time

QOS_PROFILE_JOY = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=1
)

QOS_PROFILE_ODOM = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    depth=10
)

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('command')
        
        self.publisher = self.create_publisher(Joy, '/cmd_vel', QOS_PROFILE_JOY)
        
        self.subscriber = self.create_subscription(Joy, '/joy', self.listener_callback, QOS_PROFILE_JOY)
        
        self.subscriber_odom = self.create_subscription(String, '/odom', self.odom_callback, QOS_PROFILE_ODOM)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.latest_msg = None
        self.last_processed_msg = None
        
        self.autonomous_mode = False
        self.prev_buttons = [0] * 10
        self.prev_h_dpad = 0
        self.prev_v_dpad = 0
        
        self.axis_threshold = 0.05
        
        self.trajectory = []
        self.current_index = 0
        self.executing = False
        
        self.get_logger().info('Command node has been started.')
        
    def control_loop(self):
        if self.latest_msg is None:
            return

        if not self.joy_changed(self.latest_msg, self.last_processed_msg):
            return 
        self.last_processed_msg = self.latest_msg
        
        buttons = self.latest_msg.buttons

        # Toggle mode 
        if buttons[2] == 1 and self.prev_buttons[2] == 0:
            self.autonomous_mode = not self.autonomous_mode
            if self.autonomous_mode:
                self.executing = False
            mode = "autonome" if self.autonomous_mode else "manuel"
            self.get_logger().info(f"Mode changé : {mode}")

        if not self.autonomous_mode:
            self.publisher.publish(self.latest_msg)
            self.get_logger().info("Publish")
            return 

        # Construction de la trajectoire
        if not self.executing:
            dpad_horizontal = self.latest_msg.axes[6]
            dpad_vertical = self.latest_msg.axes[7]

            if dpad_vertical == 1 and self.prev_v_dpad != 1:
                self.trajectory.append("up")
                self.get_logger().info("Ajout: haut")
            elif dpad_vertical == -1 and self.prev_v_dpad != -1:
                self.trajectory.append("down")
                self.get_logger().info("Ajout: bas")

            if dpad_horizontal == -1 and self.prev_h_dpad != -1:
                self.trajectory.append("right")
                self.get_logger().info("Ajout: droite")
            elif dpad_horizontal == 1 and self.prev_h_dpad != 1:
                self.trajectory.append("left")
                self.get_logger().info("Ajout: gauche")

        # Lancer
        if buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.executing = True
            if self.trajectory:
                self.send_movement_command(self.trajectory[0])
                self.current_index += 1
            self.get_logger().info("Trajectoire lancée.")

        # Pause
        if buttons[1] == 1 and self.prev_buttons[1] == 0:
            self.executing = False
            self.get_logger().info("Trajectoire en pause.")

        # Effacer
        if buttons[3] == 1 and self.prev_buttons[3] == 0:
            self.trajectory.clear()
            self.current_index = 0
            self.executing = False
            self.get_logger().info("Trajectoire effacée.")

        self.prev_h_dpad = self.latest_msg.axes[6]
        self.prev_v_dpad = self.latest_msg.axes[7]
        self.prev_buttons = buttons[:]

    def joy_changed(self, new_msg, old_msg):
        if old_msg == None:
            return True  # Always process first message
        # Check if any axis changed significantly
        for new_val, old_val in zip(new_msg.axes, old_msg.axes):
            if abs(new_val - old_val) > self.axis_threshold:
                return True

        # Check if any button changed
        if new_msg.buttons != old_msg.buttons:
            return True

        return False

    def listener_callback(self, msg):
        self.latest_msg = msg
        
    def odom_callback(self, msg):
        if not self.autonomous_mode or not self.executing:
            return
        
        self.get_logger().info(f'Etape {self.trajectory[self.current_index - 1]} terminée')

        # Logique de suivi de trajectoire
        if self.current_index < len(self.trajectory):
            direction = self.trajectory[self.current_index]
            self.send_movement_command(direction)
            self.current_index += 1
        else:
            self.executing = False
            self.send_movement_command("none")
            self.get_logger().info("Trajectoire terminée.")
        
    def send_movement_command(self, direction):
        msg = Joy()
        msg.header = Header()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = "joy"
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 12

        if direction == "up":
            msg.axes[1] = 1
        elif direction == "down":
            msg.axes[1] = -1
        elif direction == "left":
            msg.axes[0] = 1
        elif direction == "right":
            msg.axes[0] = -1

        self.publisher.publish(msg)
        self.get_logger().info(f"Commande envoyée: {direction}")

def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
