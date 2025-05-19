import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from sensor_msgs import Joy
from std_msgs.msg import String

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
        
        self.autonomous_mode = False
        self.prev_buttons = [0] * 10
        
        self.trajectory = []
        self.current_index = 0
        self.executing = False
        
        self.get_logger().info('Command node has been started.')

    def listener_callback(self, msg):
        buttons = msg.buttons

        # Toggle mode 
        """if buttons[2] == 1 and self.prev_buttons[2] == 0:
            self.autonomous_mode = not self.autonomous_mode
            mode = "autonome" if self.autonomous_mode else "manuel"
            self.get_logger().info(f"Mode changé : {mode}")"""
            
        if buttons[2] == 1:
            self.get_logger().info("Index 2")
        
        if buttons[1] == 1:
            self.get_logger().info("Index 1")
            
        if buttons[0] == 1:
            self.get_logger().info("Index 0")
            
        if buttons[3] == 1:
            self.get_logger().info("Index 3")

        if not self.autonomous_mode:
            self.publisher.publish(msg)
            return 

        # Construction de la trajectoire
        if not self.executing:
            dpad_horizontal = msg.axes[6]
            dpad_vertical = msg.axes[7]

            if dpad_vertical == 1:
                self.trajectory.append("up")
                self.get_logger().info("Ajout: haut")
            elif dpad_vertical == -1:
                self.trajectory.append("down")
                self.get_logger().info("Ajout: bas")

            if dpad_horizontal == 1:
                self.trajectory.append("right")
                self.get_logger().info("Ajout: droite")
            elif dpad_horizontal == -1:
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
        if buttons[2] == 1 and self.prev_buttons[2] == 0:
            self.trajectory.clear()
            self.current_index = 0
            self.executing = False
            self.get_logger().info("Trajectoire effacée.")

        self.prev_buttons = buttons[:]
        
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
            self.get_logger().info("Trajectoire terminée.")
        
    def send_movement_command(self, direction):
        msg = Joy()
        msg.axes[0] = 0
        msg.axes[1] = 0

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