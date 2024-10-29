import rclpy  # Importing ROS2 Python client library
from rclpy.node import Node  # Importing Node class from rclpy
from std_msgs.msg import String  # Importing standard message type String

class GpioCommandPublisher(Node):  # Defining a class that inherits from Node
    def __init__(self):
        super().__init__('gpio_command_publisher')  # Initializing the parent Node class
        self.publisher_ = self.create_publisher(String, 'gpio_command', 10)  # Creating a publisher to the 'gpio_command' topic
        timer_period = 2.0  # Time period in seconds for the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Creating a timer that calls the timer_callback function every 2 seconds
        self.command = 'on'  # Initial command set to 'on'

    def timer_callback(self):
        msg = String()  # Creating a String message object
        msg.data = self.command  # Setting the message data to the current command
        self.publisher_.publish(msg)  # Publishing the message to the 'gpio_command' topic
        self.get_logger().info(f'Publishing: {msg.data}')  # Logging the published message

        # Toggle command for next publish
        if self.command == 'on':
            self.command = 'off'
        else:
            self.command = 'on'

def main(args=None):
    rclpy.init(args=args)  # Initializing the ROS2 Python library
    node = GpioCommandPublisher()  # Creating an instance of the GpioCommandPublisher node

    try:
        rclpy.spin(node)  # Spinning the node to keep it active
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Destroying the node when done
        rclpy.shutdown()  # Shutting down ROS2

if __name__ == '__main__':
    main()  # Running the main function

