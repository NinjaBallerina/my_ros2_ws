import rclpy  # Importing ROS2 Python client library
from rclpy.node import Node  # Importing Node class from rclpy
from std_msgs.msg import String  # Importing standard message type String
import RPi.GPIO as GPIO  # Importing GPIO library for Raspberry Pi

# Set up GPIO
GPIO.setmode(GPIO.BCM)  # Setting GPIO mode to BCM (Broadcom SOC channel)
GPIO.setup(18, GPIO.OUT)  # Setting GPIO pin 18 as output

class GpioControlNode(Node):  # Defining a class that inherits from Node
    def __init__(self):
        super().__init__('gpio_control_node')  # Initializing the parent Node class
        self.publisher_ = self.create_publisher(String, 'gpio_status', 10)  # Creating a publisher to the 'gpio_status' topic
        self.subscription = self.create_subscription(String, 'gpio_command', self.listener_callback, 10)  # Creating a subscription to the 'gpio_command' topic
        self.subscription  # Preventing unused variable warning

    def listener_callback(self, msg):
        command = msg.data.lower()  # Retrieving the command from the received message
        if command == 'on':
            GPIO.output(18, GPIO.HIGH)  # Setting GPIO pin 18 to high (on)
            self.get_logger().info('GPIO pin 18 turned on.')  # Logging the status
            self.publisher_.publish(String(data='GPIO pin 18 is on'))  # Publishing the status
        elif command == 'off':
            GPIO.output(18, GPIO.LOW)  # Setting GPIO pin 18 to low (off)
            self.get_logger().info('GPIO pin 18 turned off.')  # Logging the status
            self.publisher_.publish(String(data='GPIO pin 18 is off'))  # Publishing the status

def main(args=None):
    rclpy.init(args=args)  # Initializing the ROS2 Python library
    node = GpioControlNode()  # Creating an instance of the GpioControlNode

    try:
        rclpy.spin(node)  # Spinning the node to keep it active
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Destroying the node when done
        rclpy.shutdown()  # Shutting down ROS2
        GPIO.cleanup()  # Cleaning up GPIO

if __name__ == '__main__':
    main()  # Running the main function
