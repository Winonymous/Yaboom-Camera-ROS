import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time
import tf2_ros
import geometry_msgs.msg

class ServoControlNode(Node):

    def __init__(self):
        super().__init__('servo_control_node')

        # Setup GPIO
        self.servo_pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # Set up PWM for servo control
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)

        # Create a TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to periodically check the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz frequency

        self.get_logger().info('Servo control node has been started and listening for transforms.')

    def timer_callback(self):
        try:
            # Look up the transform from 'base_link' to 'camera_link'
            transform = self.tf_buffer.lookup_transform('base_link', 'arm', rclpy.time.Time())

            # Extract the position (or orientation) from the transform
            # Here, we assume we want to control the servo based on the X position of the camera
            x_position = transform.transform.translation.x

            # Log the position and calculate the angle
            self.get_logger().info(f'Camera X position: {x_position}')

            # Map the X position to a servo angle (you can adjust the scale factor here)
            # For example, map X in range [-2.0, 2.0] meters to [0, 180] degrees
            angle = self.map_position_to_angle(x_position)
            self.get_logger().info(f'Mapped angle: {angle}')

            # Control the servo
            self.set_angle(angle)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Transform not available yet.')

    def set_angle(self, angle):
        """Convert angle to PWM duty cycle and move the servo."""
        duty_cycle = (angle / 18) + 2
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(1)

    def map_position_to_angle(self, position):
        """Map position to an angle for servo control."""
        # Example: map position range [-2.0, 2.0] to [0, 180]
        min_position = -2.0  # min camera X position (in meters)
        max_position = 2.0   # max camera X position (in meters)
        min_angle = 0        # min angle (degrees)
        max_angle = 180      # max angle (degrees)

        # Ensure the position is within the range
        position = max(min(position, max_position), min_position)

        # Map position to angle
        angle = ((position - min_position) / (max_position - min_position)) * (max_angle - min_angle) + min_angle
        return angle

    def cleanup(self):
        """Clean up PWM and GPIO when shutting down."""
        self.pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)

    servo_control_node = ServoControlNode()

    try:
        rclpy.spin(servo_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_control_node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()