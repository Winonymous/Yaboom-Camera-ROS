import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import RPi.GPIO as GPIO
import time
import math

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Servo and GPIO setup
        self.SERVO_PIN = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.SERVO_PIN, 50)  # 50Hz
        self.pwm.start(0)

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        try:
            # Look up transform between two frames
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'target_frame', rclpy.time.Time())
            
            # Example: use X position to compute servo angle
            x_position = trans.transform.translation.x
            self.get_logger().info(f"Transform X: {x_position:.2f}")

            # Convert x_position to angle (you can change this logic!)
            angle = max(0, min(180, int((x_position + 1.0) * 90)))  # Map from [-1, 1] to [0, 180]
            self.set_servo_angle(angle)

        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")

    def set_servo_angle(self, angle):
        duty = 2.5 + (angle / 180.0) * 10
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  # Prevent jitter

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
