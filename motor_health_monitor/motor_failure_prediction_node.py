import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ESCStatus

class MotorFailureDetector(Node):
    def __init__(self):
        super().__init__('motor_failure_detector')
        self.subscription = self.create_subscription(
            ESCStatus,
            '/mavros/esc_status/status',
            self.esc_status_callback,
            10)

    def esc_status_callback(self, msg):
        # Extract RPM values from the ESC status
        rpm_values = [esc.rpm for esc in msg.esc_status]
        avg_rpm = sum(rpm_values) / len(rpm_values)
        
        # Check for significant deviations in RPM
        for idx, rpm in enumerate(rpm_values):
            if rpm < avg_rpm * 0.7:  # 30% below average, possible failure
                self.get_logger().warn(f"Motor {idx + 1} possible failure: RPM {rpm}")
            else:
                self.get_logger().info(f"Motor {idx + 1} RPM: {rpm}")

def main(args=None):
    rclpy.init(args=args)
    motor_failure_detector = MotorFailureDetector()
    rclpy.spin(motor_failure_detector)
    motor_failure_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
