import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from lib.motor_controller import MotorController
from rcl_interfaces.msg import SetParametersResult
import signal
import sys

def my_map(value: float, in_min: float, in_max,out_min: float, out_max: float) -> float:

    targetPos: float = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return targetPos

def get_min_max_encoder_pos(zero_encoder_pos: float, min_angle_pos: float, max_angle_pos: float) -> tuple:
    current_180 = 2011.0

    min_encoder_pos: float = min_angle_pos * current_180 / 180.0 + zero_encoder_pos
    max_encoder_pos: float = max_angle_pos * current_180 / 180.0 + zero_encoder_pos

    return min_encoder_pos, max_encoder_pos
        
class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('kp', 0.11)
        self.declare_parameter('ki', 0.11)
        self.declare_parameter('kd', 0.005)
        self.declare_parameter('min_angle', -10.0)
        self.declare_parameter('max_angle', 180.0)
        self.declare_parameter('zero_encoder_pos', 2011.0)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value   
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.min_angle = self.get_parameter('min_angle').get_parameter_value().double_value
        self.max_angle = self.get_parameter('max_angle').get_parameter_value().double_value
        self.zero_encoder_pos = self.get_parameter('zero_encoder_pos').get_parameter_value().double_value

        self.min_encoder_pos, self.max_encoder_pos = get_min_max_encoder_pos(self.zero_encoder_pos, self.min_angle, self.max_angle)

        self.create_subscription(Float64, '/arm_teleop/joint2', self.callback_joint2, 10)

        self.add_on_set_parameters_callback(self.parameter_callback)    
        self.angle_joint2: float = 0.0

        self.motor = MotorController(self.port, 1000000, 0)
        self.motor.set_pid_gains(self.kp, self.ki, self.kd)
        self.motor.set_windup_limit(20)
    
        if self.motor.init_motor():
            self.get_logger().info(f"Motor controller initialized on port: {self.port}")
        else:
            self.get_logger().info(f"Failed to initialized motor on pott: {self.port}")

        signal.signal(signal.SIGINT, self.close_uart)

    def parameter_callback(self, params) -> None:
        
        for param in params:
            if param.name == 'kp':

                if param.value < 0.0:
                    self.get_logger().warn("kp cannot be negative")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp = param.value.double_value
                    self.motor.set_pid_gains(self.kp, self.ki, self.kd)
                    self.get_logger().info(f"New kp value: {self.kp}")
            elif param.name == 'ki':

                if param.value < 0.0:
                    self.get_logger().warn("ki cannot be negative")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki = param.value.double_value
                    self.motor.set_pid_gains(self.kp, self.ki, self.kd)
                    self.get_logger().info(f"New ki value: {self.ki}")
            elif param.name == 'kd':
                
                if param.value < 0.0:
                    self.get_logger().warn("kd cannot be negative")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd = param.value.double_value
                    self.motor.set_pid_gains(self.kp, self.ki, self.kd)
                    self.get_logger().info(f"New kd value: {self.kd}")

            elif param.name == 'min_angle':
                if param.value.double_value < -180.0:
                    self.get_logger().warn("min_angle cannot be less than -180.0")
                    return SetParametersResult(successful=False, reason="min_angle cannot be less than -180.0")
                else:
                    self.min_angle = param.value.double_value
                    self.min_encoder_pos, self.max_encoder_pos = get_min_max_encoder_pos(self.zero_encoder_pos, self.min_angle, self.max_angle)
                    self.get_logger().info(f"New min_angle value: {self.min_angle}")

            elif param.name == 'max_angle':

                if param.value.double_value > 180.0:
                    self.get_logger().warn("max_angle cannot be greater than 180.0")
                    return SetParametersResult(successful=False, reason="max_angle cannot be greater than 180.0")
                else:

                    self.max_angle = param.value.double_value
                    self.min_encoder_pos, self.max_encoder_pos = get_min_max_encoder_pos(self.zero_encoder_pos, self.min_angle, self.max_angle)
                    self.get_logger().info(f"New max_angle value: {self.max_angle}")

            elif param.name == 'zero_encoder_pos':
                self.zero_encoder_pos = param.value.double_value
                self.min_encoder_pos, self.max_encoder_pos = get_min_max_encoder_pos(self.zero_encoder_pos, self.min_angle, self.max_angle)
                self.get_logger().info(f"New zero_encoder_pos value: {self.zero_encoder_pos}")

    def close_uart(self, sig, frame) -> None:
        if self.motor and self.motor.uart and self.motor.uart.is_open:
            self.motor.close_motor()
            self.get_logger().info("Motor closed.")
        
        if rclpy.ok(): 
            rclpy.shutdown()
        sys.exit(0)

    def callback_joint2(self, msg: Float64) -> None:
        self.angle_joint2 = float(msg.data)
        self.joint_arm_2()

    def joint_arm_2(self) -> None:

        if self.motor is None:
            return
        
        if(self.angle_joint2 >= self.min_angle and self.angle_joint2 <= self.max_angle):

            targetPos = my_map(self.angle_joint2, self.min_angle, self.max_angle, self.min_encoder_pos, self.max_encoder_pos)
            self.motor.set_setpoint(targetPos)

        else:
            self.get_logger().info(f"Angle out of range: {self.angle_joint2}")
            
def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControl()
    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        motor_control.get_logger().info("Interrupción detectada, cerrando el nodo...")
    finally:
        motor_control.close_uart(None, None)
        motor_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    