import os
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy
from lib.motor_controller import MotorController
from std_msgs.msg import Float64MultiArray, Float64, Bool
import signal
import sys



class MotorCalibration(Node):
    def __init__(self):
        super().__init__('motor_calibration')
        self.config_path = os.path.join(get_package_share_directory('motor_gen_control'), 'config', 'config.yaml')

        self.declare_parameter('port', '/dev/ttyTHS1')

        self.port = self.get_parameter('port').get_parameter_value().string_value

        self.motor = MotorController(self.port, 1000000)
        self.motor.motor_mode(2)

        if self.motor.init_motor():
            self.get_logger().info(f"Motor calibration initialized on port: {self.port}")
        else:
            self.get_logger().info(f"Failed to initialized motor on port: {self.port}")

        signal.signal(signal.SIGINT, self.close_uart)

        self.zero_encoder_pos = 0.0
        self.min_angle = 0.0
        self.max_angle = 0.0

        self.last_position = None

        self.position = Float64()

        self.create_subscription(Float64MultiArray, '/arm_teleop/joint2/update_limits', self.callback_limits, 10)
        self.create_subscription(Bool, '/arm_teleop/joint2/calibrate', self.callback_calibrate, 10)
        self.pub = self.create_publisher(Float64, '/arm_teleop/joint2/position', 10)
        self.create_timer(0.5, self.publish_position)

        self.get_logger().info('Motor calibration node started')


    def callback_calibrate(self, msg):
        self.zero_encoder_pos = self.motor.get_position()
        self.get_logger().info(f'Zero position: {self.zero_encoder_pos}')
        self.update_yaml_file({'zero_encoder_pos': self.zero_encoder_pos})

    def publish_position(self):
        pos = self.motor.get_position()
        if pos is not None:
            self.position.data = pos
            if self.last_position != pos:  # Solo imprime si la posición cambió
                self.get_logger().info(f'Posición actual: {self.position.data}')
                self.last_position = pos
            self.pub.publish(self.position)
        else:
            #self.get_logger().error('⚠ No se pudo obtener la posición del motor. Verifica la conexión y el puerto.')
            pass

    def close_uart(self, sig, frame) -> None:
        if self.motor:
            self.motor.close_motor()
        self.get_logger().info("Cerrando nodo de calibración.")
        
        if rclpy.ok():  # Asegura que ROS no se apague dos veces
            rclpy.shutdown()
        
        sys.exit(0)


    def callback_limits(self, msg):
        self.min_angle = msg.data[0]
        self.max_angle = msg.data[1]
        
        self.update_yaml_file({
        'min_angle': self.min_angle,
        'max_angle': self.max_angle
    })


    def update_yaml_file(self, params_to_update):
        try:
            if not os.path.exists(self.config_path):
                self.get_logger().error(f'El archivo de configuración no existe: {self.config_path}')
                return

            with open(self.config_path, 'r') as file:
                config_data = yaml.safe_load(file) or {}

            if '/motor_gen_control' not in config_data:
                config_data['/motor_gen_control'] = {'ros__parameters': {}}

            # Actualizar solo los parámetros especificados
            for param, value in params_to_update.items():
                config_data['/motor_gen_control']['ros__parameters'][param] = value

            with open(self.config_path, 'w') as file:
                yaml.dump(config_data, file, default_flow_style=False)

            self.get_logger().info(f'Archivo actualizado: {self.config_path}')
        except Exception as e:
            self.get_logger().error(f'Error al actualizar el archivo: {e}')

def main(args=None):
    rclpy.init(args=args)
    motor_cal = MotorCalibration()
    try:
        rclpy.spin(motor_cal)
    except KeyboardInterrupt:
        pass
    finally:
        motor_cal.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
