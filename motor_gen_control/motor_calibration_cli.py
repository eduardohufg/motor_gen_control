import os
import yaml
import rclpy
import sys
import termios
import tty
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from lib.motor_controller import MotorController
import signal

def get_key():
    """Captura una tecla presionada sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class MotorCalibrationCLI(Node):
    def __init__(self):
        super().__init__('motor_calibration_cli')
        
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('username', 'eduardohufg')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.username = self.get_parameter('username').get_parameter_value().string_value

        self.config_path = os.path.join(get_package_share_directory('motor_gen_control'), 'config', 'config.yaml')
        self.config_path2 = os.path.join(f'/home/{self.username}/ros2_ws/src/motor_gen_control/config', 'config.yaml')
        
        self.motor = MotorController(self.port, 1000000, 3)
        
        if self.motor.init_motor():
            self.get_logger().info(f"Motor connected on port: {self.port}")
        else:
            self.get_logger().error(f"Failed to initialize motor on port: {self.port}")
            sys.exit(1)
        
        self.run_cli()
        signal.signal(signal.SIGINT, self.close_uart)

    def close_uart(self, sig, frame) -> None:
        if self.motor and self.motor.uart and self.motor.uart.is_open:
            self.motor.close_motor()
            self.get_logger().info("Motor closed.")
        
        if rclpy.ok(): 
            rclpy.shutdown()
        sys.exit(0)

    def run_cli(self):
        """Ejecuta la interfaz de calibración desde la terminal."""
        self.get_logger().info("Presiona 'q' para calibrar el motor a la posición actual.")
        while True:
            pos = self.motor.get_position()
            if pos is not None:
                print(f'\rPosición actual del motor: {pos:.2f}   ', end='', flush=True)
            else:
                self.get_logger().warn("No se pudo obtener la posición del motor.")
            
            key = get_key()
            if key == 'q':
                self.calibrate_motor()
                break
        
        self.get_logger().info("¿Quieres cambiar los límites de los ángulos? (s/n): ")
        change_limits = input().strip().lower()
        if change_limits == 's':
            self.set_limits()
        
        self.get_logger().info("Calibración finalizada.")
        rclpy.shutdown()
        sys.exit(0)
    
    def calibrate_motor(self):
        """Guarda la posición actual como la nueva posición cero."""
        zero_position = self.motor.get_position()
        self.get_logger().info(f'Nueva posición cero: {zero_position:.2f}')
        self.update_yaml_file(self.config_path, {'zero_encoder_pos': zero_position})
        self.update_yaml_file(self.config_path2, {'zero_encoder_pos': zero_position})
    
    def set_limits(self):
        """Permite ingresar nuevos límites de ángulo manualmente."""
        try:
            min_angle = float(input("Ingrese el nuevo ángulo mínimo: "))
            max_angle = float(input("Ingrese el nuevo ángulo máximo: "))
            self.update_yaml_file(self.config_path, {'min_angle': min_angle, 'max_angle': max_angle})
            self.update_yaml_file(self.config_path2, {'min_angle': min_angle, 'max_angle': max_angle})
            self.get_logger().info(f'Límites actualizados: min={min_angle}, max={max_angle}')
        except ValueError:
            self.get_logger().error("Entrada inválida. No se actualizaron los límites.")
    
    def update_yaml_file(self, file_path, params_to_update):
        """Actualiza el archivo de configuración YAML."""
        try:
            if not os.path.exists(file_path):
                self.get_logger().error(f'El archivo de configuración no existe: {file_path}')
                return
            
            with open(file_path, 'r') as file:
                config_data = yaml.safe_load(file) or {}
            
            if '/motor_gen_control' not in config_data:
                config_data['/motor_gen_control'] = {'ros__parameters': {}}
            
            for param, value in params_to_update.items():
                config_data['/motor_gen_control']['ros__parameters'][param] = value
            
            with open(file_path, 'w') as file:
                yaml.dump(config_data, file, default_flow_style=False)
            
            self.get_logger().info(f'Archivo actualizado: {file_path}')
        except Exception as e:
            self.get_logger().error(f'Error al actualizar el archivo: {e}')

def main(args=None):
    rclpy.init(args=args)
    motor_cal = MotorCalibrationCLI()
    try:
        rclpy.spin(motor_cal)
    except KeyboardInterrupt:
        pass
    finally:
        motor_cal.destroy_node()
        rclpy.shutdown()

    

if __name__ == '__main__':
    main()
