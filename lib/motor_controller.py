import serial
import time

class MotorController:
    def __init__(self, port, baudrate=115200, mode=0) -> None:

        """
        Initialize the motor controller object
        :param port: Serial port to connect to the motor controller
        :param baudrate: Baudrate of the serial port
        """

        self.port = port
        self.baudrate = baudrate
        try:
            self.uart = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            if self.uart.is_open:
                self.motor_mode(mode)
        except serial.SerialException as e:
            self.uart = None

    def __del__(self) -> None:
        if self.uart and self.uart.is_open:
            try:
                commad = "STOP\n"
                self.uart.write(commad.encode())
                self.uart.close()
            except Exception:
                pass

    def init_motor(self) -> bool:
        if self.uart is not None:
            self.uart.write("INIT\n".encode())
            return True
        else:
            return False
        
    def close_motor(self) -> bool:
        if self.uart is not None:
            self.uart.write("STOP\n".encode())
            self.uart.close()
            return True
        else:
            return False
        
    def motor_mode(self, mode) -> bool:
        if self.uart is not None and self.uart.is_open:
            command = f"MODE,{mode}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False

    def set_pid_gains(self, kp, ki, kd) -> bool:
        if self.uart is not None:
            command = f"PID,{kp},{ki},{kd}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False
        
    def set_setpoint(self, setpoint) -> bool:
        if self.uart is not None:
            command = f"SP,{setpoint}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False
        
    def get_position(self) -> str:
        if self.uart is not None:
            self.uart.write("GET_POS\n".encode())
            return self.uart.readline().decode().strip()
        
    def get_velocity(self) -> str:
        if self.uart is not None:
            self.uart.write("GET_VEL\n".encode())
            return self.uart.readline().decode().strip()
        
    def set_windup_limit(self, limit) -> bool:
        if self.uart is not None:
            command = f"LIMIT,{limit}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False
        
    def set_motor_direction(self, direction) -> bool:
        if self.uart is not None:
            command = f"DIRECTION,{direction}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False
        
    def set_sensor_phase(self, phase) -> bool:
        if self.uart is not None:
            command = f"PHASE,{phase}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False
        
    def set_saturation(self, saturation) -> bool:
        if self.uart is not None:
            command = f"SAT,{saturation}\n"
            self.uart.write(command.encode())
            return True
        else:
            return False