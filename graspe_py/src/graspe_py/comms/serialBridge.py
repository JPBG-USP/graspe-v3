import serial
import time
from .commands import Command

STX = b"<"
ETX = b">"

class SerialBridge:
    """
    Handles serial communication between the host computer and the ESP32-based
    controller used by the manipulator. This class provides methods for sending
    and receiving framed messages, performing handshake, and exchanging joint data.
    """

    def __init__(
        self,
        port="/dev/ttyACM0",
        baudrate=115200,
        timeout=0.05,
    ):
        """
        Initializes the SerialBridge object and establishes a serial connection.

        Args:
            port (str, optional): Serial port device path (e.g., '/dev/ttyACM0').
            baudrate (int, optional): Communication speed in baud (default: 115200).
            timeout (float, optional): Serial read timeout in seconds (default: 0.05).
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def read_message(self, timeout=None):
        """
        Reads a framed serial message from the serial port.

        Messages are expected in the following format:
            <MESSAGE_CONTENT>

        Args:
            timeout (float, optional): Maximum time to wait (in seconds). If None,
                waits indefinitely until a message is received.

        Returns:
            str | None: The message content between '<' and '>' if received successfully.
                Returns None if a timeout occurs or a decode error happens.

        Notes:
            - The function continuously reads bytes until it detects the start (STX = '<')
              and end (ETX = '>') delimiters.
            - It will ignore any data received outside of these delimiters.
        """
        start_time = time.time()
        buffer = bytearray()

        while True:
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)

                # Detect message start
                if byte == STX:
                    buffer = bytearray()
                    # Read until ETX
                    while True:
                        if self.serial.in_waiting > 0:
                            b = self.serial.read(1)
                            if b == ETX:
                                try:
                                    msg = buffer.decode("utf-8").strip()
                                    return msg
                                except UnicodeDecodeError:
                                    print("[SerialBridge] Decode error.")
                                    return None
                            else:
                                buffer.extend(b)
                        # Check timeout inside inner loop
                        if timeout and (time.time() - start_time > timeout):
                            return None

            if timeout and (time.time() - start_time > timeout):
                return None


    def send_message(self, message: str):
        """
        Sends a framed message through the serial interface.

        The message is automatically wrapped between '<' and '>' before being sent.

        Args:
            message (str): The message content to send.

        Example:
            >>> bridge.send_message("HELLO")
            # Sends "<HELLO>" through the serial port.
        """
        msg = "<" + message + ">"
        self.serial.write(msg.encode("utf-8"))
        self.serial.flush()


    def send_command(self, command: Command):
        """
        Sends a manipulator command to the controller.

        This function automatically converts a Command object into a formatted
        string using `Command.as_string()`, and sends it through the serial port.

        Args:
            command (Command): The command object to send.

        Example:
            >>> cmd = Command(command_type="ManipulatorPosition", q1=0.0, q2=1.0, q3=0.5, q4=-0.5)
            >>> bridge.send_command(cmd)
            # Sends "<POSALL 0.0 1.0 0.5 -0.5>"
        """
        self.send_message(command.as_string())
            

    def handshake(self, timeout=3.0) -> bool:
        """
        Performs a handshake routine with the ESP32 controller.

        The handshake is executed as follows:
            1. Sends the message "<HELLO>".
            2. Waits for a response "<HELLO_ACK>" within the specified timeout.

        Args:
            timeout (float, optional): Maximum waiting time for acknowledgment in seconds.

        Returns:
            bool: True if handshake succeeds, False if timeout or failure occurs.

        Example:
            >>> if bridge.handshake():
            >>>     print("Connected to manipulator.")
        """
        print("[SerialBridge] Starting handshake...")
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.serial.write(b"<HELLO>")
        self.serial.flush()

        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.read_message(timeout=timeout)
            if msg == "HELLO_ACK":
                print("[SerialBridge] Handshake successful.")
                return True
            elif msg:
                print(f"[SerialBridge] Received: {msg}")

        print("[SerialBridge] Handshake failed (timeout).")
        return False

    def get_joints_position(self):
        """
        Reads the current joint positions reported by the controller.

        The expected message format is:
            <POSALL q1 q2 q3 q4>

        Returns:
            tuple[float, float, float, float] | None:
                - A tuple with the four joint positions (q1, q2, q3, q4) if successful.
                - None if no valid message is received or if the format is incorrect.

        Example:
            >>> positions = bridge.get_joints_position()
            >>> if positions:
            >>>     print(f"Joint positions: {positions}")
        """
        msg: str = self.read_message(timeout=1)
        if not msg:
            return None

        if msg.startswith("POSALL"):
            try:
                parts = msg.split()
                if len(parts) != 5:
                    print(f"[SerialBridge] Unexpected message format: {msg}")
                    return None

                positions = tuple(float(x) for x in parts[1:])
                return positions

            except ValueError as e:
                print(f"[SerialBridge] Value conversion error: {e}")
                return None
        else:
            return None
