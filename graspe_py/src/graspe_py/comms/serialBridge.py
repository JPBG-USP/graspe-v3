import serial
import time

STX = b"<"
ETX = b">"

class SerialBridge:
    def __init__(
        self,
        port="/dev/ttyACM0",
        baudrate=115200,
        timeout=0.05,
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def read_message(self, timeout=None):
        """
        Reads a serial message in the format:
            <msg>
        Returns:
            str: The message content between '<' and '>', or None if timeout occurs.
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

    def handshake(self, timeout=3.0) -> bool:
        """
        Performs handshake with ESP32:
            Sends <HELLO> and waits for <HELLO_ACK>.
        Returns:
            bool: True if handshake succeeds, False otherwise.
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
        Reads a message in the format:
            <POSALL q1 q2 q3 q4>
        Returns:
            tuple[float, float, float, float]: The joint positions, or None if the format is invalid.
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
