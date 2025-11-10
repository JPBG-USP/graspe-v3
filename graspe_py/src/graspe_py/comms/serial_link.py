import threading
import serial
import queue
import time

STX = b'<'   # byte inicializador
ETX = b'>'   # byte finalizador

class SerialLink:
    """
    Comunicação simples via serial (<mensagem>), com thread de leitura assíncrona.
    """

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.serial = None
        self._running = False
        self._rx_buffer = bytearray()
        self._rx_queue = queue.Queue(maxsize=50)
        self._rx_thread = None

    # ---------- Conexão ----------
    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._running = True
            self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._rx_thread.start()
            print(f"[SerialLink] Conectado a {self.port} @ {self.baudrate} bps")
            return True
        except serial.SerialException as e:
            print(f"[SerialLink] Erro ao conectar: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[SerialLink] Porta serial fechada")

    def is_connected(self):
        return self.serial and self.serial.is_open

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

    # ---------- Loop de leitura ----------
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
            return None
        
    def send(self, msg: str):
        """Envia mensagem <msg>"""
        if not self.is_connected():
            print("[SerialLink] Não conectado.")
            return False
        frame = STX + msg.encode() + ETX
        try:
            self.serial.write(frame)
            print(f"[TX] {msg}")
            return True
        except serial.SerialException as e:
            print(f"[SerialLink] Erro no envio: {e}")
            return False

    # ---------- Função para obter a posição do motor 2 ----------
    def obter_posicao_motor(self):
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