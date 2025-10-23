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

    def handshake(self, retries=3, wait_time=0.5):
        """Envia <HELLO> e aguarda <HELLO_ACK>."""
        for i in range(retries):
            self.send("HELLO")
            start = time.time()
            while time.time() - start < wait_time:
                msg = self.get_message(timeout=0.1)
                if msg == "HELLO_ACK":
                    print("[SerialLink] Handshake bem-sucedido")
                    return True
            print(f"[SerialLink] Tentativa {i+1}/{retries} falhou.")
        print("[SerialLink] Handshake falhou após tentativas.")
        return False

    # ---------- Loop de leitura ----------
    def _read_loop(self):
        print("[SerialLink] Thread de leitura iniciada")
        while self._running and self.serial and self.serial.is_open:
            try:
                data = self.serial.read(64)
                if data:
                    for b in data:
                        self._rx_buffer.append(b)
                        if b == ETX[0]:
                            start_idx = self._rx_buffer.find(STX)
                            if start_idx != -1:
                                frame = self._rx_buffer[start_idx+1:-1]
                                msg = frame.decode(errors="ignore").strip()
                                if self._rx_queue.full():
                                    self._rx_queue.get_nowait() 
                                self._rx_queue.put_nowait(msg)

                            self._rx_buffer.clear()
            except Exception as e:
                print(f"[SerialLink] Erro na leitura: {e}")
                time.sleep(0.1)
        print("[SerialLink] Thread de leitura finalizada")

    # ---------- Recepção ----------
    def get_message(self, timeout=None):
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
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
        Lê mensagens no formato <posqXVALOR> e retorna (X, VALOR)
        Exemplo: "<posq2 123.456>" -> (2, 123.456)
        """
        msg: str = self.get_message(timeout=1)
        if not msg:
            print("not msg")
            return None, None

        # Exemplo esperado: <posq1 123.456>
        if msg.startswith("posq"):
            try:
                # Extrai o número do motor logo após 'posq'
                if msg[4].isdigit():
                    motor_index = int(msg[4])
                else:
                    print("Fudeu, não é digito")

                # Extrai o valor numérico restante
                valor_str = msg[5:]
                print(msg)
                print(valor_str)
                motor_position = float(valor_str)

                return motor_index, motor_position

            except Exception as e:
                print(f"[SerialLink] Erro ao interpretar mensagem de posição: {e}")

        elif msg == "<ACK>":
            print("[SerialLink] Recebido ACK da ESP32")

        else:
            print("[SerialLink] Mensagem não começa com <posq")
        return None, None
