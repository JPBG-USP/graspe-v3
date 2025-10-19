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
        self._rx_queue = queue.Queue()
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
                                self._rx_queue.put(msg)
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

    # ---------- Função para obter a posição do motor 1 ----------
    def obter_posicao_motor(self):
        msg = self.get_message(timeout=1)  # Espera até 1 segundo pela mensagem
        if msg:
            if msg.startswith("<posq1"):
                dados = msg[6:].strip('>')  # Retira o "<posq1" e captura o valor
                try:
                    motor_position = float(dados)  # Converte o valor para float
                    return motor_position  # Retorna a posição do motor 1
                except ValueError:
                    print("Erro ao converter os dados de posição.")
            elif msg == "<ACK>":
                print("[SerialLink] Recebido ACK da ESP32")
        return None
