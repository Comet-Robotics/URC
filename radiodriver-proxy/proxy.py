import threading
import enum
import logging
import time
import socket
logger = logging.getLogger(__name__)

RADIODRIVER_SERIAL_PORT = '/dev/tty.debug-console'
UNIX_SOCKET_PATH = '/var/run/radiodriver-proxy.sock'
SLEEP_TIME = 0.001

messages: list[bytes] = []
kiss_message_queue: list[bytes] = []

class KISSChars(enum.Enum):
    FEND = 0xC0 # Frame End
    FESC = 0xDB # Frame Escape
    TFEND = 0xDC # Transposed Frame End
    TFESC = 0xDD # Transposed Frame Escape


def format_kiss_message(data: bytes, tnc_port: int = 0) -> bytes:
    tnc_port_valid = tnc_port >= 0 and tnc_port <= 9
    if not tnc_port_valid:
        raise ValueError("TNC Port must be between 0 and 9")
    
    packet = bytearray()
    packet.append(KISSChars.FEND.value)

    dataframe = tnc_port << 4
    packet.append(dataframe)

    for char in data:
        if char == KISSChars.FEND.value:
            packet.append(KISSChars.FESC.value)
            packet.append(KISSChars.TFEND.value)
        elif char == KISSChars.FESC.value:
            packet.append(KISSChars.FESC.value)
            packet.append(KISSChars.TFESC.value)
        else:
            packet.append(char)
    
    packet.append(KISSChars.FEND.value)
    return packet

def serial_manager():
    logger = logging.getLogger("SerialManager")
    logger.info("Starting serial manager...")
    with open(RADIODRIVER_SERIAL_PORT, "wb") as ser:
        logger.info(f"Serial port opened: {ser.name}")
        while True:
            time.sleep(0.001)
            while len(kiss_message_queue) > 0:
                m = kiss_message_queue.pop(0)
                logger.info("Writing message to serial")
                ser.write(m)
                logger.info("Message written to serial")

def message_formatter():
    logger = logging.getLogger("MessageFormatter")
    logger.info("Starting message formatter...")
    while True:
        time.sleep(0.001)
        while len(messages) > 0:
            logger.info("Formatting message...")
            formatted_message = format_kiss_message(messages.pop(0))
            logger.info("Formatted message")
            kiss_message_queue.append(formatted_message)
            logger.info("Message added to queue")
            

def mock_message_receiver():
    logger = logging.getLogger("MockMessageReceiver")
    logger.info("Starting message receiver...")
    import json, datetime, random
    while True:
        fake_msg = json.dumps({"random": random.random(), "time": datetime.datetime.now().isoformat()})
        logger.info(f"Received message: {fake_msg}")
        messages.append(fake_msg)

class ListeningFor(enum.IntEnum):
    PAYLOAD_SIZE = 0
    PAYLOAD = 1

DEFAULT_CHUNK_SIZE = 1024
def receive_all(conn: socket.socket, size: int) -> bytes:
    """
    Receive an exact number of bytes from a socket connection.
    Implements chunked reading with a reasonable buffer size.
    """
    chunks = []
    bytes_recd = 0
    while bytes_recd < size:
        remaining_bytes = size - bytes_recd

        chunk = conn.recv(min(remaining_bytes, DEFAULT_CHUNK_SIZE))
        if not chunk:  # Connection was closed
            raise ConnectionError("Socket connection closed while receiving data")
        chunks.append(chunk)
        bytes_recd += len(chunk)
    return b''.join(chunks)

def unix_socket_message_receiver():
    import socket, struct, os

    logger = logging.getLogger("UnixSocketMessageReceiver")
    logger.info("Starting Unix socket message receiver...")
    os.unlink(UNIX_SOCKET_PATH)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(UNIX_SOCKET_PATH)
    sock.listen(1)
    logger.info(f"Listening on {UNIX_SOCKET_PATH}")

    listening_for = ListeningFor.PAYLOAD_SIZE
    expected_payload_size = -1

    conn, addr = sock.accept()
    while True:
        logger.debug(f"Listening for next {listening_for.name}...")
        try:
            if listening_for == ListeningFor.PAYLOAD_SIZE:
                buf = receive_all(conn, 4)  # Always read exactly 4 bytes for size
                size = struct.unpack('!I', buf)[0]
                logger.info(f"Expected payload size: {size}")
                expected_payload_size = size
                listening_for = ListeningFor.PAYLOAD
            elif listening_for == ListeningFor.PAYLOAD:
                buf = receive_all(conn, expected_payload_size)
                messages.append(buf)
                logger.debug("Received message")
                listening_for = ListeningFor.PAYLOAD_SIZE
        except ConnectionError as e:
            logger.error(f"Connection error: {e}")
            break  # Exit loop on connection error
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            listening_for = ListeningFor.PAYLOAD_SIZE  # Reset state on error
            continue

        time.sleep(SLEEP_TIME)

def main():
    logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(asctime)s: %(message)s',)
    logger.info("--- STARTING RADIODRIVER PROXY ---")

    message_receiver_thread = threading.Thread(target=unix_socket_message_receiver)
    message_receiver_thread.start()

    serial_thread = threading.Thread(target=serial_manager)
    serial_thread.start()

    message_formatter_thread = threading.Thread(target=message_formatter)
    message_formatter_thread.start()

if __name__ == "__main__":
    main()