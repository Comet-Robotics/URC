import threading
import enum
import logging
import time
import socket
import os
import struct
import json
import datetime
import random
from typing import NamedTuple

logger = logging.getLogger(__name__)

RADIODRIVER_SERIAL_PORT = '/dev/tty.debug-console'
UNIX_SOCKET_PATH = '/var/run/radiodriver-proxy.sock'
SLEEP_TIME = 0.001

# Queue of messages that need to be framed before being sent to the radio driver via serial. Queue is populated by the message transport thread and consumed by the message formatter thread.
unframed_messages_to_tx: list[bytes] = []

# Queue of framed messages that are ready to be sent to the radio driver via serial. Queue is populated by the message formatter thread, consumed by the serial manager thread.
framed_messages_to_tx: list[bytearray] = []

# Queue of lines that have been received from the radio driver via serial. Each line should begin with '[PACKET RX]', then the bytes of the framed message, and ending with a number indicating the number of bytes in the message before it was framed. Queue is populated by the serial manager thread and consumed by the message formatter thread.
lines_read_from_driver: list[bytes] = []
LINE_PREFIX = b'[PACKET RX]'

# Queue of messages that have been unframed by the message formatter thread. Queue is populated by the message transport thread and consumed by the message transport thread.
unframed_messages_from_rx: list[bytes] = []

class KISSChars(enum.Enum):
    FEND = 0xC0 # Frame End
    FESC = 0xDB # Frame Escape
    TFEND = 0xDC # Transposed Frame End
    TFESC = 0xDD # Transposed Frame Escape


def format_kiss_message(data: bytes, tnc_port: int = 0) -> bytearray:
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
    global framed_messages_to_tx
    logger.info("Starting serial manager...")
    with open(RADIODRIVER_SERIAL_PORT, "wb") as ser:
        # switching to nonblocking file reads so we can quickly switch over to tx if there is nothing to read - https://stackoverflow.com/a/66410605
        os.set_blocking(ser.fileno(), False)
        
        logger.info(f"Serial port opened: {ser.name}")
        while True:
            time.sleep(SLEEP_TIME)

            while len(framed_messages_to_tx) > 0:
                m = framed_messages_to_tx.pop(0)
                logger.info("Writing message to serial")
                ser.write(m)
                logger.info("Message written to serial")

            logger.info("Reading line")
            line = ser.readline()
            if len(line) > 0:
              logger.debug(f"Line read: {line}")
              lines_read_from_driver.append(line)
            else:
              logger.debug("No line read")

            
def message_formatter():
    logger = logging.getLogger("MessageFormatter")
    logger.info("Starting message formatter...")
    while True:
        time.sleep(SLEEP_TIME)
        while len(unframed_messages_to_tx) > 0:
            logger.info("Formatting message...")
            formatted_message = format_kiss_message(unframed_messages_to_tx.pop(0))
            logger.info("Formatted message")
            framed_messages_to_tx.append(formatted_message)
            logger.info("Message added to queue")
            

        line = lines_read_from_driver.pop(0)
        
        try:
          parsed_line = parse_data_and_length_from_line(line)
          unframed_messages_from_rx.append(parsed_line.data)
        except ValueError as e:
          logger.error(f"Error parsing line: {e}")


class ParsedLine(NamedTuple):
  data: bytes
  length: int
      
def parse_data_and_length_from_line(line: bytes) -> ParsedLine:
  if line.startswith(LINE_PREFIX):
    stripped_line = line[len(LINE_PREFIX):]
  else:
    raise ValueError("Line does not start with expected prefix", LINE_PREFIX, line)
    
  unframed_data = b''
  while len(stripped_line) > 0:
    # TODO: unframing. raise an exception if the data is badly formed. break out out the loop when we reach the end of the data.
    pass
  
  expected_data_length = int(stripped_line)
  if len(unframed_data) != expected_data_length:
    raise ValueError("Expected data length does not match actual data length", expected_data_length, len(unframed_data))
  
  return ParsedLine(data=unframed_data, length=expected_data_length)
    
  

def mock_message_transport():
    logger = logging.getLogger("MockMessageTransport")
    logger.info("Starting message transport...")
    while True:
        fake_msg = json.dumps({"random": random.random(), "time": datetime.datetime.now().isoformat()})
        logger.info(f"Message from process: {fake_msg}")
        unframed_messages_to_tx.append(fake_msg.encode())

        while len(unframed_messages_from_rx) > 0:
            msg = unframed_messages_from_rx.pop(0)
            logger.info(f"Message from radio: {msg}")
            
        time.sleep(SLEEP_TIME)

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

def unix_socket_message_transport():
    logger = logging.getLogger("UnixSocketMessageTransport")
    logger.info("Starting Unix socket message transport...")
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
                unframed_messages_to_tx.append(buf)
                logger.debug("Received message")
                listening_for = ListeningFor.PAYLOAD_SIZE
                
            if len(unframed_messages_from_rx) > 0:
                msg = unframed_messages_from_rx.pop(0)
                conn.sendall(msg)
                logger.debug("Message sent")
        except ConnectionError as e:
            logger.error(f"Connection error: {e}")
            break  # Exit loop on connection error
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            listening_for = ListeningFor.PAYLOAD_SIZE  # Reset state on error
            continue

        time.sleep(SLEEP_TIME)

def main():
    logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(asctime)s | %(name)s: %(message)s',)
    logger.info("--- STARTING RADIODRIVER PROXY ---")

    message_transport_thread = threading.Thread(target=unix_socket_message_transport)
    message_transport_thread.start()

    serial_manager_thread = threading.Thread(target=serial_manager)
    serial_manager_thread.start()

    message_formatter_thread = threading.Thread(target=message_formatter)
    message_formatter_thread.start()

if __name__ == "__main__":
    main()