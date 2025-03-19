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

logger = logging.getLogger(__name__)

DEV = True
RADIODRIVER_SERIAL_PORT = '/dev/tty.debug-console'
UNIX_SOCKET_PATH = '/var/run/radiodriver-proxy.sock'

# slowing down so I can read the logs ;)
SLEEP_TIME = 1 if DEV else 0.001

# Queue of messages that need to be framed before being sent to the radio driver via serial. Queue is populated by the message transport thread and consumed by the message formatter thread.
unframed_messages_to_tx: list[bytes] = []

# Queue of framed messages that are ready to be sent to the radio driver via serial. Queue is populated by the message formatter thread, consumed by the serial manager thread.
framed_messages_to_tx: list[bytearray] = []

# Queue of messages that have been unframed by the message formatter thread. Queue is populated by the message formatter thread and consumed by the message transport thread.
unframed_messages_from_rx: list[bytearray] = []

# Buffer used to store bytes as they are received from the serial port. Populated by the serial manager thread, consumed by the message formatter thread. Once a full message has been received, it is added to the framed_messages_from_rx queue.
serial_read_buffer = b''

# Lock used to ensure that only one thread is accessing the serial_read_buffer at a time.
serial_read_buffer_lock = threading.Lock()

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
    global framed_messages_to_tx, serial_read_buffer, serial_read_buffer_lock
    logger.info("Starting serial manager...")
    with open(RADIODRIVER_SERIAL_PORT, "w+b") as ser:
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

            chars = ser.read()
            if chars and len(chars) > 0:
              with serial_read_buffer_lock:
                serial_read_buffer += chars


class KISSPacketParserStates(enum.Enum):
  EXPECTING_INITIAL_FEND = 0
  EXPECTING_COMMAND = 1
  EXPECTING_DATA_OR_FINAL_FEND = 2
  EXPECTING_ESCAPED_CHAR = 3

            
def message_formatter():
    global serial_read_buffer_lock
    logger = logging.getLogger("MessageFormatter")
    logger.info("Starting message formatter...")
    
    def parse_serial_read_buffer():
      global serial_read_buffer
      EXPECTED_COMMAND = 0x00
      logger.info("Parsing serial read buffer...")
      unframed_message = bytearray()
      state = KISSPacketParserStates.EXPECTING_INITIAL_FEND
      
      for char_index, char in enumerate(serial_read_buffer):
        match state:
          case KISSPacketParserStates.EXPECTING_INITIAL_FEND:
            if char == KISSChars.FEND.value:
              state = KISSPacketParserStates.EXPECTING_COMMAND
            else:
              logger.error("Unexpected character in EXPECTING_INITIAL_FEND state")
              serial_read_buffer = serial_read_buffer[char_index+1:]
              break
          case KISSPacketParserStates.EXPECTING_COMMAND:
            if char == EXPECTED_COMMAND:
              state = KISSPacketParserStates.EXPECTING_DATA_OR_FINAL_FEND
            else:
              logger.error("Unexpected character in EXPECTING_COMMAND state")
              serial_read_buffer = serial_read_buffer[char_index+1:]
              break
          case KISSPacketParserStates.EXPECTING_DATA_OR_FINAL_FEND:
            match char:
              case KISSChars.FESC.value:
                state = KISSPacketParserStates.EXPECTING_ESCAPED_CHAR
              case KISSChars.FEND.value:
                if len(unframed_message) > 0:
                  logger.debug("Adding message to unframed_messages_from_rx")
                  unframed_messages_from_rx.append(unframed_message)
                else:
                  logger.debug("Message is empty, ignoring")
                serial_read_buffer = serial_read_buffer[char_index+1:]
                break
              case _:
                unframed_message.append(char)
          case KISSPacketParserStates.EXPECTING_ESCAPED_CHAR:
            match char:
              case KISSChars.TFEND.value:
                unframed_message.append(KISSChars.FEND.value)
              case KISSChars.TFESC.value:
                unframed_message.append(KISSChars.FESC.value)
              case _:
                logger.error("Unexpected character in EXPECTING_ESCAPED_CHAR state")
    while True:
        time.sleep(SLEEP_TIME)
        while len(unframed_messages_to_tx) > 0:
            logger.info("Formatting message...")
            formatted_message = format_kiss_message(unframed_messages_to_tx.pop(0))
            logger.info("Formatted message")
            framed_messages_to_tx.append(formatted_message)
            logger.info("Message added to queue")
            
        logger.debug("Taking serial buffer lock")
        with serial_read_buffer_lock:
          logger.debug("Serial buffer lock taken")
          parse_serial_read_buffer()
        logger.debug("Lock released")  
            


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