import threading
import enum
import logging
import time
logger = logging.getLogger(__name__)

RADIODRIVER_SERIAL_PORT = '/dev/tty.debug-console'
LOG_PATH = 'radiodriver-proxy.log'

messages: list[str] = []
kiss_message_queue: list[bytes] = []

class KISSChars(enum.Enum):
    FEND = 0xC0 # Frame End
    FESC = 0xDB # Frame Escape
    TFEND = 0xDC # Transposed Frame End
    TFESC = 0xDD # Transposed Frame Escape


def format_kiss_message(chars: str, tnc_port: int = 0) -> bytes:
    tnc_port_valid = tnc_port >= 0 and tnc_port <= 9
    if not tnc_port_valid:
        raise ValueError("TNC Port must be between 0 and 9")
    
    packet = bytearray()
    packet.append(KISSChars.FEND.value)

    dataframe = tnc_port << 4
    packet.append(dataframe)

    for char in chars:
        if char == KISSChars.FEND.value:
            packet.append(KISSChars.FESC.value)
            packet.append(KISSChars.TFEND.value)
        elif char == KISSChars.FESC.value:
            packet.append(KISSChars.FESC.value)
            packet.append(KISSChars.TFESC.value)
        else:
            packet.append(ord(char))
    
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
    logger = logging.getLogger("MessageReceiver")
    logger.info("Starting message receiver...")
    # TODO: receive messages from ros/basestation
    import json, datetime, random
    while True:
        fake_msg = json.dumps({"random": random.random(), "time": datetime.datetime.now().isoformat()})
        logger.info(f"Received message: {fake_msg}")
        messages.append(fake_msg)
        time.sleep(1)

def main():
    print(f"Logging to {LOG_PATH}.")

    logging.basicConfig(level=logging.INFO, filename=LOG_PATH)
    logger.info("--- STARTING RADIODRIVER PROXY ---")

    message_receiver_thread = threading.Thread(target=mock_message_receiver)
    message_receiver_thread.start()

    serial_thread = threading.Thread(target=serial_manager)
    serial_thread.start()

    message_formatter_thread = threading.Thread(target=message_formatter)
    message_formatter_thread.start()

if __name__ == "__main__":
    main()