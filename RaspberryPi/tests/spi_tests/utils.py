# utils.py
from datetime import datetime
import time
from .protocol import *
import random
import struct


def log_result(test_id, description, result, extra=""):
    time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"{time_str},{test_id},{description},{result},{extra}\n"
    with open("spi_tests/results/status_test_log.csv", "a") as f:
        f.write(line)

def print_test_result(test_id, description, result, extra=""):
    status = "✅" if result == "PASS" else "❌"
    print(f"{status} [{test_id}] {description} ({extra})")


def delay_us(microseconds):
    start = time.perf_counter()
    end = start + microseconds / 1_000_000.0
    while time.perf_counter() < end:
        pass


def check_status_response(cmd_type, arg, delay_us,result):
    """
    Verifica che la risposta dello STM32 sia valida.

    :param result: tupla (marker, count, packet_type, test_type)
    :param expected_count: se specificato, confronta anche il numero di messaggi
    :param expected_type: se specificato, confronta anche il test_type
    :return: True se la risposta è valida, False altrimenti
    """
    #if result is None or len(result) != 4:
    #    return False
    

    marker, count, packet_type, test_type = result

    if delay_us < 0.02 or delay_us > 200:
        if marker != MARKER:
            return True
    elif cmd_type == CMD_STATUS:
        if marker != MARKER or count != 0 or packet_type != CMD_STATUS or test_type != 0xFF:
            return False

    elif cmd_type == CMD_READ:
        if marker != MARKER or count != 0 or packet_type != CMD_NOT_READY or test_type != 0xFF:
            return False

    elif cmd_type == CMD_TEST_MODE:
        if arg == TEST_SINGLE_VALUE:
            if marker != MARKER or count != 42 or packet_type != CMD_TEST_MODE or test_type != TEST_SINGLE_VALUE:
                return False
        elif arg == TEST_RAMP:
            if marker != MARKER or packet_type != CMD_TEST_MODE or test_type != TEST_RAMP:
                return False

        else:
            # Comando di test sconosciuto, fallback a default behavior
            if marker != 0xFF or count != 0xFF:
                return False

    else:
        if marker != 0xFF or count != 0xFF:
            return False
    return True




def generate_test_packet(num_messages, test_id=TEST_READ_BUFFER):
    assert 0 <= num_messages <= MAX_MESSAGES, "Numero messaggi fuori range"

    # Header SPI
    header = struct.pack(HEADER_FORMAT, MARKER, num_messages, CMD_TEST_MODE, test_id)

    # Messaggi CAN casuali
    messages = b''
    for _ in range(num_messages):
        msg_id = random.randint(0x000, 0x7FF)
        dlc = random.randint(0, 8)
        data = bytes([random.randint(0, 255) for _ in range(8)])
        timestamp = random.randint(0, 0xFFFFFFFF)
        flags = random.randint(0, 255)
        messages += struct.pack(CAN_MSG_FORMAT, msg_id, dlc, data, timestamp, flags)

    # CRC finto (0x0000) – opzionale
    crc = struct.pack("<H", 0x0000)

    # Pacchetto completo
    full_packet = header + messages + crc
    return full_packet



def parse_spi_packet(packet_bytes):
    # Controlla che ci siano almeno i byte per l'header
    if len(packet_bytes) < CAN_HEADER_SIZE:
        raise ValueError("Pacchetto troppo corto")

    header = packet_bytes[:CAN_HEADER_SIZE]
    marker, count, packet_type, test_type = struct.unpack(HEADER_FORMAT, header)

    messages = []
    for i in range(count):
        start = CAN_HEADER_SIZE + i * CAN_MSG_SIZE
        end = start + CAN_MSG_SIZE
        if end > len(packet_bytes):
            raise ValueError(f"Messaggio {i+1} incompleto")
        raw_msg = packet_bytes[start:end]
        msg_id, dlc, data, timestamp, flags = struct.unpack(CAN_MSG_FORMAT, raw_msg)
        messages.append({
            "id": msg_id,
            "dlc": dlc,
            "data": list(data[:dlc]),
            "timestamp": timestamp,
            "flags": flags
        })

    # Lettura CRC finale (ultimi 2 byte)
    crc_start = CAN_HEADER_SIZE + count * CAN_MSG_SIZE
    if len(packet_bytes) >= crc_start + 2:
        crc = struct.unpack("<H", packet_bytes[crc_start:crc_start+2])[0]
    else:
        crc = None

    return {
        "header": {
            "marker": marker,
            "count": count,
            "packet_type": packet_type,
            "test_type": test_type,
            "crc": crc
        },
        "messages": messages
    }

def compare_parsed_packets(pkt1, pkt2, verbose=False):
    """
    Confronta due pacchetti già parsati in strutture logiche (dict).
    """
    if pkt1["header"] != pkt2["header"]:
        if verbose:
            print("❌ Header diverso:")
            print("Inviato:", pkt1["header"])
            print("Ricevuto:", pkt2["header"])
        return False

    msgs1 = pkt1["messages"]
    msgs2 = pkt2["messages"]
    if len(msgs1) != len(msgs2):
        if verbose:
            print(f"❌ Numero di messaggi diversi: {len(msgs1)} != {len(msgs2)}")
        return False

    for i, (m1, m2) in enumerate(zip(msgs1, msgs2)):
        if m1 != m2:
            if verbose:
                print(f"❌ Messaggio {i} diverso:")
                print("Inviato:", m1)
                print("Ricevuto:", m2)
            return False

    return True
