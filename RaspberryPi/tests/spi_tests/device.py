# spi_test/device.py

import spidev
import time
import struct
from .protocol import *

class CANLoggerSPI:
    def __init__(self, bus=0, device=0, speed_hz=16000000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

    def close(self):
        self.spi.close()

    def get_status(self):
        raw = self.spi.xfer([CMD_STATUS, 0x00])
        #marker, count = struct.unpack(HEADER_FORMAT, bytes(raw))
        #if marker == MARKER:
        #    return count
        #return 0
    
    def read_status(self):
        # 1. Invia il comando CMD_STATUS e leggi la risposta
        raw = self.spi.xfer([0x00, 0x00, 0x00, 0x00])

        # 2. Controllo dell'header
        header = raw[:CAN_HEADER_SIZE]
        marker, count, packet_type, _ = struct.unpack(HEADER_FORMAT, bytes(header))

        if marker != MARKER or packet_type != CMD_STATUS:
            #print("❌ Pacchetto non valido o incompleto.")
            #print("Raw data:", raw)
            #print("Header:", header)
            #print("Marker:", marker)
            #print("Packet type:", packet_type)
            return None

        return count
    
    def get_read(self):
        raw = self.spi.xfer([CMD_READ, 0x00])

    def read_messages(self, count):
        # Calcola quanti byte aspettarsi in risposta
        total_bytes = count * CAN_MSG_SIZE + CAN_HEADER_SIZE

        # 1. Invia il comando CMD_READ e leggi la risposta
        dummy = [0x00] * (total_bytes)
        data = self.spi.xfer(dummy)

        # 2. Controllo dell'header
        header = data[:CAN_HEADER_SIZE]
        marker, msg_count, packet_type, _ = struct.unpack(HEADER_FORMAT, bytes(header))

        if marker != MARKER:
            print("❌ Marker errato.")
            return []

        if packet_type == CMD_NOT_READY:
            print("⏳ STM32 non pronto.")
            return []

        if packet_type != CMD_READ or msg_count != count:
            print("❌ Pacchetto non valido.")
            return []

        # 3. Estrai i messaggi CAN
        messages = []

        for i in range(count):
            offset = CAN_HEADER_SIZE + i * CAN_MSG_SIZE
            chunk = data[offset:offset + CAN_MSG_SIZE]
            messages.append(self.parse_can_message(chunk))
        return messages

    def parse_can_message(chunk):
        msg_id, dlc, raw_data, ts, flags = struct.unpack(CAN_MSG_FORMAT, bytes(chunk))
        return {
            "id": msg_id,
            "dlc": dlc,
            "data": list(raw_data[:dlc]),
            "timestamp": ts,
            "flags": flags
        }

    def set_test_mode(self, test_type):
        self.spi.xfer([CMD_TEST_MODE, test_type])

    def read_test_status(self):
        
        raw = self.spi.xfer([0xff, 0xff, 0xff, 0xff])
        marker, count, pkt_type, test_type = struct.unpack(HEADER_FORMAT, bytes(raw))
        return (marker, count, pkt_type, test_type), raw

