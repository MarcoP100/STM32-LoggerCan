# spi_test/protocol.py

import struct

# Comandi SPI
CMD_STATUS = 0x01
CMD_READ   = 0x02
CMD_NOT_READY = 0x03
CMD_TEST_MODE = 0x10

# Comandi di test
TEST_SINGLE_VALUE = 0x01
TEST_RAMP         = 0x02
TEST_OVERFLOW     = 0x03
TEST_FILL_READ_BUFFER = 0x04
TEST_READ_BUFFER = 0x05


# Marker pacchetto valido
MARKER = 0xA5

MAX_MESSAGES = 64

# Formati binari
HEADER_FORMAT = "<BBBB"  # marker, messageCount, packetType,testType 
CAN_MSG_FORMAT = "<IB8sIB"  # id, dlc, data[8], timestamp_us, flags
CAN_MSG_SIZE = struct.calcsize(CAN_MSG_FORMAT)
CAN_HEADER_SIZE = struct.calcsize(HEADER_FORMAT )
