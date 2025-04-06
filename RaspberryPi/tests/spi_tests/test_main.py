# test_main.py
import spidev
import struct
import time
from .device import CANLoggerSPI
from .utils import *
from .protocol import *
import random
import os

logger = CANLoggerSPI()

def test_single_value():
    print("Test 001 - Risposta valida con count = 42")
    logger.set_test_mode(TEST_SINGLE_VALUE)
    delay_us(15)
    result, raw = logger.read_test_status()
    if result[0] == MARKER and result[1] == 42 and result[2] == CMD_STATUS and result[3] == TEST_SINGLE_VALUE:
        log_result("TC-001", "Header valido", "PASS", f"count={result[1]}")
        print_test_result("TC-001", "Header valido", "PASS", f"count={result[1]}")
    else:
        log_result("TC-001", "Header non valido", "FAIL", f"raw={raw}")
        print_test_result("TC-001", "Header non valido", "FAIL", f"raw={raw}")

def test_ramp_count():
    print("🧪 Test 002 - Verifica count da 0 a 100 e ritorno")
    # Salita
    for expected_count in range(0, 101):
        logger.set_test_mode(TEST_RAMP)
        delay_us(15)
        result, raw = logger.read_test_status()
        if result != (MARKER, expected_count, CMD_STATUS, TEST_RAMP):
            msg = f"❌ Errore salita: atteso count={expected_count}, ricevuto={result[1]}"
            print_test_result("TC-002", msg, "FAIL", f"raw={raw}")
            log_result("TC-002", msg, "FAIL", f"raw={raw}")
            return
        delay_us(100)

    # Discesa
    for expected_count in range(99, -1, -1):
        logger.set_test_mode(TEST_RAMP)
        delay_us(15)
        result, raw = logger.read_test_status()
        if result != (MARKER, expected_count, CMD_STATUS, TEST_RAMP):
            msg = f"❌ Errore discesa: atteso count={expected_count}, ricevuto={result[1]}"
            print_test_result("TC-002", msg, "FAIL", f"raw={raw}")
            log_result("TC-002", msg, "FAIL", f"raw={raw}")
            return
        delay_us(100)

    print_test_result("TC-002", "Ramp salita/discesa corretta", "PASS")
    log_result("TC-002", "Ramp salita/discesa corretta", "PASS")


#TEST WATCHDOG
#WD-001 - Funzionamento corretto
def test_watchdog_no_timeout():
    print("Test WD-001 - Risposta valida con count = 42")
    logger.set_test_mode(TEST_SINGLE_VALUE)
    delay_us(15)
    result, raw = logger.read_test_status()
    if result[0] == MARKER and result[1] == 42 and result[2] == CMD_STATUS and result[3] == TEST_SINGLE_VALUE:
        log_result("WD-001", "Header valido", "PASS", f"count={result[1]}")
        print_test_result("WD-001", "Header valido", "PASS", f"count={result[1]}")
    else:
        log_result("WD-001", "Header non valido", "FAIL", f"raw={raw}")
        print_test_result("WD-001", "Header non valido", "FAIL", f"raw={raw}")

#WD-002 - Errore timeout watchdog
def test_watchdog_timeout():
    print("WD-002 - Timeout watchdog")
    logger.set_test_mode(TEST_SINGLE_VALUE)
    time.sleep(1)  # attesa minima per garantire corretto frame
    result, raw = logger.read_test_status()
    if result[0] != MARKER:
        log_result("WD-002", "Timeout avvenuto", "PASS")
        print_test_result("WD-002", "Timeout avvenuto", "PASS")
    else:
        log_result("WD-002", "Timeout non avvenuto", "FAIL", f"raw={raw}")
        print_test_result("WD-002", "Timeout non avvenuto", "FAIL", f"raw={raw}")
    time.sleep(1)  # attesa minima per garantire corretto frame

#WD-003 - Timeout watchdog
def test_watchdog_timeout_repeated(times=20):
    print("WD-003 - Timeout e recupero ripetuto")
    all_passed = True
    for i in range(times):
        time.sleep(1)
        logger.set_test_mode(TEST_SINGLE_VALUE)
        time.sleep(1)  # Simula inattività prolungata
        logger.set_test_mode(TEST_SINGLE_VALUE)
        delay_us(15)  # attesa minima per garantire corretto frame
        result, raw = logger.read_test_status()

        if not result or result[0] != MARKER or result[1] != 42 or result[2] != CMD_STATUS or result[3] != TEST_SINGLE_VALUE:
            log_result("WD-003", f"Tentativo {i+1}", "FAIL", f"raw={raw}")
            print_test_result("WD-003", f"Tentativo {i+1}", "FAIL", f"raw={raw}")
            all_passed = False
            break
        else:
            print(f"✅ Tentativo {i+1} passato", f"raw={raw}")
        

    if all_passed:
        log_result("WD-003", f"Ripetuto {times} volte", "PASS")
        print_test_result("WD-003", f"Ripetuto {times} volte", "PASS")

#WD-004 - Test di stress
def test_watchdog_stress(iterations=100):
    print("🔧 Test WD-004 - SPI Stress Test Ignorante™")
    error_found = 0
    for i in range(iterations):
        
        # 1. Comando casuale (valido o no)
        cmd_type = random.choice([CMD_STATUS, CMD_READ, CMD_TEST_MODE, 0xAA, 0x55])
        arg = random.randint(0, 255)

        # 2. Invio comando
        logger.spi.xfer([cmd_type, arg])

        # 3. Aspetta un tempo random
        delay = random.choice([0.005, 0.01, 0.015, 0.025, 0.05, 0.1, 0.5, 2.0, 10.0, 100.0, 1000.0])  # millisecondi
        if delay < 10.0:
            delay_us(delay * 1000)
        else:
            time.sleep(delay / 1000)

        print(f"Comando {cmd_type:#04x} arg {arg:#04x} delay {delay} ms")
        # 4. Prova lettura status
        try:
            result, raw = logger.read_test_status()
            if check_status_response(cmd_type, arg, delay, result):
                print_test_result("WD-004", f"Iterazione {i+1} - Comando {cmd_type:#04x} arg {arg:#04x} delay {delay} - Risposta valida", "PASS", f"raw={raw}")
                log_result("WD-004", f"Comando {cmd_type:#04x} arg {arg:#04x} - Risposta valida", "PASS")
            else:
                print_test_result("WD-004", f"Iterazione {i+1}- Comando {cmd_type:#04x} arg {arg:#04x} delay {delay} - Risposta non valida","FAIL", f"raw={raw}")
                log_result("WD-004", f"Errore marker a iterazione {i+1}", "FAIL", f"raw={raw}")
                error_found = error_found + 1
                #break
        except Exception as e:
            print_test_result("WD-004", f"Eccezione a iterazione {i+1}", "FAIL", str(e))
            log_result("WD-004", f"Eccezione a iterazione {i+1}", "FAIL", str(e))
            error_found = error_found + 1
            break
        
        if delay < 0.2 or delay > 200:
            time.sleep(1)
        elif cmd_type == CMD_STATUS or cmd_type == CMD_READ:
            time.sleep(0.01)
        elif cmd_type == CMD_TEST_MODE:
            if arg == TEST_SINGLE_VALUE or arg == TEST_RAMP:
                time.sleep(0.01)
            else:
                time.sleep(1)  # attesa minima per garantire corretto frame
        else:
            time.sleep(1)

    print(f"Test WD-004 completato con {error_found} errori")

#TR-001 - TEST READ BUFFER
def test_read_buffer(num_msg):
    print("Test TR-001 - Test Read Buffer")
     # 1. Genera pacchetto finto con `num_msg`
    sent_bytes = generate_test_packet(num_msg, TEST_READ_BUFFER)
    sent_parsed = parse_spi_packet(sent_bytes)

    # 2. Invio comando di scrittura buffer
    logger.set_test_mode(TEST_FILL_READ_BUFFER)
    delay_us(20)

    # 3. Invia pacchetto binario
    logger.spi.xfer(list(sent_bytes))  # list(...) converte bytes in lista di int

    # 4. Attendi 
    time.sleep(0.01)  # attesa minima per garantire corretto frame

    # 5. Richiedi lettura
    logger.set_test_mode(TEST_READ_BUFFER)
    delay_us(20)
    received_bytes = logger.read_messages(num_msg)
    received_parsed = parse_spi_packet(received_bytes)

    # 6. Confronta con i messaggi originali
    if compare_parsed_packets(sent_parsed, received_parsed, verbose=True):
        print_test_result("TR-001", f"{num_msg} messaggi ricevuti correttamente", "PASS")
        log_result("TR-001", f"{num_msg} messaggi ok", "PASS")
    else:
        print_test_result("TR-001", f"Errore su {num_msg} messaggi", "FAIL")
        log_result("TR-001", f"Errore dati ricevuti", "FAIL")

def run_tests():
    """
    #watchdog tests
    test_watchdog_no_timeout()
    test_watchdog_timeout()
    test_watchdog_no_timeout()
    test_watchdog_timeout_repeated(200)
    test_watchdog_stress(200)

    
    # Test 001 - Risposta valida con count = 42
    for _ in range(200):
        
       
        test_single_value()
        time.sleep(0.001)
        # Test 002 - Risposta valida con count da 0 a 100 e da 100 a 0
    
        test_ramp_count()
        time.sleep(0.001)
    #test_ramp_count()
    """
    


if __name__ == "__main__":
    run_tests()
