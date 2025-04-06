# 📄 Test SPI - FASE 1: Verifica base della comunicazione STM32 <-> Raspberry Pi

## 📅 Data: 2025-03-30  
## 🔧 Firmware STM32: `test_spi_header_ok`  
## 💻 Script Python: `main.py` semplice con `xfer([0x00, 0x00, 0x00, 0x00])`

---

### 🎯 Obiettivo

Verificare che Raspberry Pi e STM32 si scambino 4 byte correttamente via SPI.  
STM32 deve rispondere con un `SPIHeader` preimpostato.

---

### 🛠️ Setup

- STM32H723 con firmware che risponde sempre:
SPIHeader: marker = 0xA5 messageCount = 42 packetType = 0x77 reserved = 0x00

- Python su RPi:
raw = spi.xfer([0x00, 0x00, 0x00, 0x00])
print("SPI header raw:", raw)

---

### Risultati attesi
SPI header raw: [165, 42, 119, 0]

---

### ✅ Risultati ottenuti
SPI header raw: [165, 42, 119, 0]
Tutto corretto. Comunicazione SPI confermata.

---

### 💬 Note e osservazioni
La trasmissione funziona anche ripetuta ogni 1s (sleep(1)).

Il DMA è configurato correttamente.

La ricezione è stabile.

---

### 🧠 Conclusione
Test superato. Possiamo procedere alla fase 2: CMD_STATUS reale.