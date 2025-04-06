# 📄 Test: Verifica comando CMD_STATUS (protocollo SPI)

**Data:** 2025-03-29  
**Firmware:** 
**Host/Script:** 

---

## 🎯 Obiettivo

Verificare che STM32 risponda correttamente al comando `CMD_STATUS` via SPI, con `messageCount = 42` (valore forzato).

---

## 🛠️ Setup

- **Hardware coinvolto**:
  - STM32: STM32H723
  - Raspberry Pi: RaspberryPi 5 8Gb, spi0.0

- **Firmware/Script attivo**:
  - STM32: Nel task SPI è stata aggiunta l'impostazione
            `#ifdef SPI_TEST_MODE
                numMsgToSend = 42;
            #endif`
    che imposta a 42 i messaggi da inviare
    
  - RPi: main.py modificato:
        `logger.get_status()
        time.sleep(0.01)
        count = logger.read_status()
        print("count:", count)
        assert count == 42, f"❌ Errore: count atteso 42, ricevuto {count}"
        time.sleep(1)  # ogni 1 ms`

- **Condizioni particolari**:
  - [es: "spi_ready inizialmente = 1", "txBuffer1Ready = true", ecc.]

---

## 🔢 Parametri di test

- SPI speed: 16MHz
- `sleep()` Python tra step: [es: 0.001s]
- Comando testato: [es: `CMD_STATUS = 0x01`]

---

## ✅ Risultati attesi

[Descrivi l’output o il comportamento atteso]

---

## 🔎 Risultati ottenuti

[Output reale o valore letto via print/log]

---

## 💬 Osservazioni

- [Annotazioni su eventuali errori, latenze, glitch]
- [Comportamenti interessanti o imprevisti]
- [Modifiche fatte durante il test, se ci sono]

---

## 🧠 Conclusioni

[Test superato? Cosa conferma? Cosa manca per lo step successivo?]

---

## 📎 Allegati (opzionale)

- [Screenshot, log, schema di collegamento, tracce di logic analyzer]