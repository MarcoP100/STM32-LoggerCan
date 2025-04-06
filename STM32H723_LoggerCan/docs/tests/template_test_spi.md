# 📄 Test: [Titolo del test]

## 🗓️ Data: [YYYY-MM-DD]
## 🔧 Firmware: [nome branch o descrizione versione STM32]
## 💻 Host/Script: [nome script Python, versione Qt, ecc.]

---

## 🎯 Obiettivo

[Descrizione di cosa vuoi verificare o testare in questo step.]

---

## 🛠️ Setup

- **Hardware coinvolto**:
  - STM32: [modello, periferiche usate]
  - Raspberry Pi: [modello, collegamenti SPI, etc.]

- **Firmware/Script attivo**:
  - STM32: [funzione attesa o codice importante attivo]
  - RPi: [funzione invocata, comandi xfer, ecc.]

- **Condizioni particolari**:
  - [es: "spi_ready inizialmente = 1", "txBuffer1Ready = true", ecc.]

---

## 🔢 Parametri di test

- SPI speed: [es: 1 MHz]
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