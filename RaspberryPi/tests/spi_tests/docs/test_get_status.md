# Test: GET_STATUS

## 📋 Descrizione
Verifica che lo STM32 risponda correttamente al comando `CMD_STATUS`, restituendo il numero di messaggi disponibili nel buffer CAN.

## ⚙️ Condizioni iniziali
- STM32 alimentato e avviato
- RPi collegato via SPI
- STM32 ha `count = 42` (forzato tramite `#ifdef TEST_MODE`)
- Watchdog abilitato
- Timeout SPI: 1s

## 🧪 Casi testati

### ✅ Caso 1: richiesta e risposta corretta
- **Input**: `[CMD_STATUS, 0xFF]` + clock `[0xFF, 0xFF, 0xFF, 0xFF]`
- **Atteso**: `[0xA5, 42, CMD_STATUS, 0x00]`
- **Risultato**: ✅ OK (valore corretto ricevuto)

### ❌ Caso 2: richiesta ma nessuna risposta
- **Simulazione**: SPI non inizializzato
- **Atteso**: Timeout o errore
- **Risultato**: 🟡 Timeout osservato

### 🔁 Caso 3: watchdog
- **Simulazione**: nessuna richiesta per > 1s
- **Atteso**: STM32 resetta la comunicazione
- **Risultato**: ✅ OK, ritorna a default state

## 🔎 Note
- Richiede `prepareTxStatus` correttamente implementata
- Il marker usato è `0xA5`
- STM32 resetta `spiRxBuffer[0]` dopo ogni richiesta

---

## 📌 Conclusioni
- ✅ Comando stabile
- ⚠️ Alcuni delay sotto 1ms causano comportamenti instabili → da migliorare/monitorare
