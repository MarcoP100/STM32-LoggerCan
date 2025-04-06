# Codice Raspberry Pi

Questa cartella contiene il codice che gira su Raspberry Pi, diviso in:

- `qt/` → GUI scritta in Qt (C++/QML)
- `tests/` → Script di test per SPI, TCP, CAN, ecc.
- `utils/` → Librerie di supporto comuni

## Esecuzione test SPI

```bash
cd tests
python -m spi_tests.test_main