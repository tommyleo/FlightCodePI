# FlightCodePI

Flight controller Quad X rate mode per Raspberry Pi Pico 2 W, compatibile con
il configuratore unico `C:\SvilST\FlightCodeConfigurator`.

## Funzioni

- loop di controllo e acquisizione gyro a 8 kHz;
- MPU6500, MPU9250 o MPU9255 su SPI0;
- ricevente SBUS a 16 canali;
- DSHOT150, DSHOT300 e DSHOT600;
- PID roll, pitch e yaw con anti-windup e D-term filtrato;
- rate individuali, expo, feedforward e TPA;
- mixer Quad X con idle configurabile e direzione yaw normale/invertita;
- allineamento software della flight controller sui tre assi;
- calibrazione gyro automatica e manuale;
- test motori con timeout e blocco tramite canale ARM;
- simulazione PID con uscite motori fisiche sempre soppresse;
- telemetria estesa e diagnostica ricevente;
- log di volo a 200 Hz, 4096 campioni, persistente in flash;
- riavvio USB BOOTSEL dal configuratore.

## Memoria persistente

L'ultimo settore flash contiene tutte le impostazioni. I 25 settori precedenti
sono riservati all'ultimo log di volo. Le aree sono separate e non si
sovrappongono al firmware.

## Sicurezza di armamento

Il firmware richiede:

1. IMU valida e calibrazione completata;
2. segnale SBUS valido;
3. canale ARM passato prima nello stato basso;
4. throttle non superiore al 5% al momento dell'armamento;
5. configuratore scollegato, salvo la simulazione PID protetta.

Durante la simulazione PID i calcoli restano attivi ma i quattro segnali DSHOT
fisici vengono forzati a zero.

I collegamenti sono descritti in [HARDWARE.md](HARDWARE.md).
