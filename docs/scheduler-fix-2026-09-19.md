# Correzione scheduler — 19 settembre 2026

La precedente ottimizzazione introduceva una regressione: spostando la scadenza alla fine di ogni ciclo in ritardo, si accumulava una deriva e diminuiva la frequenza media. Questa correzione sostituisce quella gestione in FlightCode e FlightCodePI.

## Modifiche

- Scadenze ancorate alla fase originale. A 16 kHz il periodo nominale è 62,5 microsecondi. Un ritardo breve viene recuperato senza spostare permanentemente la griglia temporale.
- In caso di ritardo prolungato vengono saltati e contati gli slot interi già persi, evitando di recuperare una lunga coda di cicli arretrati.
- Pico conserva l'alternanza 62/63 microsecondi e il resto frazionario anche quando salta slot.
- Telemetria periodica con conversione numerica limitata e senza formattazione floating-point printf. Ordine e precisione decimale dei campi conservati; l'arrotondamento di un caso esattamente a metà può differire nell'ultima cifra. PID e mixer non modificati.
- Lettura tempo STM32 con divisione intera a 32 bit. Il prodotto intermedio resta entro il limite per i clock delle schede supportate. Nel disassemblato CLRACINGF4 la funzione board_micros usa una divisione hardware anziché la routine software a 64 bit.
- Il configurator continua a visualizzare la frequenza realmente misurata.

## Verifiche

- Simulazione a 16 kHz con lo stesso carico periodico: precedente scheduler 15483,87 Hz, corretto 16000,00 Hz. È una simulazione del difetto, non una misura della scheda né una riproduzione del valore esatto segnalato.
- Test di ritardi brevi, sovraccarico persistente, slot persi, ritorno a zero del contatore e periodi frazionari superati.
- Test del formato telemetria, 160004 casi numerici, limiti buffer e valori non finiti superati.
- Compilazioni Release completate per MAMBAF411, CLRACINGF4, FLYWOOF405NANO, FLYWOOF405NANO_ANALOG, HDZERO_HALO, Pico 2 e Pico 2 W.
- Controllo diff senza errori di spaziatura. Restano avvisi linker sulle stub libc.

## Limite della verifica

Nessun flash e nessuna misura temporale sull'hardware. La correzione elimina la deriva introdotta dallo scheduler, ma non può garantire ogni ciclo entro 62,5 microsecondi se periferiche o interrupt superano il budget. Verificare frequenza e periodo massimo con configurator collegato, e successivamente con le periferiche utilizzate in volo. I sette firmware compilati e SHA-256 sono nella cartella firmware; baseline, test e log restano accanto a questo rapporto.
