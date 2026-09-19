# Verifica e ottimizzazione firmware — 19 settembre 2026

Analizzati FlightCode (STM32) e FlightCodePI (Pico), partendo dallo stato locale successivo alla rimozione del limite di salita del gas. Le modifiche locali preesistenti sono state conservate.

> Correzione successiva: la gestione dei ritardi descritta qui introduceva deriva della frequenza. È stata sostituita dalle scadenze a fase fissa descritte in [scheduler-fix-2026-09-19.md](scheduler-fix-2026-09-19.md). Le righe scheduler sotto documentano la versione precedente.

## Interventi applicati

| Area | Prima | Ora |
| --- | --- | --- |
| Filtri dei due controllori | Coefficienti ricalcolati nelle elaborazioni dei singoli assi | Coefficienti comuni memorizzati, aggiornati quando cambia dt o una frequenza di taglio |
| Allineamento IMU Pico | Seni, coseni e prodotti di rotazione ripetuti per ogni vettore | Matrice aggiornata solo quando cambiano gli angoli; verificata una volta per campione |
| Feedforward STM32 a guadagno zero | Calcolo della curva progressiva anche con contributo nullo | Ritorno immediato del contributo zero |
| Ritardi scheduler STM32 | Scadenza arretrata, con potenziale raffica di cicli di recupero | Ripartenza dalla fine del ciclo in ritardo |
| Ritardi scheduler Pico | Dopo un ritardo veniva aggiunto un intero periodo di attesa | Ripartenza immediata, poi ritorno alla cadenza nominale |
| Uscita motori Pico | Conversione del campione blackbox prima dell'invio | Invio motori prima della conversione del campione |
| OSD analogico STM32 | Tutti i caratteri cambiati scritti nello stesso aggiornamento | Massimo un carattere e scansione di massimo 16 celle per ciclo, dopo l'uscita motori |

L'OSD richiede circa 484 byte aggiuntivi di RAM. I filtri mantengono le stesse formule; guadagni, frequenze configurate, limiti PID e logica del mixer non sono stati ritoccati.

## Verifiche eseguite

- Compilazioni Release riuscite: MAMBAF411, CLRACINGF4, FLYWOOF405NANO, FLYWOOF405NANO_ANALOG, HDZERO_HALO, Pico 2 e Pico 2 W. Pico usa il backend SPI MPU6500 configurato nel progetto.
- Confronto delle implementazioni precedenti e ottimizzate su PC: 120.000 aggiornamenti per firmware. Comandi motore e termini di controllo coincidenti nei casi provati; massimo scostamento misurato zero.
- Le sequenze comprendono variazioni di dt, saturazioni, cambi delle frequenze dei filtri e del D dinamico. Pico comprende timestamp duplicati, ritorno a zero del contatore e disarmi; STM32 comprende calibrazione, armo e verifica dello stop per perdita ricevitore. Verificato anche il percorso feedforward nullo.
- Allineamento: 20.000 vettori, 2.000 configurazioni angolari, differenza massima zero su PC. Nel test le valutazioni trigonometriche scendono da 120.000 a 12.000; a configurazione fissa avvengono solo al primo aggiornamento.
- Test OSD: scritture limitate, cancellazione dei caratteri, sostituzione della schermata durante il trasferimento e reset.
- Test scheduler: cadenza ordinaria, ciclo in ritardo e wrap del contatore a 32 bit.
- Test già presenti: formato metadati dei due firmware e gas diretto/disarmo/riarmo Pico superati.
- Controllo degli errori di formattazione delle modifiche superato. Restano avvisi di compilazione sulle stub della libreria C e sulle funzioni OSD inutilizzate nei target senza OSD analogico.

I confronti numerici sono test host, non misure temporali sulle schede. Non viene dichiarata una percentuale di velocizzazione né una certificazione di stabilità in volo.

## Punti da misurare sulla scheda per il racing

1. **Tempo massimo, oltre alla media.** Un ciclo a 8 kHz dispone di 125 microsecondi, uno a 16 kHz di 62,5 microsecondi. Servono misure con gyro, ricevitore, motori, OSD e registrazione attivi insieme, idealmente con un pin di temporizzazione e analizzatore logico. Il recupero delle scadenze evita raffiche ma non elimina la causa dei ritardi.
2. **Blackbox e periferiche SPI.** La coda SD è di 32 blocchi; una scheda occupata a lungo può ancora perdere campioni. Il codice usa DMA per la scrittura dei blocchi, ma conserva transazioni HAL sincrone e timeout. Nel log fornito nella conversazione si erano già osservate perdite di campioni: questo intervento non ne prova la risoluzione.
3. **OSD.** La scrittura è distribuita, ma il trasferimento del singolo carattere rimane sincrono; la formattazione della schermata resta periodica. Il limite per ciclo è un limite di lavoro, non un tempo massimo garantito in caso di errore SPI.
4. **Campionamento gyro STM32.** La lettura resta pianificata dal main loop e il PID usa dt nominale. Per ulteriori miglioramenti di precisione temporale va valutata una lettura sincronizzata al data-ready del sensore, con misure sull'hardware e prove dei guasti.
5. **Risoluzione ingressi Pico.** Gli stick sono ancora quantizzati a percentuali intere. Una futura estensione della risoluzione modificherebbe la risposta agli stick e richiede una verifica separata del protocollo e del comportamento di volo.
6. **Taratura.** La stabilità meccanica e la taratura su questo telaio richiedono un nuovo log. I test host non riproducono vibrazioni, aerodinamica, ESC o rumore del sensore.

## File prodotti

La cartella `firmware` contiene i sette binari locali e le relative impronte SHA-256. Non è stato effettuato alcun flash sulla scheda o pubblicazione. Le versioni dei formati salvati sono quelle introdotte con la precedente rimozione del limite gas; le impostazioni del firmware precedente a quella rimozione devono essere riconfigurate.

La cartella `baseline` conserva i sorgenti usati per il confronto. In `host` si trovano programmi di verifica, comandi di compilazione e risultati; i log di compilazione sono nella cartella di questo rapporto.
