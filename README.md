# CAN ↔ Modbus RTU Configurable Gateway

## 📌 Descrizione del progetto

Questo repository contiene il codice sorgente di un **gateway configurabile CAN ↔ Modbus RTU**, sviluppato come progetto di tesi triennale in Ingegneria Informatica.

Il sistema consente la traduzione bidirezionale dei messaggi tra una rete **CAN** e una rete **Modbus RTU su RS-485**, utilizzando file di configurazione in formato JSON caricati a runtime da scheda SD.  
L’obiettivo principale è dimostrare un’architettura flessibile e riutilizzabile, separando la logica applicativa dalla configurazione del sistema.

---

## 🧱 Architettura del sistema

Il progetto è composto da due nodi principali:

### 🔹 Gateway CAN ↔ Modbus
- **Scheda:** Arduino UNO R4 Minima  
- **Ruolo:**
  - Master Modbus RTU  
  - Nodo CAN  
- **Funzionalità principali:**
  - Lettura dei file di configurazione JSON da SD
  - Parsing e validazione a runtime
  - Costruzione dinamica dei messaggi CAN
  - Gestione bidirezionale dei flussi CAN → Modbus e Modbus → CAN

### 🔹 Slave Modbus RTU
- **Scheda:** Arduino UNO R3  
- **Ruolo:**
  - Slave Modbus RTU  
- **Funzionalità principali:**
  - Esposizione di holding register
  - Aggiornamento periodico dei registri (simulazione segnali di campo)
  - Logging dei dati ricevuti su file CSV tramite modulo SD

---

## ⚙️ Configurazione dinamica

Il comportamento del gateway è definito interamente tramite file JSON caricati a runtime:

- `can.json`  
  Definisce i parametri del bus CAN e la struttura dei messaggi.

- `modbus.json`  
  Definisce le risorse Modbus (registri, indirizzi, tipi di dato).

- `mapping.json`  
  Definisce le regole di mapping tra segnali CAN e registri Modbus.

La modifica di questi file consente di adattare il sistema a differenti scenari senza ricompilare il firmware.

---

## 🧩 Serializzazione dei dati

La serializzazione e deserializzazione dei dati è gestita tramite funzioni generiche basate su **template C++**, implementate come utility indipendenti dalla configurazione.  
Questo approccio permette di supportare diversi tipi di dato e differenti endianness, mantenendo il codice compatto e riutilizzabile.

---

## 🧪 Testing e validazione

Il sistema è stato testato utilizzando:
- sniffer CAN per il monitoraggio e l’iniezione manuale dei messaggi;
- logging seriale su gateway e slave;
- logging su file CSV su scheda SD;
- indicatori LED per il debug hardware in tempo reale.

I test hanno validato il corretto funzionamento di entrambi i flussi:
- **Modbus → CAN**
- **CAN → Modbus**

---

## 📚 Librerie utilizzate

- **Arduino CAN** – gestione del bus CAN  
- **ArduinoJson** – parsing dei file di configurazione JSON  
- **ModbusMaster** – master Modbus RTU (gateway)  
- **ModbusRTU** – slave Modbus RTU  
- **AltSoftSerial** – comunicazione seriale software RS-485  

---

## 📂 Struttura del repository

