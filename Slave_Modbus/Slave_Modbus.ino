// ===== UNO R3 — Modbus RTU SLAVE via MAX485 + LOG su SD =====
//
// Collegamenti principali:
//  - RS485 (MAX485):
//      RO -> D8   (AltSoftSerial RX)
//      DI -> D9   (AltSoftSerial TX)
//      RE -> D7   (ponticellato con DE)
//      DE -> D7
//
//  - LED:
//      LED_RX -> D4  (lampeggia quando il MASTER scrive i registri FAN_CMD)
//      LED_TX -> D5  (lampeggia quando lo SLAVE aggiorna i registri ENV)
//
//  - SD card:
//      CS   -> D10
//      MOSI -> D11
//      MISO -> D12
//      SCK  -> D13
//
// Funzionamento:
//  - Espone via Modbus RTU due blocchi di holding registers:
//       MB_ENV    (addr base 0,  count 3): temperatura float32 (2 reg) + umidità (1 reg)
//       MB_FAN_CMD(addr base 20, count 2): fan_speed (u16) + fan_on (bool in u16, uso 0/1)
//
//  - Ogni ENV_PERIOD_MS aggiorna i registri ENV con valori random.
//  - Quando il MASTER (gateway) scrive FAN_CMD con valori DIVERSI dai precedenti, lo slave:
//       lampeggia LED_RX
//       stampa
//       logga su SD in log.csv: timestamp_ms,fan_speed,fan_on
//
//  - Parametri Modbus devono combaciare con modbus.json del gateway:
//      baud    = 9600
//      slaveId = 1
// ========================================================

#include <AltSoftSerial.h>
#include <ModbusRTU.h>
#include <SD.h>

// ---------- Pin ----------
constexpr uint8_t PIN_RE_DE = 7;   // RE+DE del MAX485
constexpr uint8_t LED_RX    = 4;   // LED per scritture del master (FAN_CMD)
constexpr uint8_t LED_TX    = 5;   // LED per update ambiente (ENV)
constexpr uint8_t PIN_SD_CS = 10;  // CS del modulo SD

// ---------- Parametri RTU ----------
constexpr uint32_t MB_BAUD    = 9600;
constexpr uint8_t  MB_SLAVEID = 1;     // deve combaciare con JSON del gateway

// ---------- Indirizzi registri (coerenti con JSON gateway) ----------
// MB_ENV:
//   address = 0, count = 3
//     0...1 -> temperatura float32 (due registri, little-endian)
//     2   -> umidità (u16)
// MB_FAN_CMD:
//   address = 20, count = 2
//     20 -> fan_speed (u16)
//     21 -> fan_on (bool, usiamo 0/1 in u16)
constexpr uint16_t ADDR_ENV_BASE = 0;
constexpr uint16_t ADDR_FAN_BASE = 20;

// ---------- Periodo aggiornamento "ambiente" ----------
constexpr uint32_t ENV_PERIOD_MS = 2000;

// ---------- Oggetti globali ----------
AltSoftSerial  ASerial;   // usa fissi: RX=8, TX=9 su UNO
ModbusRTU      mb;
File           logFile;   // per il file CSV su SD

// Buffer locale per rilevare cambiamenti dei registri ventola
uint16_t prevFanSpeed = 0;
uint16_t prevFanOn    = 0;

// ------------------------ Helper LED ------------------------
static void blink(uint8_t pin, uint16_t ms = 50)
{
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

// ------------------ Float32 <-> due registri ----------------
// Converte un float32 in due registri Modbus (16 bit ciascuno) in formato
// "little endian word order": lo = parte bassa, hi = parte alta.
// Il gateway ricostruirà con: u32 = (hi << 16) | lo;
static void floatToRegsLE(float f, uint16_t& lo, uint16_t& hi)
{
  union { float f; uint8_t b[4]; } u;
  u.f = f;
  // little endian: b[0] LSB, b[3] MSB
  lo = (uint16_t)( u.b[0] | (uint16_t(u.b[1]) << 8) );
  hi = (uint16_t)( u.b[2] | (uint16_t(u.b[3]) << 8) );
}

// --------------------- Dati fittizi ambiente ----------------
// Simulano temperatura e umidità realistiche
static float fakeTemperature()
{
  // 20.0 .. 30.0 °C con un po' di jitter
  return 20.0f + (millis() % 1000) / 100.0f + random(-5, 6) * 0.1f;
}

static uint16_t fakeHumidity()
{
  // circa 40 .. 70 %RH
  return 40 + (millis() / 1000) % 31;
}

// ========================= SETUP ============================
void setup() {
  // Pin RS485 e LED
  pinMode(PIN_RE_DE, OUTPUT);
  digitalWrite(PIN_RE_DE, LOW); // inizialmente in RX (ricezione)

  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);
  digitalWrite(LED_RX, LOW);
  digitalWrite(LED_TX, LOW);

  // Serial monitor (solo debug locale)
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n[SLAVE] UNO R3 Modbus RTU via MAX485 (AltSoftSerial 8/9)"));

  // ===== Inizializzazione SD =====
  if (!SD.begin(PIN_SD_CS))
  {
    Serial.println(F("[SD] Errore inizializzazione SD!"));
  }
  else
  {
    Serial.println(F("[SD] SD inizializzata OK"));

    // Se il file non esiste, creiamo e scriviamo l'intestazione CSV
    if (!SD.exists("log.csv"))
    {
      logFile = SD.open("log.csv", FILE_WRITE);

      if (logFile)
      {
        logFile.println("timestamp_ms,fan_speed,fan_on");
        logFile.close();
        Serial.println(F("[SD] Creato log.csv con header"));
      }
      else
      {
        Serial.println(F("[SD] Impossibile creare log.csv"));
      }
    }
    else
    {
      Serial.println(F("[SD] log.csv esistente, append attivo"));
    }
  }

  // ===== Serial RS485 su AltSoftSerial =====
  // AltSoftSerial lavora sempre 8N1; ModbusRTU libreria si adegua
  ASerial.begin(MB_BAUD);

  // Collega ModbusRTU allo stream e al pin RE/DE
  mb.begin(&ASerial, PIN_RE_DE);
  mb.slave(MB_SLAVEID);

  // ===== Definizione holding registers esposti dallo slave =====
  // ENV: 3 registri 0..2
  mb.addHreg(ADDR_ENV_BASE, 0, 3);
  // FAN_CMD: 2 registri 20..21
  mb.addHreg(ADDR_FAN_BASE, 0, 2);

  // Inizializza ENV con valori di partenza
  uint16_t lo, hi;
  floatToRegsLE(22.5f, lo, hi);          // 22.5°C
  mb.Hreg(ADDR_ENV_BASE + 0, lo);        // parte bassa float
  mb.Hreg(ADDR_ENV_BASE + 1, hi);        // parte alta float
  mb.Hreg(ADDR_ENV_BASE + 2, 50);        // humidity 50%

  // Inizializza FAN_CMD (comandi ventola)
  mb.Hreg(ADDR_FAN_BASE + 0, 0);         // fan_speed = 0
  mb.Hreg(ADDR_FAN_BASE + 1, 0);         // fan_on = 0 (false)

  prevFanSpeed = 0;
  prevFanOn    = 0;

  // Seed random per simulazione
  randomSeed(analogRead(A0));

  Serial.println(F("[SLAVE] pronto. Indirizzo=1, 9600 8N1"));
}

// ========================== LOOP ============================
void loop() {
  // Servizio Modbus:
  //  - ascolta richieste del MASTER (gateway)
  //  - risponde automaticamente a READ/WRITE
  mb.task();

  // --------- Rilevazione scritture comandi ventola ----------
  // Se il gateway scrive i registri 20 (fan_speed) o 21 (fan_on),
  // ModbusRTU aggiorna internamente Hreg(...).
  //
  // NOTA IMPORTANTE:
  //   Rileviamo SOLO quando il valore CAMBIA rispetto al precedente.
  //   Se il master riscrive lo stesso valore (es. 1000,1 -> 1000,1) non c'è
  //   modo di distinguere questa write da "nessuna write"
  uint16_t curFanSpeed = mb.Hreg(ADDR_FAN_BASE + 0);
  uint16_t curFanOn    = mb.Hreg(ADDR_FAN_BASE + 1) & 0x0001;  // forziamo a bit0

  if (curFanSpeed != prevFanSpeed || curFanOn != prevFanOn)
  {
    // Il MASTER ha cambiato almeno uno dei due registri -> lampeggia LED_RX
    blink(LED_RX);

    // Aggiorna i valori precedenti
    prevFanSpeed = curFanSpeed;
    prevFanOn    = curFanOn;

    // Log
    Serial.print(F("[SLAVE] FAN_CMD updated by master -> speed="));
    Serial.print(curFanSpeed);
    Serial.print(F(" on="));
    Serial.println(curFanOn ? F("true") : F("false"));

    // ===== Scrittura su SD: log.csv =====
    // Apre il file e scrive una riga con:
    // timestamp_ms,fan_speed,fan_on
    logFile = SD.open("log.csv", FILE_WRITE);
    if (logFile)
    {
      logFile.print(millis());
      logFile.print(",");
      logFile.print(curFanSpeed);
      logFile.print(",");
      logFile.println(curFanOn);    // 0 o 1
      logFile.close();
      Serial.println(F("[SD] Riga log scritta in log.csv"));
    }
    else
    {
      Serial.println(F("[SD] Errore apertura log.csv"));
    }
  }

  // --------- Aggiornamento periodico ENV (simulato) ----------
  static uint32_t lastEnv = 0;
  uint32_t now = millis();

  if (now - lastEnv >= ENV_PERIOD_MS)
  {
    lastEnv = now;

    float    t = fakeTemperature();
    uint16_t h = fakeHumidity();

    uint16_t lo, hi;
    floatToRegsLE(t, lo, hi);

    mb.Hreg(ADDR_ENV_BASE + 0, lo);
    mb.Hreg(ADDR_ENV_BASE + 1, hi);
    mb.Hreg(ADDR_ENV_BASE + 2, h);

    // Indica aggiornamento locale con LED_TX
    blink(LED_TX);

    // Log su Serial dell'ambiente
    Serial.print(F("[SLAVE] ENV update -> T="));
    Serial.print(t, 2);
    Serial.print(F(" °C  H="));
    Serial.print(h);
    Serial.println(F(" %"));
  }
}
