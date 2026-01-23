/*
  Progetto: Gateway Modbus RTU <-> CAN bus su Arduino UNO R4 Minima

  Descrizione generale
  --------------------
  Questo progetto implementa un gateway embedded in grado di tradurre in tempo
  reale dati tra un dispositivo Modbus RTU (slave) e una rete CAN bus.

  L’unità centrale è un Arduino UNO R4 Minima che svolge il ruolo di:
    - Master Modbus RTU su interfaccia RS485
    - Nodo CAN per trasmissione e ricezione frame
    - Mappatura tra registri Modbus e campi dei messaggi CAN

  La configurazione del sistema è completamente esterna e caricata da tre file
  JSON:
    - modbus.json   -> definisce risorse, indirizzi e modalità di polling Modbus
    - can.json      -> definisce messaggi CAN, ID, DLC e struttura dei campi
    - mapping.json  -> definisce le regole di conversione tra Modbus <-> CAN

  Obiettivo
  ---------
  Fornire un collegamento trasparente tra un dispositivo/rete Modbus e una 
  rete CAN, senza che i due dispositivi conoscano reciprocamente
  i protocolli dell’altro.
  -----------------------------
*/

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "utils.h"
#include "sd_manager.h"
#include "can_manager.h"
#include "modbus_manager.h"
#include "mapping.h"

constexpr uint8_t LED_CAN_TX = 8; // LED TX CAN

// ===== SD paths =====
constexpr uint8_t PIN_SD_CS   = 10;
constexpr char    CAN_PATH[]  = "/CAN~1.JSO";
constexpr char    MB_PATH[]   = "/MODBUS~1.JSO";
constexpr char    MAP_PATH[]  = "/MAPPIN~1.JSO";

// ===== runtime config =====
long g_canBitrate = 500000;                   //CAN bitrate
std::vector<CanMessageSpec>     g_canMsgs;    //messaggi CAN da can.json
ModbusRtuConfig                 g_rtu;        //configurazione RTU
std::vector<ModbusResourceSpec> g_mbRes;      //risosre Modbus da modbus.json
std::vector<MappingRule>        g_rules;      //regole mapping CAN <-> Modbus

// Tengo traccia di quando è stata letta l'ultima volta ciascuna risorsa Modbus
struct PollState {
  const ModbusResourceSpec* res;
  uint32_t last_ms = 0;
};

std::vector<PollState> g_pollers;

// Crea una lista di risorse da leggere periodicamente in base alle regole MB2CAN
static void buildPollers() {
  for (auto& r : g_rules) 
  {
    if (r.dir != RuleDir::MB2CAN || !r.fromModbus) 
    {
      continue;
    }
    bool already=false;
    for (auto& p : g_pollers) 
    {
      if (p.res == r.fromModbus) 
      { 
        already=true; 
        break; 
      }
    }

    if (!already) 
    {
      g_pollers.push_back({ r.fromModbus, 0 });
    }
  }
}

// ACCENSIONE LED CAN TX
static void blinkCanLed(uint16_t ms = 50)
{
  digitalWrite(LED_CAN_TX, HIGH);
  delay(ms);
  digitalWrite(LED_CAN_TX, LOW);
}

void setup() 
{
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n=== Gateway definitivo: CAN <-> Modbus via JSON ==="));

  // PIN LED CAN TX
  pinMode(LED_CAN_TX, OUTPUT);
  digitalWrite(LED_CAN_TX, LOW);

  // SD
  if (!SDM_begin(PIN_SD_CS)) 
  { 
    Serial.println(F("[SD] init FAIL")); 
    while(true){} 
  }

  // Caricamento CAN
  String canJson;
  if (!SDM_readText(CAN_PATH, canJson)) 
  { 
    Serial.println(F("[SD] can.json missing")); 
    while(true){} 
  }
  if (!parseCanJson(canJson, g_canBitrate, g_canMsgs)) 
  { 
    Serial.println(F("[JSON] can FAIL")); 
    while(true){} 
  }
  Serial.print(F("[CFG] CAN bitrate=")); 
  Serial.println(g_canBitrate);

  // Caricamento Modbus
  String mbJson;
  if (!SDM_readText(MB_PATH, mbJson)) 
  { 
    Serial.println(F("[SD] modbus.json missing")); 
    while(true){} 
  }

  if (!parseModbusJson(mbJson, g_rtu, g_mbRes)) 
  { 
    Serial.println(F("[JSON] modbus FAIL")); 
    while(true){} 
  }

  Serial.print(F("[CFG] MB RTU baud=")); 
  Serial.print(g_rtu.baud);
  Serial.print(F(" slave=")); 
  Serial.println(g_rtu.slave_id);

  // Load Mapping
  String mapJson;
  if (!SDM_readText(MAP_PATH, mapJson)) 
  { 
    Serial.println(F("[SD] mapping.json missing"));
    while(true){} 
  }
  if (!parseMappingJson(mapJson, g_mbRes, g_canMsgs, g_rules)) 
  { 
    Serial.println(F("[JSON] mapping FAIL")); 
    while(true){} 
  }
  Serial.print(F("[CFG] rules=")); 
  Serial.println((int)g_rules.size());

  // Inizializzazione CAN
  if (!CANM::begin(g_canBitrate)) 
  { 
    Serial.println(F("[CAN] init FAIL")); 
    while(true){} 
  }
  Serial.println(F("[CAN] init OK"));

  // Inizializzazione ModbusMaster (DE/RE su D7)
  if (!MBM::begin(g_rtu, 7)) 
  { 
    Serial.println(F("[MB] init FAIL")); 
    while(true){} 
  }
  Serial.println(F("[MB] init OK"));

  // Prepara pollers
  buildPollers();
}

// buffer temporanei per registri
static uint16_t regsBuf[16]; // sufficiente per i gli esempi (si aumenta se serve)

void loop() 
{
  // ========= RX CAN -> Modbus (CAN2MB) =========
  if (CAN.available()) 
  {
    CanMsg rx = CAN.read();
    CANM::prettyPrintRx(g_canMsgs, rx);

    for (auto& rule : g_rules) 
    {
      if (rule.dir != RuleDir::CAN2MB || !rule.fromCan || !rule.toModbus) 
      {
        continue;
      }
      if (rule.fromCan->id != rx.id)
      {
        continue;
      }

      //numero di registri che vogliamo riempire (es. MP_FAN_CMD-cout = 2)
      uint16_t outCount = rule.toModbus->count;

      if (extractModbusFromCan(rule, rx.data, rx.data_length, regsBuf, outCount)) 
      {
        if (!MBM::writeResource(*rule.toModbus, regsBuf, outCount)) 
        {
          Serial.println(F("[CAN->MB] writeResource FAIL"));
        } 
        else 
        {
          Serial.print(F("[CAN->MB] write OK to ")); 
          Serial.print(rule.toModbus->name);
          Serial.print(F(" addr = ")); 
          Serial.println(rule.toModbus->address);
        }
      }
    }
  }

  // ========= Poll Modbus -> CAN (MB2CAN) =========
  uint32_t now = millis();
  for (auto& p : g_pollers) 
  {
    const ModbusResourceSpec* res = p.res;
    if (!res || res->period_ms == 0) 
    {
      continue;
    }
    if (now - p.last_ms < res->period_ms) 
    {
      continue; // non ancora tempo
    }
    p.last_ms = now;

    // Leggi registri per la risorsa
    if (!MBM::readResource(*res, regsBuf)) 
    {
      Serial.print(F("[MB poll] read FAIL for ")); 
      Serial.println(res->name);
      continue;
    }

    // per ogni regola MB2CAN che usa questa risorsa costruisce e invia il frame
    for (auto& rule : g_rules) 
    {
      if (rule.dir != RuleDir::MB2CAN || rule.fromModbus != res || !rule.toCan) 
      {
        continue;
      }

      uint32_t id; 
      uint8_t dlc; 
      uint8_t data[8];

      if (buildCanFromModbus(rule, regsBuf, res->count, id, dlc, data)) 
      {
        if (!CANM::sendRaw(id, dlc, data)) 
        {
          Serial.println(F("[MB->CAN] sendRaw FAIL (bus busy/no ACK)"));
        } 
        else 
        {
          Serial.print(F("[MB->CAN] TX ")); 
          Serial.print(rule.toCan->name);
          Serial.print(F(" id=0x")); 
          Serial.print(id, HEX);
          Serial.print(F(" dlc="));
          Serial.println(dlc);
          blinkCanLed(); // lampeggio led CAN TX
        }
      }
    }
  }
}
