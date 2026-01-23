#include "modbus_manager.h"

// Oggetto ModbusMaster della libreria ModbusMaster
static ModbusMaster g_mb;

// Pin usato per controllare RE/DE RS485
static uint8_t g_deRePin = 7;
static bool    g_inited  = false;

// preTransmission / postTransmission
// funzioni di callback richieste da ModbusMaster: 
// si chiamano automaticamente prima e dopo ogni trasmissione per abilitare/disabilitare
// il driver RS485 (DE high -> trasmetto, DE low -> ricezione).
static void preTransmission() 
{ 
  digitalWrite(g_deRePin, HIGH); //abilita trasmissione
}
static void postTransmission()
{ 
  digitalWrite(g_deRePin, LOW); //torna in ricezione
}

namespace MBM {

// begin
// Inizializza il ModbusMaster come master RTU su Serial1 
// Imposta il pin RE/DE.

bool begin(const ModbusRtuConfig& cfg, uint8_t deRePin) 
{
  // Salva e configura il pin DE/RE
  g_deRePin = deRePin;
  pinMode(g_deRePin, OUTPUT);
  digitalWrite(g_deRePin, LOW);

  g_mb.begin(cfg.slave_id, Serial1);
  Serial1.begin(cfg.baud, SERIAL_8N1); // per semplicità: 8N1 (parity/stop custom non supportati su R4 facilmente)
  g_mb.preTransmission(preTransmission);
  g_mb.postTransmission(postTransmission);

  g_inited = true;
  return true;
}

// readResource
// Legge i registri di una risorsa Modbus definita in ModbusResourceSpec.
// - supporta solo ModbusFn::ReadHolding
// - outRegs deve avere spazio >= res.count.
// Restituisce true se la lettura ha successo.

bool readResource(const ModbusResourceSpec& res, uint16_t* outRegs) 
{
  if (!g_inited) return false;
  if (res.fn != ModbusFn::ReadHolding) return false;

  // necessaria la lettura dei registri (address, count)
  uint8_t ec = g_mb.readHoldingRegisters(res.address, res.count);

  // verifica codice di errore
  if (ec != g_mb.ku8MBSuccess) 
  {
    Serial.print(F("[MB] read ERR code=")); Serial.println(ec);
    return false;
  }

  // copia la response nel buffer outRegs
  for (uint16_t i=0;i<res.count;i++) 
  {
    outRegs[i] = g_mb.getResponseBuffer(i);
  }
  return true;
}

// writeResource
// Scrive registri su una risorsa Modbus. Supporta sia WriteSingle che
// WriteMultiple a seconda di res.fn.
//   count = quanti registri sono passati in 'regs' 
//   (per WriteMultiple deve essere >= res.count).
bool writeResource(const ModbusResourceSpec& res, const uint16_t* regs, uint16_t count) 
{
  if (!g_inited) return false;

  // WRITE SINGLE REGISTER
  if (res.fn == ModbusFn::WriteSingle) 
  {
    if (count < 1) return false;

    uint8_t ec = g_mb.writeSingleRegister(res.address, regs[0]);

    if (ec != g_mb.ku8MBSuccess) 
    {
      Serial.print(F("[MB] writeSingle ERR code=")); 
      Serial.println(ec);
      return false;
    }
    return true;

    // WRITE MULTIPLE REGISTERS
  } 
  else 
  {
    if (res.fn == ModbusFn::WriteMultiple) 
    {
      if (count < res.count) return false;

      g_mb.clearTransmitBuffer();

      for (uint16_t i=0;i<res.count;i++)
      {
        g_mb.setTransmitBuffer(i, regs[i]);
      } 

      uint8_t ec = g_mb.writeMultipleRegisters(res.address, res.count);

      if (ec != g_mb.ku8MBSuccess) 
      {
        Serial.print(F("[MB] writeMultiple ERR code=")); 
        Serial.println(ec);
        return false;
      }
      return true;
    }
  }

  // funzione non supportata
  return false;
}

// Restituisce riferimento all'oggetto ModbusMaster se serve accesso diretto
ModbusMaster& client() 
{ 
  return g_mb; 
}

} // namespace MBM
