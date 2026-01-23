#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>
#include "utils.h"

// uso ModbusMaster (Doc Walker) su Serial1 + MAX485 (DE/RE su un pin D7)

namespace MBM {
  bool begin(const ModbusRtuConfig& cfg, uint8_t deRePin);

  // Lettura holding registers (ReadHolding)
  bool readResource(const ModbusResourceSpec& res, uint16_t* outRegs /*len>=res.count*/);

  // Scrittura holding registers (WriteSingle / WriteMultiple)
  bool writeResource(const ModbusResourceSpec& res, const uint16_t* regs, uint16_t count);

  // Accesso a ModbusMaster (per eventuali debug)
  ModbusMaster& client();
}
