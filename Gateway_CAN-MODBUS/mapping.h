#pragma once
#include <Arduino.h>
#include <vector>
#include "utils.h"   // contiene RuleDir, MapPair, MappingRule, FieldSpec...

/**
 * parseMappingJson
 *  - Legge il JSON del mapping (stringa) e costruisce il vettore di regole (MappingRule)
 *  - Risolve i riferimenti a risorse Modbus / messaggi CAN in puntatori (toModbus/fromModbus/toCan/fromCan)
 * 
 * @param json         contenuto mapping.json
 * @param mbResources  elenco risorse Modbus già parsate
 * @param canMessages  elenco messaggi CAN già parsati
 * @param outRules     vettore di regole risultanti
 * @return true se tutto ok
 */
bool parseMappingJson(const String& json, const std::vector<ModbusResourceSpec>& mbResources, const std::vector<CanMessageSpec>& canMessages, std::vector<MappingRule>& outRules);

/**
 * buildCanFromModbus
 * Usa una regola MB2CAN per costruire un frame CAN a partire dai registri Modbus
 */
bool buildCanFromModbus(
  const MappingRule& rule, 
  const uint16_t* regs,     // buffer registri letti per la risorsa rule.fromModbus (size >= rule.fromModbus->count)
  uint16_t regCount,        // quanti registri sono validi in "regs"
  uint32_t& outId, uint8_t& outDlc, uint8_t            outData[8]);

/**
 * extractModbusFromCan
 * Usa una regola CAN2MB per estrarre valori da un frame CAN e riempire registri Modbus
 */
bool extractModbusFromCan(
  const MappingRule& rule, const uint8_t* rxData, uint8_t rxDlc,
  uint16_t*          outRegs,     // buffer output registri (size >= rule.toModbus->count)
  uint16_t           outCount);
