#pragma once
#include <Arduino.h>
#include <Arduino_CAN.h>
#include <vector>
#include "utils.h"

namespace CANM {

bool begin(long bitrate);
bool sendRaw(uint32_t id, uint8_t dlc, const uint8_t data[8]);

// Trasmissione “per nome” secondo spec + key=value dal terminale
// Esempio cmd: TXN CAN_CMD fan_speed=1200 fan_on=1
bool sendByName(const std::vector<CanMessageSpec>& specs, const String& name, const std::vector<String>& kvPairs);

// Decodifica e stampa un frame ricevuto usando la spec (se c’è match id)
void prettyPrintRx(const std::vector<CanMessageSpec>& specs, const CanMsg& rx);

} // namespace

