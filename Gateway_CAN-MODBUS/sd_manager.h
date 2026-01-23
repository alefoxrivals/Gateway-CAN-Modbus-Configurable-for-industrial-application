#pragma once
#include <Arduino.h>
#include <SD.h>

bool SDM_begin(uint8_t csPin);
bool SDM_readText(const char* path, String& out);
