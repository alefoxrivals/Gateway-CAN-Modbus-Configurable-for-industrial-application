#include "sd_manager.h"

bool SDM_begin(uint8_t csPin) 
{
  return SD.begin(csPin);
}

bool SDM_readText(const char* path, String& out) 
{
  File f = SD.open(path, FILE_READ);

  if (!f) return false;

  out = f.readString();
  f.close();
  return true;
}
