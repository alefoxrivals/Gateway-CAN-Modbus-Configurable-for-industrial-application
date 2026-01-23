#include "utils.h"

// ----- util stringhe -----

// trimBoth toglie gli spazi
String trimBoth(const String& s) 
{
  int i=0, j=s.length()-1;
  while (i<=j && isspace((unsigned char)s[i])) 
  {
    i++;
  }
  while (j>=i && isspace((unsigned char)s[j])) 
  {
    j--;
  }
  if (j<i) 
  {
    return String();
  }
  return s.substring(i, j+1);
}

bool parseUIntFlexible(const String& s, uint32_t& v) 
{
  char* end; v = strtoul(s.c_str(), &end, 0);
  return end != s.c_str();
}

bool strToBool(const String& s, bool& out) 
{
  if (s.equalsIgnoreCase("1") || s.equalsIgnoreCase("true") || s.equalsIgnoreCase("on"))  
  { 
    out=true;  
    return true; 
  }
  if (s.equalsIgnoreCase("0") || s.equalsIgnoreCase("false")|| s.equalsIgnoreCase("off")) 
  { 
    out=false; 
    return true; 
  }
  return false;
}

Endian parseEndianStr(const String& s) {
  return s.equalsIgnoreCase("big") ? Endian::Big : Endian::Little;
}

FieldType parseFieldType(const String& s) {
  if (s.equalsIgnoreCase("uint16")) return FieldType::Uint16;
  if (s.equalsIgnoreCase("int16"))  return FieldType::Int16;
  if (s.equalsIgnoreCase("float"))  return FieldType::Float32;
  if (s.equalsIgnoreCase("bool"))   return FieldType::Bool;
  return FieldType::Unknown;
}

CanDir parseDirStr(const String& s) {
  if (s.equalsIgnoreCase("NET2INT")) return CanDir::NET2INT;
  if (s.equalsIgnoreCase("INT2NET")) return CanDir::INT2NET;
  if (s.equalsIgnoreCase("BOTH"))    return CanDir::BOTH;
  return CanDir::INVALID;
}

ModbusFn parseModbusFn(const String& s) {
  if (s.equalsIgnoreCase("read_holding"))   return ModbusFn::ReadHolding;
  if (s.equalsIgnoreCase("write_single"))   return ModbusFn::WriteSingle;
  if (s.equalsIgnoreCase("write_multiple")) return ModbusFn::WriteMultiple;
  return ModbusFn::Unknown;
}

// ============ JSON → CAN ============
bool parseCanJson(const String& json, long& outBitrate, std::vector<CanMessageSpec>& outMsgs)
{
  outMsgs.clear();
  outBitrate = 500000;
  JSONVar root = JSON.parse(json);

  if (JSON.typeof(root) == "undefined") 
  {
    Serial.println(F("[JSON] CAN parse fallito"));
    return false;
  }

  if (root.hasOwnProperty("bitrate")) 
  {
    outBitrate = (long)root["bitrate"];
  }

  if (!root.hasOwnProperty("messages") || JSON.typeof(root["messages"])!="array") 
  {
    Serial.println(F("[JSON] CAN 'messages' mancante o non array"));
    return false;
  }

  JSONVar msgs = root["messages"];

  for (unsigned int i=0; i<msgs.length(); ++i) 
  {
    JSONVar m = msgs[i];
    CanMessageSpec spec;

    if (m.hasOwnProperty("name") && JSON.typeof(m["name"])=="string") 
    {
      spec.name = (const char*)m["name"];
    }

    String idStr;
    if (m.hasOwnProperty("id")) 
    {
      if (JSON.typeof(m["id"])=="string")      
      {
        idStr = (const char*)m["id"];
      }
      else
      {
        if (JSON.typeof(m["id"])=="number") 
        {
          idStr = String((long)m["id"]);
        }
      } 
    }

    uint32_t idv; 

    if (!idStr.length() || !parseUIntFlexible(idStr, idv)) 
    { 
      Serial.println(F("[JSON] id invalido")); 
      continue; 
    }

    spec.id = idv;

    if (!m.hasOwnProperty("dlc")) 
    { 
      Serial.println(F("[JSON] dlc mancante")); 
      continue; 
    }
    
    long dlc = (long)m["dlc"];

    if (dlc<0 || dlc>8) 
    { 
      Serial.println(F("[JSON] dlc fuori range")); 
      continue; 
    }

    spec.dlc = (uint8_t)dlc;

    if (!m.hasOwnProperty("dir") || JSON.typeof(m["dir"])!="string") 
    { 
      Serial.println(F("[JSON] dir mancante")); 
      continue; 
    }

    spec.dir = parseDirStr((const char*)m["dir"]);

    if (spec.dir==CanDir::INVALID) 
    { 
      Serial.println(F("[JSON] dir invalida")); 
      continue;
    }

    if (!m.hasOwnProperty("fields") || JSON.typeof(m["fields"])!="array") 
    { 
      Serial.println(F("[JSON] fields mancante/non array")); 
      continue; 
    }
    JSONVar arr = m["fields"];

    uint16_t maxUsed=0;
    for (unsigned int j=0; j<arr.length(); ++j) 
    {
      JSONVar f = arr[j];
      FieldSpec fs;

      if (f.hasOwnProperty("name") && JSON.typeof(f["name"])=="string") 
      {
        fs.name = (const char*)f["name"];
      }

      fs.type   = parseFieldType(f.hasOwnProperty("type") ? (const char*)f["type"] : "");
      fs.offset = (uint16_t)((long)f["offset"]);
      fs.size   = (uint8_t)((long)f["size"]);
      fs.endian = parseEndianStr(f.hasOwnProperty("endian") ? (const char*)f["endian"] : "little");
      fs.scale  = f.hasOwnProperty("scale") ? (double)f["scale"] : 1.0;

      if (fs.type==FieldType::Unknown || fs.size==0) 
      { 
        Serial.println(F("[JSON] field invalido")); 
        continue; 
      }

      if (fs.offset + fs.size > spec.dlc)            
      { 
        Serial.println(F("[JSON] field fuori DLC")); 
        continue; 
      }

      spec.fields.push_back(fs);

      if (fs.offset + fs.size > maxUsed) 
      {
        maxUsed = fs.offset + fs.size;
      }
    }
    outMsgs.push_back(spec);
  }
  return !outMsgs.empty();
}

// ============ JSON → Modbus ============
bool parseModbusJson(const String& json, ModbusRtuConfig& outRTU, std::vector<ModbusResourceSpec>& outRes)
{
  outRes.clear();
  outRTU = ModbusRtuConfig();
  JSONVar root = JSON.parse(json);

  if (JSON.typeof(root) == "undefined") 
  {
    Serial.println(F("[JSON] Modbus parse fallito"));
    return false;
  }

  if (root.hasOwnProperty("rtu")) 
  {
    JSONVar rtu = root["rtu"];

    if (rtu.hasOwnProperty("baud"))
    {
      outRTU.baud = (long)rtu["baud"];
    }      
    if (rtu.hasOwnProperty("parity"))    
    {
      outRTU.parity = ((const char*)rtu["parity"])[0];
    }
    if (rtu.hasOwnProperty("stop_bits")) 
    {
      outRTU.stop_bits = (uint8_t)((long)rtu["stop_bits"]);
    }
    if (rtu.hasOwnProperty("slave_id"))  
    {
      outRTU.slave_id = (uint8_t)((long)rtu["slave_id"]);
    }
  }

  if (!root.hasOwnProperty("resources") || JSON.typeof(root["resources"])!="array") 
  {
    Serial.println(F("[JSON] Modbus 'resources' mancante o non array"));
    return false;
  }

  JSONVar arr = root["resources"];

  for (unsigned int i = 0; i < arr.length(); ++i) 
  {
    JSONVar r = arr[i];
    ModbusResourceSpec res;

    if (r.hasOwnProperty("name") && JSON.typeof(r["name"])=="string")
    {
      res.name = (const char*)r["name"];
    } 

    res.fn        = parseModbusFn(r.hasOwnProperty("fn") ? (const char*)r["fn"] : "");
    res.address   = (uint16_t)((long)r["address"]);
    res.count     = (uint16_t)((long)r["count"]);
    res.period_ms = r.hasOwnProperty("period_ms") ? (uint32_t)((long)r["period_ms"]) : 0;

    if (!r.hasOwnProperty("fields") || JSON.typeof(r["fields"])!="array") 
    { 
      Serial.println(F("[JSON] Modbus fields mancanti")); 
      continue; 
    }

    JSONVar farr = r["fields"];

    for (unsigned int j = 0; j < farr.length(); ++j) 
    {
      JSONVar f = farr[j];
      ModbusField mf;

      if (f.hasOwnProperty("name") && JSON.typeof(f["name"])=="string") 
      {
        mf.name = (const char*)f["name"];
      }

      mf.type  = parseFieldType(f.hasOwnProperty("type") ? (const char*)f["type"] : "");
      mf.index = (uint8_t)((long)f["index"]);
      mf.scale = f.hasOwnProperty("scale") ? (double)f["scale"] : 1.0;
      res.fields.push_back(mf);
    }
    outRes.push_back(res);
  }
  return !outRes.empty();
}

// ====== istanze template ======
template<typename T>
void writeValue(uint8_t* buf, T v, Endian e, uint8_t size) {
  if (size == 1) 
  { 
    buf[0] = (uint8_t)v; return; 
  }
  union { T v; uint8_t b[sizeof(T)]; } u; u.v = v;
  if (size == 2) 
  { if (e==Endian::Little) 
    {
      buf[0]=u.b[0]; buf[1]=u.b[1]; } 
    else
    { 
      buf[0]=u.b[1]; buf[1]=u.b[0]; 
    } 
    return;
  }
  if (size == 4) 
  { 
    if (e==Endian::Little) 
    { 
      buf[0]=u.b[0]; 
      buf[1]=u.b[1]; 
      buf[2]=u.b[2]; 
      buf[3]=u.b[3]; 
    } 
    else 
    { 
      buf[0]=u.b[3]; 
      buf[1]=u.b[2]; 
      buf[2]=u.b[1]; 
      buf[3]=u.b[0]; 
    } 
    return; 
  }
}

template<typename T>
T readValue(const uint8_t* buf, Endian e, uint8_t size) 
{
  if (size == 1) return (T)buf[0];
  union { T v; uint8_t b[sizeof(T)]; } u;
  if (size == 2) 
  { 
    if (e==Endian::Little) 
    { 
      u.b[0]=buf[0]; 
      u.b[1]=buf[1]; 
    } 
    else 
    { 
      u.b[0]=buf[1]; 
      u.b[1]=buf[0]; 
    } 
  return u.v; 
  }
  if (size == 4) 
  { 
    if (e==Endian::Little) 
    { 
      u.b[0]=buf[0]; 
      u.b[1]=buf[1]; 
      u.b[2]=buf[2];
      u.b[3]=buf[3]; 
    } 
    else 
    { 
      u.b[0]=buf[3]; 
      u.b[1]=buf[2]; 
      u.b[2]=buf[1]; 
      u.b[3]=buf[0]; 
    } 
    return u.v; 
  }
  return (T)0;
}

template void writeValue<uint16_t>(uint8_t*, uint16_t, Endian, uint8_t);
template void writeValue<int16_t>(uint8_t*, int16_t, Endian, uint8_t);
template void writeValue<float>(uint8_t*, float, Endian, uint8_t);
template void writeValue<uint8_t>(uint8_t*, uint8_t, Endian, uint8_t);

template uint16_t readValue<uint16_t>(const uint8_t*, Endian, uint8_t);
template int16_t  readValue<int16_t>(const uint8_t*, Endian, uint8_t);
template float    readValue<float>(const uint8_t*, Endian, uint8_t);
template uint8_t  readValue<uint8_t>(const uint8_t*, Endian, uint8_t);

// find helpers
const CanMessageSpec* findCanByName(const std::vector<CanMessageSpec>& v, const String& name) { for (auto& m : v) if (m.name == name) return &m; return nullptr; }

const FieldSpec* findFieldByName(const std::vector<FieldSpec>& v, const String& name)         { for (auto& f : v) if (f.name == name) return &f; return nullptr; }

const ModbusResourceSpec* findMbResByName(const std::vector<ModbusResourceSpec>& v, const String& name) { for (auto& r : v) if (r.name == name) return &r; return nullptr; }

const ModbusField*        findMbFieldByName(const std::vector<ModbusField>& v, const String& name)       { for (auto& f : v) if (f.name == name) return &f; return nullptr; }

