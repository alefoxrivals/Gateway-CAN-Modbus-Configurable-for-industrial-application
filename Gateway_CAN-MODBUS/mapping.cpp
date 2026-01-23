#include "mapping.h"

/*
  parseMappingJson
  - legge il JSON del mapping
  - verifica struttura e campi per ciascuna regola
  - risolve i nomi (stringhe) in puntatori verso le strutture Modbus/CAN già parse
  - costruisce la lista di coppie (src,dst) per ogni regola
*/
bool parseMappingJson(const String& json, const std::vector<ModbusResourceSpec>& mbRes, const std::vector<CanMessageSpec>& canMsgs, std::vector<MappingRule>& outRules)
{
  outRules.clear();

  JSONVar root = JSON.parse(json);

  if (JSON.typeof(root) == "undefined") 
  {
    Serial.println(F("[MAP] JSON parse fallito"));
    return false;
  }

  // La radice deve contenere un array "rules"
  if (!root.hasOwnProperty("rules") || JSON.typeof(root["rules"]) != "array") 
  {
    Serial.println(F("[MAP] Campo 'rules' mancante o non array"));
    return false;
  }

  JSONVar rules = root["rules"];

  // Itera tutte le regole trovate nel file
  for (unsigned int i = 0; i < rules.length(); ++i) 
  {
    JSONVar r = rules[i];
    MappingRule rule;
    rule.pairs.clear();

    // dir
    if (!r.hasOwnProperty("dir") || JSON.typeof(r["dir"]) != "string") 
    {
      Serial.println(F("[MAP] dir mancante"));
      return false;
    }

    String dirStr = (const char*)r["dir"];

    if (dirStr.equalsIgnoreCase("MB2CAN")) 
    {
      rule.dir = RuleDir::MB2CAN;
    }
    else if (dirStr.equalsIgnoreCase("CAN2MB")) 
    {
      rule.dir = RuleDir::CAN2MB;
    }
    else 
    { 
      Serial.println(F("[MAP] dir invalida")); 
      return false; 
    }

    // Caso MB -> CAN
    if (rule.dir == RuleDir::MB2CAN) 
    {
      // Controlla presenza from_modbus e to_can
      if (!r.hasOwnProperty("from_modbus") || !r.hasOwnProperty("to_can")) 
      {
        Serial.println(F("[MAP] campi MB2CAN mancanti"));
        return false;
      }

      JSONVar fm = r["from_modbus"];
      JSONVar tc = r["to_can"];

      // I campi interni devono essere stringhe (resource / message)
      if (!fm.hasOwnProperty("resource") || JSON.typeof(fm["resource"]) != "string" || !tc.hasOwnProperty("message")  || JSON.typeof(tc["message"])  != "string") 
      {
        Serial.println(F("[MAP] from_modbus.resource / to_can.message mancanti"));
        return false;
      }

      // Salva i nomi e risolvi i puntatori alle strutture già parsate
      rule.from = (const char*)fm["resource"];
      rule.to   = (const char*)tc["message"];

      rule.fromModbus = findMbResByName(mbRes, rule.from);    // puntatore verso ModbusResourceSpec
      rule.toCan      = findCanByName(canMsgs, rule.to);      // puntatore verso CanMessageSpec

      if (!rule.fromModbus) 
      { 
        Serial.println(F("[MAP] resource Modbus non trovata")); 
        return false; 
      }
      if (!rule.toCan)      
      { 
        Serial.println(F("[MAP] message CAN non trovato"));    
        return false; 
      }

      // map array
      if (!r.hasOwnProperty("map") || JSON.typeof(r["map"]) != "array") 
      {
        Serial.println(F("[MAP] array 'map' mancante"));
        return false;
      }

      JSONVar mp = r["map"];

      // valido ogni coppia mappata e la aggiungo alla regola
      for (unsigned int k=0; k<mp.length(); ++k) 
      {
        JSONVar m = mp[k];
        if (JSON.typeof(m) != "object" || !m.hasOwnProperty("src") || !m.hasOwnProperty("dst"))
        {
          continue;
        }
          
        String src = (const char*)m["src"];
        String dst = (const char*)m["dst"];

        // Verifica che il field esista nella risorsa Modbus (src) e nel frame CAN (dst)
        const ModbusField* srcF = findMbFieldByName(rule.fromModbus->fields, src);
        const FieldSpec*   dstF = findFieldByName(rule.toCan->fields, dst);

        if (!srcF || !dstF) 
        {
          Serial.println(F("[MAP] campo src/dst non trovato in MB2CAN"));
          return false;
        }
        rule.pairs.push_back({src, dst});
      }
    } 
    else // Caso CAN -> MB
    { 
      // Controlla presenza from_can e to_modbus
      if (!r.hasOwnProperty("from_can") || !r.hasOwnProperty("to_modbus")) 
      {
        Serial.println(F("[MAP] campi CAN2MB mancanti"));
        return false;
      }

      JSONVar fc = r["from_can"];
      JSONVar tm = r["to_modbus"];

      if (!fc.hasOwnProperty("message") || JSON.typeof(fc["message"]) != "string" || !tm.hasOwnProperty("resource")|| JSON.typeof(tm["resource"])!= "string") 
      {
        Serial.println(F("[MAP] from_can.message / to_modbus.resource mancanti"));
        return false;
      }

      // Salva i nomi e risolvi puntatori
      rule.from = (const char*)fc["message"];
      rule.to   = (const char*)tm["resource"];

      rule.fromCan  = findCanByName(canMsgs, rule.from);
      rule.toModbus = findMbResByName(mbRes, rule.to);

      if (!rule.fromCan)  
      { 
        Serial.println(F("[MAP] message CAN non trovato"));     
        return false; 
      }

      if (!rule.toModbus) 
      { 
        Serial.println(F("[MAP] resource Modbus non trovata")); 
        return false; 
      }

      // array "map"
      if (!r.hasOwnProperty("map") || JSON.typeof(r["map"]) != "array") 
      {
        Serial.println(F("[MAP] array 'map' mancante"));
        return false;
      }

      JSONVar mp = r["map"];

      for (unsigned int k=0; k<mp.length(); ++k) 
      {
        JSONVar m = mp[k];

        if (JSON.typeof(m) != "object" || !m.hasOwnProperty("src") || !m.hasOwnProperty("dst"))
        {
          continue;
        }
          
        String src = (const char*)m["src"];
        String dst = (const char*)m["dst"];

        /// validazione src esiste nel CAN message, dst esiste nella resource Modbus
        const FieldSpec*   srcF = findFieldByName(rule.fromCan->fields, src);
        const ModbusField* dstF = findMbFieldByName(rule.toModbus->fields, dst);

        if (!srcF || !dstF) 
        {
          Serial.println(F("[MAP] campo src/dst non trovato in CAN2MB"));
          return false;
        }
        rule.pairs.push_back({src, dst});
      }
    }

    outRules.push_back(rule);
  }

  // ritorna true se abbiamo caricato almeno una regola
  return !outRules.empty();
}

// MB -> CAN : costruire payload CAN a partire dal buffer di registri Modbus
// - regBuf: array di registri letti dalla risorsa Modbus
// - regCount: quanti registri sono validi in regBuf
// - outId/outDlc/outData: risultati (id, dlc, payload)

bool buildCanFromModbus(const MappingRule& rule, const uint16_t* regBuf, uint16_t regCount, uint32_t& outId, uint8_t& outDlc, uint8_t outData[8])
{
  //validazione iniziale: tipo regola e puntatori risolti
  if (rule.dir != RuleDir::MB2CAN || !rule.fromModbus || !rule.toCan) return false;

  // imposta header CAN
  outId  = rule.toCan->id;
  outDlc = rule.toCan->dlc;
  for (uint8_t i=0;i<outDlc;i++) outData[i]=0;

  // per ogni coppia (src Modbus -> dst CAN)
  for (auto& p : rule.pairs) 
  {
    // trova field Modbus sorgente e field CAN destinazione
    const ModbusField* srcF = findMbFieldByName(rule.fromModbus->fields, p.src);
    const FieldSpec*   dstF = findFieldByName   (rule.toCan->fields,       p.dst);

    if (!srcF || !dstF) 
    {
      return false;
    }

    // controlla che il campo CAN entri nel DLC
    if (dstF->offset + dstF->size > outDlc) 
    {
      return false;
    }
    uint8_t* dst = &outData[dstF->offset];

    // Leggi dal buffer Modbus e scrivi nel payload CAN convertendo/scalandp
    switch (srcF->type) {

      case FieldType::Bool: {
        if (srcF->index >= regCount) return false;
        uint16_t reg = regBuf[srcF->index];
        uint8_t b = (reg & 0x0001) ? 1 : 0;  // bit0
        writeValue<uint8_t>(dst, b, dstF->endian, dstF->size);
      } break;

      case FieldType::Uint16: {
        if (srcF->index >= regCount) return false;
        uint16_t u = regBuf[srcF->index];
        if (dstF->type == FieldType::Float32) 
        {
          float f = (float)u / (float)srcF->scale;
          writeValue<float>(dst, f, dstF->endian, dstF->size);
        } 
        else 
        {
          uint16_t v = (uint16_t)((double)u / srcF->scale);
          writeValue<uint16_t>(dst, v, dstF->endian, dstF->size);
        }
      } break;

      case FieldType::Int16: {
        if (srcF->index >= regCount) return false;
        int16_t s = (int16_t)regBuf[srcF->index];
        if (dstF->type == FieldType::Float32) 
        {
          float f = (float)s / (float)srcF->scale;
          writeValue<float>(dst, f, dstF->endian, dstF->size);
        } 
        else 
        {
          int16_t v = (int16_t)((double)s / srcF->scale);
          writeValue<int16_t>(dst, v, dstF->endian, dstF->size);
        }
      } break;

      case FieldType::Float32: {
        // float memorizzato su 2 registri: lo/hi (qui assumiamo lo e hi ordinati in regBuf)
        if (srcF->index + 1 >= regCount) return false;
        uint16_t lo = regBuf[srcF->index];
        uint16_t hi = regBuf[srcF->index + 1];
        uint32_t u32 = ((uint32_t)hi << 16) | lo;  // word order: [hi][lo]
        union { uint32_t u; float f; } cvt; 
        cvt.u = u32;
        float f = cvt.f / (float)srcF->scale;
        writeValue<float>(dst, f, dstF->endian, dstF->size);
      } break;

      default: return false;
    }
  }

  return true;
}

// CAN -> MB : estrae valori dal payload CAN e scrive nei registri Modbus
// - rxData/rxDlc: payload ricevuto
// - regsOut: buffer destinazione (deve avere dimensione >= outCount)
bool extractModbusFromCan(const MappingRule& rule, const uint8_t* rxData, uint8_t rxDlc, uint16_t* regsOut, uint16_t outCount)
{
  if (rule.dir != RuleDir::CAN2MB || !rule.fromCan || !rule.toModbus) 
  {
    return false;
  }

  // per ogni coppia (src CAN -> dst Modbus)
  for (auto& p : rule.pairs) 
  {
    const FieldSpec*   srcF = findFieldByName   (rule.fromCan->fields,     p.src);
    const ModbusField* dstF = findMbFieldByName(rule.toModbus->fields,     p.dst);

    // controlla che il campo CAN sia presente nel payload
    if (!srcF || !dstF) 
    {
      return false;
    }

    if (srcF->offset + srcF->size > rxDlc) 
    {
      return false;
    }
    const uint8_t* src = &rxData[srcF->offset];

    switch (dstF->type) {
      case FieldType::Bool: {
        uint8_t v = readValue<uint8_t>(src, srcF->endian, srcF->size);
        if (dstF->index >= outCount) return false;
        regsOut[dstF->index] = (regsOut[dstF->index] & ~0x0001) | (v ? 1 : 0);
      } break;

      case FieldType::Uint16: {
        uint16_t u = readValue<uint16_t>(src, srcF->endian, srcF->size);
        uint16_t raw = (uint16_t)((double)u * dstF->scale);
        if (dstF->index >= outCount) return false;
        regsOut[dstF->index] = raw;
      } break;

      case FieldType::Int16: {
        int16_t s = readValue<int16_t>(src, srcF->endian, srcF->size);
        int16_t raw = (int16_t)((double)s * dstF->scale);
        if (dstF->index >= outCount) return false;
        regsOut[dstF->index] = (uint16_t)raw;
      } break;

      case FieldType::Float32: {
        float f = readValue<float>(src, srcF->endian, srcF->size);
        float scaled = f * (float)dstF->scale;
        if (dstF->index + 1 >= outCount) return false;
        union { uint32_t u; float f; } cvt; cvt.f = scaled;
        uint16_t lo = (uint16_t)(cvt.u & 0xFFFF);
        uint16_t hi = (uint16_t)((cvt.u >> 16) & 0xFFFF);
        regsOut[dstF->index]     = lo;
        regsOut[dstF->index + 1] = hi;
      } break;

      default: return false;
    }
  }

  return true;
}
