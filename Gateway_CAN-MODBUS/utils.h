#pragma once
#include <Arduino.h>
#include <Arduino_JSON.h>
#include <vector>

// ======================= Tipi generali =======================
enum class Endian : uint8_t { Little, Big };
enum class FieldType : uint8_t { Uint16, Int16, Float32, Bool, Unknown };
enum class CanDir : uint8_t { BOTH, NET2INT, INT2NET, INVALID };

// ======================= CAN spec ============================
struct FieldSpec {
  String    name;
  FieldType type       = FieldType::Unknown;
  uint16_t  offset     = 0;   // byte offset nel payload
  uint8_t   size       = 0;   // 1,2,4
  Endian    endian     = Endian::Little;
  double    scale      = 1.0; // opzionale
};

struct CanMessageSpec {
  String               name;
  uint32_t             id      = 0;
  uint8_t              dlc     = 0;
  CanDir               dir     = CanDir::INVALID;
  std::vector<FieldSpec> fields;
};

// ======================= Modbus spec =========================
enum class ModbusFn : uint8_t { ReadHolding, WriteSingle, WriteMultiple, Unknown };

struct ModbusField {
  String    name;
  FieldType type   = FieldType::Unknown; // supporta u16/i16/float32/bool
  uint16_t  index  = 0;                  // indice nel blocco di registri
  uint8_t   count  = 1;                  // numero registri (es. float=2)
  double    scale  = 1.0;                // opzionale
};

struct ModbusResourceSpec {
  String                name;
  ModbusFn              fn        = ModbusFn::Unknown;
  uint16_t              address   = 0;
  uint16_t              count     = 0;       // n registri coinvolti
  uint32_t              period_ms = 0;       // 0 = nessun polling
  std::vector<ModbusField> fields;
};

struct ModbusRtuConfig {
  uint32_t baud      = 9600;
  char     parity    = 'N'; // 'N','E','O'
  uint8_t  stop_bits = 1;
  uint8_t  slave_id  = 1;
};

// ======================= Mapping spec ========================
enum class RuleDir : uint8_t { MB2CAN, CAN2MB };

struct MapPair {
  String src; // nome field sorgente
  String dst; // nome field destinazione
};

struct MappingRule {
  RuleDir dir = RuleDir::MB2CAN;
  String  from;
  String  to;

  // puntatori risolti dopo parsing
  const ModbusResourceSpec* fromModbus = nullptr;
  const ModbusResourceSpec* toModbus   = nullptr;
  const CanMessageSpec*     fromCan    = nullptr;
  const CanMessageSpec*     toCan      = nullptr;

  std::vector<MapPair> pairs; // <— era "map"
};

// ======================= Helpers string/parse =================
String   trimBoth(const String& s);
bool     parseUIntFlexible(const String& s, uint32_t& v);
bool     strToBool(const String& s, bool& out);

Endian    parseEndianStr(const String& s);
FieldType parseFieldType(const String& s);
CanDir    parseDirStr(const String& s);
ModbusFn  parseModbusFn(const String& s);

// ======================= Template generics ====================
template<typename T>
void writeValue(uint8_t* buf, T v, Endian e, uint8_t size);

template<typename T>
T readValue(const uint8_t* buf, Endian e, uint8_t size);

// ======================= Parsers JSON =========================
bool parseCanJson     (const String& json, long& outBitrate,
                       std::vector<CanMessageSpec>& outMsgs);

bool parseModbusJson  (const String& json, ModbusRtuConfig& outRTU,
                       std::vector<ModbusResourceSpec>& outRes);

bool parseMappingJson (const String& json,
                       const std::vector<ModbusResourceSpec>& mbRes,
                       const std::vector<CanMessageSpec>& canMsgs,
                       std::vector<MappingRule>& outRules);

// ======================= find helpers =========================
const CanMessageSpec*     findCanByName(const std::vector<CanMessageSpec>& v, const String& name);
const FieldSpec*          findFieldByName(const std::vector<FieldSpec>& v, const String& name);
const ModbusResourceSpec* findMbResByName(const std::vector<ModbusResourceSpec>& v, const String& name);
const ModbusField*        findMbFieldByName(const std::vector<ModbusField>& v, const String& name);

// ============= esplicite instanziazioni dichiarate =============
extern template void writeValue<uint16_t>(uint8_t*, uint16_t, Endian, uint8_t);
extern template void writeValue<int16_t>(uint8_t*, int16_t, Endian, uint8_t);
extern template void writeValue<float>(uint8_t*, float, Endian, uint8_t);
extern template void writeValue<uint8_t>(uint8_t*, uint8_t, Endian, uint8_t);

extern template uint16_t readValue<uint16_t>(const uint8_t*, Endian, uint8_t);
extern template int16_t  readValue<int16_t>(const uint8_t*, Endian, uint8_t);
extern template float    readValue<float>(const uint8_t*, Endian, uint8_t);
extern template uint8_t  readValue<uint8_t>(const uint8_t*, Endian, uint8_t);
