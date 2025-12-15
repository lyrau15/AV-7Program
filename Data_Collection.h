#pragma once

#include <vector>
#include <RTClib.h>
// #include <ModbusMaster.h>
#include <SoftwareSerial.h>

//RTC Parameter
#define SDA_I2C 8
#define SCL_I2C 9

extern RTC_PCF8563 rtc;
extern String hari, tanggal, bulan, tahun, jam, menit, detik;
volatile unsigned long UnixTime;

extern DateTime NetTime;

// //RS485 Parameter
SoftwareSerial SSerial(18, 17);
#define MODBUS SSerial
#define MODBUS_TIMEOUT 100
// ModbusMaster node;
String SemeatechType[41] = { "", "", "CO", "O2", "H2", "CH4", "", "CO2", "O3", "H2S", "SO2", "NH3", "CL2", "ETO", "HCL", "PH3", "", "HCN", "", "HF", "",
                             "NO", "NO2", "NOX", "CLO2", "", "", "", "", "", "", "THT", "C2H2", "C2H4", "CH2O", "", "", "", "", "CH3SH", "C2H3C" };


void getRTCParameter() {
  DateTime now = rtc.now();
  tanggal = (now.day() < 10 ? "0" : "") + String(now.day());
  bulan = (now.month() < 10 ? "0" : "") + String(now.month());
  tahun = (now.year() < 10 ? "0" : "") + String(now.year());
  jam = (now.hour() < 10 ? "0" : "") + String(now.hour());
  menit = (now.minute() < 10 ? "0" : "") + String(now.minute());
  detik = (now.second() < 10 ? "0" : "") + String(now.second());
  UnixTime = now.unixtime();
}

// =======================
// GLOBAL OUTPUT VARIABLES (punyamu)
// =======================
extern float pm10, pm25, so2, co, o3, no2, hc, ws, wd,pm10_ppm, pm25_ppm, so2_ppm, co_ppm, o3_ppm, no2_ppm, hc_ppm;
extern int stat_pm10, stat_pm25, stat_so2, stat_co, stat_o3, stat_no2, stat_hc;
extern float humidity, temperature, pressure, sr, rain_intensity, FlowPM;

// =======================
// PRIVATE HELPERS (semua logika ada di sini)
// =======================

static inline void _setStat(int &statVar, bool ok) {
  statVar = ok ? 1 : 0;
}

static inline void _flushRx(Stream &p) {
  while (p.available()) p.read();
}

static uint16_t _modbusCRC16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= data[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else crc >>= 1;
    }
  }
  return crc;  // kirim low dulu, lalu high
}

static bool _modbusReadRegisters(
  Stream &port,
  uint8_t slaveId,
  uint8_t functionCode,  // 0x03 holding, 0x04 input
  uint16_t startAddr,
  uint16_t quantity,
  uint16_t *dest,
  uint32_t timeoutMs) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = functionCode;
  req[2] = highByte(startAddr);
  req[3] = lowByte(startAddr);
  req[4] = highByte(quantity);
  req[5] = lowByte(quantity);

  uint16_t crc = _modbusCRC16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  _flushRx(port);
  port.write(req, 8);
  port.flush();

  const uint8_t minRespLen = 5;
  uint8_t resp[128];
  uint16_t idx = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < timeoutMs) {
    while (port.available()) {
      int b = port.read();
      if (b < 0) break;
      if (idx < sizeof(resp)) resp[idx++] = (uint8_t)b;
    }

    if (idx >= minRespLen) {
      uint8_t byteCount = resp[2];
      uint16_t expectedLen = 3 + byteCount + 2;

      if (idx >= expectedLen) {
        if (resp[0] != slaveId) return false;
        if (resp[1] & 0x80) return false;
        if (resp[1] != functionCode) return false;
        if (byteCount != quantity * 2) return false;

        uint16_t crcCalc = _modbusCRC16(resp, expectedLen - 2);
        uint16_t crcResp = resp[expectedLen - 2] | (resp[expectedLen - 1] << 8);
        if (crcCalc != crcResp) return false;

        for (uint16_t i = 0; i < quantity; i++) {
          uint8_t hi = resp[3 + i * 2];
          uint8_t lo = resp[3 + i * 2 + 1];
          dest[i] = ((uint16_t)hi << 8) | lo;
        }
        return true;
      }
    }
  }
  return false;
}

static bool _modbusWriteRegisters(
  Stream &port,
  uint8_t slaveId,
  uint8_t functionCode,  // 0x10
  uint16_t startAddr,
  uint16_t quantity,
  const uint16_t *values,
  uint32_t timeoutMs) {
  if (quantity == 0) return false;
  uint16_t byteCount = quantity * 2;
  if (byteCount > 246) return false;

  uint8_t req[260];
  uint16_t idx = 0;

  req[idx++] = slaveId;
  req[idx++] = functionCode;
  req[idx++] = highByte(startAddr);
  req[idx++] = lowByte(startAddr);
  req[idx++] = highByte(quantity);
  req[idx++] = lowByte(quantity);
  req[idx++] = (uint8_t)byteCount;

  for (uint16_t i = 0; i < quantity; i++) {
    req[idx++] = highByte(values[i]);
    req[idx++] = lowByte(values[i]);
  }

  uint16_t crc = _modbusCRC16(req, idx);
  req[idx++] = crc & 0xFF;
  req[idx++] = (crc >> 8) & 0xFF;

  _flushRx(port);
  port.write(req, idx);
  port.flush();

  const uint8_t expectedLen = 8;
  uint8_t resp[16];
  uint16_t rIdx = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < timeoutMs) {
    while (port.available()) {
      int b = port.read();
      if (b < 0) break;
      if (rIdx < sizeof(resp)) resp[rIdx++] = (uint8_t)b;
    }

    if (rIdx >= expectedLen) {
      if (resp[0] != slaveId) return false;
      if (resp[1] & 0x80) return false;
      if (resp[1] != functionCode) return false;

      uint16_t respStart = ((uint16_t)resp[2] << 8) | resp[3];
      uint16_t respQty = ((uint16_t)resp[4] << 8) | resp[5];
      if (respStart != startAddr) return false;
      if (respQty != quantity) return false;

      uint16_t crcCalc = _modbusCRC16(resp, expectedLen - 2);
      uint16_t crcResp = resp[expectedLen - 2] | (resp[expectedLen - 1] << 8);
      if (crcCalc != crcResp) return false;

      return true;
    }
  }
  return false;
}

static inline int32_t _hexArrayToDecimal(const uint8_t *hexArray, int length) {
  int32_t value = 0;
  for (int i = 0; i < length; i++) value = (value << 8) | hexArray[i];
  return value;
}

// SEMEATECH: frame kamu (0x3A ...)
// NOTE: ini bukan Modbus RTU standar, tapi kamu sudah punya formatnya.
// Aku pertahankan sama.
static void _writeSemeatechFrame(Stream &port, uint8_t *data, int panjang, bool crcSwap) {
  uint16_t crc = 0xFFFF;
  for (int p = 0; p < panjang; p++) {
    crc ^= data[p];
    for (int i = 0; i < 8; i++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  port.write(data, panjang);
  if (!crcSwap) {
    port.write(lowByte(crc));
    port.write(highByte(crc));
  } else {
    port.write(highByte(crc));
    port.write(lowByte(crc));
  }
}

static bool _getSensSemeatech(uint8_t devId, int32_t &uG, int32_t &ppb, String &sensorType, uint32_t timeoutMs) {
  if (devId < 1 || devId > 255) return false;

  uint8_t buf[32];
  int readLength = 0;

  uint8_t getSensorType[] = { 0x3A, devId, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00 };
  uint8_t getSensorValue[] = { 0x3A, devId, 0x03, 0x00, 0x00, 0x06, 0x00, 0x00 };

  memset(buf, 0, sizeof(buf));
  _flushRx(MODBUS);
  _writeSemeatechFrame(MODBUS, getSensorType, sizeof(getSensorType), false);

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (MODBUS.available()) {
      buf[readLength++] = (uint8_t)MODBUS.read();
      if (readLength >= 4) break;
    }
    if (readLength >= 4) break;
  }

  // buf[3] adalah type code (punyamu)
  if (readLength < 4) return false;
  if (buf[3] >= 0 && buf[3] < 41) sensorType = SemeatechType[buf[3]];
  else return false;

  readLength = 0;
  memset(buf, 0, sizeof(buf));
  _flushRx(MODBUS);
  _writeSemeatechFrame(MODBUS, getSensorValue, sizeof(getSensorValue), false);

  t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (MODBUS.available()) {
      buf[readLength++] = (uint8_t)MODBUS.read();
      if (readLength >= 18) break;
    }
    if (readLength >= 18) break;
  }

  if (readLength < 14) return false;
  if (buf[0] != 0x3A) return false;

  uint8_t cuG[4] = { buf[6], buf[7], buf[8], buf[9] };
  uint8_t cppb[4] = { buf[10], buf[11], buf[12], buf[13] };

  uG = _hexArrayToDecimal(cuG, 4);
  ppb = _hexArrayToDecimal(cppb, 4);
  return true;
}

static float _sens_mV_per_ppm(const String &type) {
  if (type == "5") return 200.0f;
  if (type == "10") return 100.0f;
  if (type == "20") return 60.0f;
  if (type == "100") return 10.0f;
  if (type == "200") return 5.0f;
  if (type == "1000") return 1.0f;
  if (type == "2000") return 0.5f;
  if (type == "5000") return 0.2f;
  if (type == "10000") return 0.1f;
  return -1.0f;
}

static bool _getSensSenovolPID(int pinAnalog, float &ppbOut, float baseline, const String &typeRange) {
  float sens_mV = _sens_mV_per_ppm(typeRange);
  if (sens_mV <= 0) return false;

  int readADC = analogRead(pinAnalog);
  float voltage = readADC * 3.3f / 4095.0f;
  float effective = voltage - baseline;
  if (effective < 0) effective = 0;

  float sens_V_per_ppm = sens_mV / 1000.0f;
  float ppm = (sens_V_per_ppm > 0) ? (effective / sens_V_per_ppm) : 0.0f;

  ppbOut = ppm * 1000.0f;
  return true;
}

static bool _getSensPM(const String &type, int address,
                       float &dat1, float &dat2, float &dat3, float &dat4, float &dat5,
                       uint32_t timeoutMs) {
  if (address < 0 || address > 255) return false;
  uint16_t regs[64];

  if (type == "OPC_N3") {
    bool ok = _modbusReadRegisters(MODBUS, (uint8_t)address, 0x03, 0, 9, regs, timeoutMs);
    if (!ok) return false;

    uint16_t Dset1 = regs[1];  // PM2.5
    uint16_t Dset2 = regs[2];  // PM10
    uint16_t Dset3 = regs[5];  // Flow
    uint16_t Dset4 = regs[7];  // Timer Set
    uint16_t Dset5 = regs[8];  // Timer Now

    dat1 = (float)Dset1;
    dat2 = (float)Dset2;
    dat3 = (float)Dset3;
    dat4 = (float)Dset4;
    dat5 = (float)Dset5;
    return true;
  }

  if (type == "METONE") {
    bool ok = _modbusReadRegisters(MODBUS, (uint8_t)address, 0x04, 100, 14, regs, timeoutMs);
    if (!ok) return false;

    uint32_t CConc = ((uint32_t)regs[1] << 16) | regs[0];
    uint32_t CTemp = ((uint32_t)regs[3] << 16) | regs[2];
    uint32_t CPres = ((uint32_t)regs[7] << 16) | regs[6];
    uint32_t CLaser = ((uint32_t)regs[11] << 16) | regs[10];
    uint32_t CFlow = ((uint32_t)regs[13] << 16) | regs[12];

    union {
      uint32_t u32;
      float f;
    } conv;

    conv.u32 = CConc;
    dat1 = conv.f;  // Conc
    conv.u32 = CPres;
    dat2 = conv.f;  // Pressure
    conv.u32 = CFlow;
    dat3 = conv.f;  // Flow
    conv.u32 = CLaser;
    dat4 = conv.f;  // Laser
    conv.u32 = CTemp;
    dat5 = conv.f;  // Temp
    return true;
  }

  return false;
}

static bool _getSensWSSensorRIKA(int address,
                                 float &wdOut, float &wsOut,
                                 float &tempOut, float &humOut, float &presOut,
                                 float &rainOut, float &srOut,  // srOut = radiation (hasil rumus)
                                 uint32_t timeoutMs) {
  if (address < 0 || address > 255) return false;

  uint16_t regs[64];
  bool ok = _modbusReadRegisters(MODBUS, (uint8_t)address, 0x03, 0, 41, regs, timeoutMs);
  if (!ok) return false;

  uint16_t WindDirection = regs[1];

  uint32_t CWindSpeed = ((uint32_t)regs[3] << 16) | regs[2];
  uint32_t CAirTemp = ((uint32_t)regs[5] << 16) | regs[4];
  uint32_t CAirHum = ((uint32_t)regs[7] << 16) | regs[6];
  uint32_t CAirPres = ((uint32_t)regs[9] << 16) | regs[8];
  uint32_t CRainfall = ((uint32_t)regs[13] << 16) | regs[12];
  uint32_t rowRadiation = ((uint32_t)regs[35] << 16) | regs[34];

  union {
    uint32_t u32;
    float f;
  } conv;

  wdOut = (float)WindDirection;
  conv.u32 = CWindSpeed;
  wsOut = conv.f;
  conv.u32 = CAirTemp;
  tempOut = conv.f;
  conv.u32 = CAirHum;
  humOut = conv.f;
  conv.u32 = CAirPres;
  presOut = conv.f;
  conv.u32 = CRainfall;
  rainOut = conv.f;

  // rumus kamu: Radiation = raw * 2000 / 65535
  srOut = (float)rowRadiation * 2000.0f / 65535.0f;
  return true;
}

static bool _getOtherSensorSENTEC(int address, float &srRaw, uint32_t timeoutMs) {
  if (address < 0 || address > 255) return false;
  uint16_t regs[2];
  bool ok = _modbusReadRegisters(MODBUS, (uint8_t)address, 0x03, 0, 1, regs, timeoutMs);
  if (!ok) return false;
  srRaw = (float)regs[0];
  return true;
}


// =======================
// ✅ SINGLE ENTRY POINT
// =======================
bool readSensor(const String &sensorType, int baudrate, int address) {
  // Default: jangan ubah nilai kalau gagal (biar nilai terakhir masih ada)
  // Tapi status akan diupdate.

  bool ok = false;

  // ------------ PM ------------
  if (sensorType == "OPC_N3") {
    // OPC_N3: 9600 8N1 (umumnya)
    // kalau baudrate dari config kamu, pakai itu.
    // begin untuk serial yang kamu pakai, jika MODBUS bukan SoftwareSerial, abaikan config.
    // (kalau MODBUS kamu HardwareSerial, begin() tidak ada di Stream — jadi kamu biasanya punya MODBUS.begin sendiri di luar)
    // Di sini kita anggap kamu memang punya begin-end seperti sebelumnya.

    // Kalau MODBUS kamu bukan object yang ada begin/end, hapus 3 baris ini dan atur di luar.
    MODBUS.end();
    delay(20);
    MODBUS.begin((uint32_t)baudrate, SWSERIAL_8N1);
    delay(20);
    _flushRx(MODBUS);

    float d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0;
    ok = _getSensPM("OPC_N3", address, d1, d2, d3, d4, d5, MODBUS_TIMEOUT);
    if (ok) {
      pm25 = d1;
      pm10 = d2;
    }
    _setStat(stat_pm25, ok);
    _setStat(stat_pm10, ok);
    return ok;
  }

  if (sensorType == "METONE_PM25" || sensorType == "METONE_PM10") {
    MODBUS.end();
    delay(20);
    MODBUS.begin((uint32_t)baudrate, SWSERIAL_8N1);
    delay(20);
    _flushRx(MODBUS);

    float conc = 0, pres = 0, flow = 0, laser = 0, temp = 0;
    ok = _getSensPM("METONE", address, conc, pres, flow, laser, temp, MODBUS_TIMEOUT);

    if (ok) {
      if (sensorType == "METONE_PM25") pm25 = conc;
      else pm10 = conc;

      // kalau kamu mau ambil temp/pres dari salah satu saja, pilih yang PM25 (umum)
      if (sensorType == "METONE_PM25") {
        pressure = pres;
        temperature = temp;
      }
    }

    if (sensorType == "METONE_PM25") _setStat(stat_pm25, ok);
    else _setStat(stat_pm10, ok);

    return ok;
  }

  // ------------ WEATHER STATION ------------
  if (sensorType == "RIKA") {
    MODBUS.end();
    delay(20);
    MODBUS.begin((uint32_t)baudrate, SWSERIAL_8E1);
    delay(20);
    _flushRx(MODBUS);

    float wdOut = 0, wsOut = 0, t = 0, h = 0, p = 0, rain = 0, rad = 0;
    ok = _getSensWSSensorRIKA(address, wdOut, wsOut, t, h, p, rain, rad, MODBUS_TIMEOUT);

    if (ok) {
      wd = wdOut;
      ws = wsOut;
      temperature = t;
      humidity = h;
      pressure = p;
      rain_intensity = rain;
      // rad bisa kamu taruh ke sr juga kalau mau:
      // sr = rad;
    }
    return ok;
  }

  // ------------ SOLAR RADIATION SENTEC ------------
  if (sensorType == "SENTEC") {
    MODBUS.end();
    delay(20);
    MODBUS.begin((uint32_t)baudrate, SWSERIAL_8N1);
    delay(20);
    _flushRx(MODBUS);

    float v = 0;
    ok = _getOtherSensorSENTEC(address, v, MODBUS_TIMEOUT);
    if (ok) sr = v;
    return ok;
  }

  // ------------ GAS SEMEATECH ------------
  if (sensorType == "SEMEATECH") {
    MODBUS.end();
    delay(20);
    MODBUS.begin((uint32_t)baudrate, SWSERIAL_8N1);
    delay(20);
    _flushRx(MODBUS);

    int32_t ug = 0, ppbVal = 0;
    String gasType = "";
    ok = _getSensSemeatech((uint8_t)address, ug, ppbVal, gasType, MODBUS_TIMEOUT);

    if (ok) {
      String g = gasType;
      g.toUpperCase();
      if (g.indexOf("SO2") >= 0) {
        so2 = (float)ppbVal;
        _setStat(stat_so2, true);
      } else if (g.indexOf("CO") >= 0) {
        co = (float)ppbVal;
        _setStat(stat_co, true);
      } else if (g.indexOf("O3") >= 0) {
        o3 = (float)ppbVal;
        _setStat(stat_o3, true);
      } else if (g.indexOf("NO2") >= 0) {
        no2 = (float)ppbVal;
        _setStat(stat_no2, true);
      } else if (g.indexOf("HC") >= 0 || g.indexOf("VOC") >= 0) {
        hc = (float)ppbVal;
        _setStat(stat_hc, true);
      }
    } else {
      // kalau mau: set fail semua
      // _setStat(stat_so2,false); _setStat(stat_co,false); _setStat(stat_o3,false); _setStat(stat_no2,false); _setStat(stat_hc,false);
    }
    return ok;
  }

  // ------------ PID SENOVOL (ANALOG) ------------
  // format sensorType: "SENOVOL:200" atau "SENOVOL:1000"
  if (sensorType.startsWith("SENOVOL:")) {
    String range = sensorType.substring(String("SENOVOL:").length());
    float ppbOut = 0;

    // address dianggap PIN analog
    ok = _getSensSenovolPID(address, ppbOut, 0.0f /*baseline*/, range);
    if (ok) {
      hc = ppbOut;
      _setStat(stat_hc, true);
    } else {
      _setStat(stat_hc, false);
    }
    return ok;
  }

  // ------------ UNKNOWN ------------
  safePrintln("[SENSOR] Unknown sensorType: " + sensorType);
  return false;
}

static void printReadResult(
  const String &sensorType,
  int baudrate,
  int address,
  bool ok,
  int idx,
  uint32_t dt) {
  String line = "[SENSOR] idx=" + String(idx) + " " + sensorType + " baud=" + String(baudrate) + " addr=" + String(address) + " | ";

  if (!ok) {
    line += "FAIL timeout/crc/resp";
    line += " | t=" + String(dt) + "ms";
    safePrintln(line);
    return;
  }

  // ===== SUKSES =====
  if (sensorType == "OPC_N3") {
    line += "OK PM25=" + String(pm25, 0) + " PM10=" + String(pm10, 0);
  } else if (sensorType == "METONE_PM25") {
    line += "OK PM25=" + String(pm25, 2) + " T=" + String(temperature, 2) + " P=" + String(pressure, 2);
  } else if (sensorType == "METONE_PM10") {
    line += "OK PM10=" + String(pm10, 2);
  } else if (sensorType == "RIKA") {
    line += "OK WD=" + String(wd, 0) + " WS=" + String(ws, 2) + " T=" + String(temperature, 2) + " H=" + String(humidity, 2);
  } else if (sensorType == "SENTEC") {
    line += "OK SR=" + String(sr, 2);
  } else if (sensorType == "SEMEATECH") {
    line += "OK SO2=" + String(so2, 2) + " CO=" + String(co, 2) + " O3=" + String(o3, 2) + " NO2=" + String(no2, 2) + " HC=" + String(hc, 2);
  } else if (sensorType.startsWith("SENOVOL:")) {
    line += "OK HC(ppb)=" + String(hc, 2);
  } else {
    line += "OK (no mapping)";
  }

  line += " | t=" + String(dt) + "ms";
  safePrintln(line);
}

void generateRandomSensorData() {
  // Particulate Matter (µg/m3)
  pm25 = random(5, 150);        // PM2.5
  pm10 = pm25 + random(0, 80);  // PM10 biasanya > PM2.5

  // Gas (ppm atau ppb tergantung sensor)
  so2 = random(0, 200) / 10.0;  // 0 – 20.0
  co = random(0, 300) / 10.0;   // 0 – 30.0
  o3 = random(0, 200) / 10.0;   // 0 – 20.0
  no2 = random(0, 200) / 10.0;  // 0 – 20.0
  hc = random(0, 500) / 10.0;   // 0 – 50.0
  pm10_ppm = random(0, 200) / 10.0;;
  pm25_ppm = random(0, 200) / 10.0;; 
  so2_ppm = random(0, 200) / 10.0;;
  co_ppm = random(0, 200) / 10.0;; 
  o3_ppm = random(0, 200) / 10.0;; 
  no2_ppm = random(0, 200) / 10.0;; 
  hc_ppm = random(0, 200) / 10.0;;

  // Cuaca / lingkungan
  temperature = random(200, 380) / 10.0;   // 20 – 38 °C
  humidity = random(300, 900) / 10.0;      // 30 – 90 %
  pressure = random(9900, 10300) / 10.0;   // 990 – 1030 hPa
  sr = random(0, 1200);                    // Solar radiation
  ws = random(0, 150) / 10.0;              // Wind speed (0 – 15 m/s)
  wd = random(0, 360);                     // Wind direction
  rain_intensity = random(0, 200) / 10.0;  // mm/h
  FlowPM = random(10, 50) / 10.0;          // airflow PM sensor

  // Status (0 = baik, 1 = sedang, 2 = buruk contoh sederhana)
  stat_pm25 = (pm25 > 55) ? 2 : (pm25 > 35 ? 1 : 0);
  stat_pm10 = (pm10 > 150) ? 2 : (pm10 > 75 ? 1 : 0);
  stat_so2 = (so2 > 10) ? 2 : (so2 > 5 ? 1 : 0);
  stat_co = (co > 15) ? 2 : (co > 9 ? 1 : 0);
  stat_o3 = (o3 > 10) ? 2 : (o3 > 5 ? 1 : 0);
  stat_no2 = (no2 > 10) ? 2 : (no2 > 5 ? 1 : 0);
  stat_hc = (hc > 25) ? 2 : (hc > 10 ? 1 : 0);
}

// /*===============NEXTION===============*/
// unsigned long lastRequest = 0;

// // Konfigurasi yang mau di-load dan di-save lewat HMI
// uint16_t NXcfg[5] = { 2, 2, 2, 2, 2 };  // pm25,pm10,memrapor1,membrapor2,davis

// // Penanda numeric return lagi baca apa
// enum NumericTarget : uint8_t {
//   NUM_NONE = 0,
//   NUM_READ = 1,  // baca read.val
//   NUM_SAVE = 2,  // baca save.val
//   NUM_N0 = 3,    // baca n0.val
//   NUM_N1 = 4,    // baca n1.val
//   NUM_N2 = 5,    // baca n2.val
//   NUM_N3 = 6,    // baca n3.val
//   NUM_N4 = 7     // baca n4.val
// };

// static NumericTarget currentNumericTarget = NUM_NONE;

// // Menyimpan nilai n0..n4 dari Nextion saat save.val == 1
// uint16_t nVals[5] = { 0 };
// bool nValid[5] = { false, false, false, false, false };

// // Info page sekarang
// uint8_t currentPage = 0;

// // ---- Tambahan untuk "page confirmed" & resync ----
// uint8_t lastPageIdSeen = 255;
// unsigned long lastPageSeenMs = 0;

// uint8_t nxTimeoutStreak = 0;
// unsigned long lastForceSendme = 0;

// // Poller read/save
// bool pollReadNext = true;  // true = kirim read dulu, false = kirim save
// unsigned long lastPoll = 0;
// const unsigned long POLL_INTERVAL = 600;  // ms

// // Timeout tunggu respon numeric
// unsigned long lastNumericRequest = 0;
// const unsigned long NUMERIC_TIMEOUT = 2500;  // ms

// // State untuk baca n0..n4 secara pelan & retry sampai semua valid
// bool readingSettings = false;
// unsigned long lastNxRequest = 0;
// const unsigned long NX_INTERVAL = 600;  // jeda antar get nX (ms)

// // Interval set command (kurangi blocking)
// const unsigned long CMD_DELAY_MS = 5;

// // Nama objek Number di Nextion untuk n0..n4 (kalau di HMI diganti, tinggal ubah sini)
// const char *NX_NUM_NAMES[5] = { "n0", "n1", "n2", "n3", "n4" };

// void sendCommand(const String &cmd) {
//   Serial1.print(cmd);
//   Serial1.write(0xFF);
//   Serial1.write(0xFF);
//   Serial1.write(0xFF);
// }

// void setNumber(const String &objName, int value) {
//   String cmd = objName + ".val=" + String(value);
//   sendCommand(cmd);
// }

// void setPicture(const String &objName, int value) {
//   String cmd = objName + ".pic=" + String(value);
//   sendCommand(cmd);
// }

// void requestRead() {
//   currentNumericTarget = NUM_READ;
//   lastNumericRequest = millis();
//   safePrintln("Requesting read.val...");
//   sendCommand("get read.val");
// }

// void requestSave() {
//   currentNumericTarget = NUM_SAVE;
//   lastNumericRequest = millis();
//   safePrintln("Requesting save.val...");
//   sendCommand("get save.val");
// }

// // Helper: request get nX.val (n0..n4)
// void requestNx(uint8_t index) {
//   if (index > 4) return;

//   String cmd = String("get ") + NX_NUM_NAMES[index] + ".val";

//   safePrint("Requesting ");
//   safePrintln(cmd);

//   sendCommand(cmd);
//   lastNumericRequest = millis();

//   currentNumericTarget = (NumericTarget)(NUM_N0 + index);
// }

// // ---- Page Logic ----
// void handlePage(uint8_t pageId) {
//   currentPage = pageId;

//   // page confirmed marker
//   lastPageIdSeen = pageId;
//   lastPageSeenMs = millis();
//   nxTimeoutStreak = 0;

//   safePrint("Detected Page : ");

//   // kalau keluar dari page 3, pastikan state setting stop
//   if (pageId != 3) {
//     readingSettings = false;
//     currentNumericTarget = NUM_NONE;
//   }

//   if (pageId == 0) {
//     safePrintln("Page 0 → monitoring ug/m3");

//     setNumber("x0", pm25);
//     delay(CMD_DELAY_MS);
//     setNumber("x1", pm10);
//     delay(CMD_DELAY_MS);
//     setNumber("x2", no2);
//     delay(CMD_DELAY_MS);
//     setNumber("x3", so2);
//     delay(CMD_DELAY_MS);
//     setNumber("x4", co);
//     delay(CMD_DELAY_MS);
//     setNumber("x5", hc);
//     delay(CMD_DELAY_MS);
//     setNumber("x6", o3);
//     delay(CMD_DELAY_MS);
//     setNumber("x7", FlowPM);
//     delay(CMD_DELAY_MS);
//     setNumber("x8", temperature);
//     delay(CMD_DELAY_MS);
//     setNumber("x9", humidity);
//     delay(CMD_DELAY_MS);
//     setNumber("x10", sr);
//     delay(CMD_DELAY_MS);
//     setNumber("x11", rain_intensity);
//     delay(CMD_DELAY_MS);
//     setNumber("x12", ws);
//     delay(CMD_DELAY_MS);
//     setNumber("x13", wd);
//     delay(CMD_DELAY_MS);
//     setNumber("x14", pressure);
//     delay(CMD_DELAY_MS);

//   } else if (pageId == 1) {
//     safePrintln("Page 1 → mgm device");

//     setPicture("p1", 5);
//     delay(CMD_DELAY_MS);
//     setPicture("p2", 5);
//     delay(CMD_DELAY_MS);

//     setNumber("x15", 300);
//     delay(CMD_DELAY_MS);
//     setNumber("x16", 300);
//     delay(CMD_DELAY_MS);
//     setNumber("x17", 300);
//     delay(CMD_DELAY_MS);
//     setNumber("x18", 300);
//     delay(CMD_DELAY_MS);

//   } else if (pageId == 3) {
//     safePrintln("Page 3 → setting");
//     // Poll read/save & baca settings diatur di loop()

//   } else if (pageId == 4) {
//     safePrintln("Page 4 → monitoring PPM");

//     setNumber("x0", pm25_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x1", pm10_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x2", no2_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x3", so2_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x4", co_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x5", hc_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x6", o3_ppm);
//     delay(CMD_DELAY_MS);
//     setNumber("x7", FlowPM);
//     delay(CMD_DELAY_MS);
//     setNumber("x8", temperature);
//     delay(CMD_DELAY_MS);
//     setNumber("x9", humidity);
//     delay(CMD_DELAY_MS);
//     setNumber("x10", sr);
//     delay(CMD_DELAY_MS);
//     setNumber("x11", rain_intensity);
//     delay(CMD_DELAY_MS);
//     setNumber("x12", ws);
//     delay(CMD_DELAY_MS);
//     setNumber("x13", wd);
//     delay(CMD_DELAY_MS);
//     setNumber("x14", pressure);
//     delay(CMD_DELAY_MS);
//   }
// }


// // ---- Parser (stabil: hitung FF 3x + handle error 0x1A + skip event lain) ----
// void readNextion() {
//   static uint8_t state = 0;

//   static uint8_t pageId = 0;
//   static uint32_t numericValue = 0;
//   static uint8_t dataIndex = 0;
//   static uint8_t ffCount = 0;

//   while (Serial1.available()) {
//     uint8_t b = Serial1.read();

//     switch (state) {
//       case 0:             // wait header
//         if (b == 0x66) {  // page
//           state = 1;
//         } else if (b == 0x71) {  // numeric
//           numericValue = 0;
//           dataIndex = 0;
//           state = 10;
//         } else if (b == 0x1A) {  // invalid instruction
//           safePrintln("[NX] ERROR 0x1A (invalid instruction)");
//           ffCount = 0;
//           state = 90;            // consume FF FF FF
//         } else if (b == 0x65) {  // touch event: 3 bytes payload + FF FF FF
//           dataIndex = 0;
//           state = 80;
//         } else if (b == 0x70) {  // string return: until FF FF FF
//           ffCount = 0;
//           state = 85;
//         }
//         break;

//       case 1:  // read page id
//         pageId = b;
//         handlePage(pageId);
//         ffCount = 0;
//         state = 2;  // consume FF FF FF
//         break;

//       case 2:  // consume FF x3 (page)
//         if (b == 0xFF) {
//           ffCount++;
//           if (ffCount >= 3) state = 0;
//         } else {
//           state = 0;
//         }
//         break;

//       case 10:  // read 4 bytes numeric (little endian)
//         numericValue |= ((uint32_t)b << (8 * dataIndex));
//         dataIndex++;
//         if (dataIndex >= 4) {
//           safePrint("Numeric value received: ");
//           safePrintln(String(numericValue));

//           switch (currentNumericTarget) {
//             case NUM_READ:
//               if (numericValue == 1) {
//                 safePrintln("read.val == 1 → kirim NXcfg ke n0..n4");
//                 setNumber("n0", NXcfg[0]);
//                 delay(CMD_DELAY_MS);
//                 setNumber("n1", NXcfg[1]);
//                 delay(CMD_DELAY_MS);
//                 setNumber("n2", NXcfg[2]);
//                 delay(CMD_DELAY_MS);
//                 setNumber("n3", NXcfg[3]);
//                 delay(CMD_DELAY_MS);
//                 setNumber("n4", NXcfg[4]);
//                 delay(CMD_DELAY_MS);

//                 // reset tombol read (lebih aman eksplisit)
//                 sendCommand("read.val=0");
//                 delay(CMD_DELAY_MS);
//                 sendCommand("vis p1,0");
//               }
//               currentNumericTarget = NUM_NONE;
//               break;

//             case NUM_SAVE:
//               if (numericValue == 1) {
//                 safePrintln("save.val == 1 → mulai baca n0..n4 (retry until all valid)");
//                 readingSettings = true;
//                 for (int i = 0; i < 5; i++) nValid[i] = false;
//                 currentNumericTarget = NUM_NONE;
//                 lastNxRequest = 0;
//               } else {
//                 safePrintln("save.val != 1 → batal baca n0..n4");
//                 currentNumericTarget = NUM_NONE;
//               }
//               break;

//             case NUM_N0:
//             case NUM_N1:
//             case NUM_N2:
//             case NUM_N3:
//             case NUM_N4:
//               {
//                 uint8_t idx = (uint8_t)currentNumericTarget - (uint8_t)NUM_N0;
//                 nVals[idx] = (uint16_t)numericValue;
//                 nValid[idx] = true;

//                 safePrint("n");
//                 safePrint(String(idx));
//                 safePrint(" = ");
//                 safePrintln(String(nVals[idx]));

//                 currentNumericTarget = NUM_NONE;
//                 break;
//               }

//             default:
//               safePrintln("Numeric diterima tapi target tidak dikenal, diabaikan.");
//               currentNumericTarget = NUM_NONE;
//               break;
//           }

//           ffCount = 0;
//           state = 11;  // consume FF FF FF
//         }
//         break;

//       case 11:  // consume FF x3 (numeric)
//         if (b == 0xFF) {
//           ffCount++;
//           if (ffCount >= 3) state = 0;
//         } else {
//           state = 0;
//         }
//         break;

//       case 80:  // touch event payload 3 bytes
//         dataIndex++;
//         if (dataIndex >= 3) {
//           ffCount = 0;
//           state = 81;
//         }
//         break;

//       case 81:  // touch terminator FFx3
//         if (b == 0xFF) {
//           ffCount++;
//           if (ffCount >= 3) state = 0;
//         } else {
//           state = 0;
//         }
//         break;

//       case 85:  // string until FFx3
//         if (b == 0xFF) {
//           ffCount++;
//           if (ffCount >= 3) state = 0;
//         } else {
//           ffCount = 0;
//         }
//         break;

//       case 90:  // error terminator FFx3
//         if (b == 0xFF) {
//           ffCount++;
//           if (ffCount >= 3) state = 0;
//         } else {
//           state = 0;
//         }
//         break;
//     }
//   }
// }