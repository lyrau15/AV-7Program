#pragma once

#define TINY_GSM_MODEM_SIM7000SSL
#define TINY_GSM_RX_BUFFER 1024
// #define MQTT_MAX_PACKET_SIZE 1024

//Program Library
#include <ETH.h>
#include <WiFi.h>
#include <time.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include "Data_collection.h"

//Sim7000 Parameter
#define MODEM_RX_PIN 15
#define MODEM_TX_PIN 16

HardwareSerial SIMSerial(2);
TinyGsm modem(SIMSerial);

WiFiMulti WiFiMulti;

TinyGsmClient GSMclient(modem, 0);
TinyGsmClientSecure GSMclientSecure(modem, 1);
WiFiClient netClient;

PubSubClient mqtt;

int modeCon = -1;
bool mqttState = true;

bool connectGSM(int retry) {
  if (!modem.testAT()) {
    return false;
  }

  modem.gprsDisconnect();
  bool ok = false;

  if (retry < 6) {
    // retry 0,1,2,3,4,5 → pakai CAPN
    ok = modem.startModemConnection(CAPN.c_str(), CAPNU.c_str(), CAPNP.c_str());
  } else {
    // retry 6,7,8,9 → pakai DAPN
    ok = modem.startModemConnection(DAPN.c_str(), DAPNU.c_str(), DAPNP.c_str());
  }

  if (!ok) {
    return false;
  }

  return true;
}

bool connectETH(int retry) {
  return false;
}

bool connectWiFi(int retry) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(50);
  WiFi.mode(WIFI_STA);
  delay(50);
  WiFiMulti.addAP(CSSID.c_str(), CSSIDP.c_str());
  WiFiMulti.addAP(DSSID.c_str(), DSSIDP.c_str());

  unsigned long startAttemptTime = millis();
  while (WiFiMulti.run() != WL_CONNECTED && millis() - startAttemptTime < 1000) {
    delay(10);
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  WiFi.disconnect(true);
  return false;
}

#define MODE_WIFI 0
#define MODE_GSM 1
#define MODE_ETH 2

const int MAX_RETRY = 10;

bool tryConnect(int mode, bool &Lock) {
  for (int retry = 0; retry < MAX_RETRY; retry++) {
    safePrint(".");
    esp_task_wdt_reset();
    bool ok = false;

    switch (mode) {
      case MODE_GSM: ok = connectGSM(retry); break;
      case MODE_ETH: ok = connectETH(retry); break;
      case MODE_WIFI: ok = connectWiFi(retry); break;
    }

    if (ok) {
      safePrintln(" CONNECTED!");
      return true;
    }
  }

  safePrintln("FAILED!");
  Lock = true;
  return false;
}

String devIPAddress;
bool connectToInternet(int &MODE_CON) {

  int modes[3];

  // Buat urutan prioritas berdasarkan DCON
  if (MODE_CON == MODE_GSM) {
    modes[0] = MODE_GSM;
    modes[1] = MODE_ETH;
    modes[2] = MODE_WIFI;
  } else if (MODE_CON == MODE_ETH) {
    modes[0] = MODE_ETH;
    modes[1] = MODE_WIFI;
    modes[2] = MODE_GSM;
  } else {
    modes[0] = MODE_WIFI;
    modes[1] = MODE_GSM;
    modes[2] = MODE_ETH;
  }

  safePrintln("[CONNECTION] CONNECTING TO INTERNET...");

  bool ConLock[3] = { false, false, false };
  // iterasi 3 metode koneksi
  for (int i = 0; i < 3; i++) {
    int mode = modes[i];

    if (!ConLock[mode]) {
      if (mode == 0) {
        safePrint("[CONNECTION] Trying WiFi");
      } else if (mode == 1) {
        safePrint("[CONNECTION] Trying GSM");
      } else if (mode == 2) {
        safePrint("[CONNECTION] Trying ETH");
      }
      if (tryConnect(mode, ConLock[mode])) {
        safePrint("[CONNECTION] INTERNET READY, IP: ");
        if (mode == 0) {
          devIPAddress = WiFi.localIP().toString();
          configTime(0, 0, "pool.ntp.org");
          time_t epoch;
          uint32_t t0 = millis();
          while ((epoch = time(nullptr)) < 100000) {
            esp_task_wdt_reset();
            vTaskDelay(300 / portTICK_PERIOD_MS);
            if (millis() - t0 > 10000) {  // 10 detik
              safePrintln("[NTP] TIMEOUT, skip NTP");
              break;
            }
          }
          int32_t offsetSec = TZ_TIME.toInt() * 3600;
          NetTime = DateTime(epoch + offsetSec);
        } else if (mode == 1) {
          int year, month, day, hour, minute, second;
          float tz;
          devIPAddress = modem.getLocalIP();
          modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &tz);
          uint32_t epoch = DateTime(year, month, day, hour, minute, second).unixtime() - (int32_t)(tz * 3600.0f);
          NetTime = DateTime(epoch);
        } else if (mode == 2) {
          devIPAddress = ETH.localIP().toString();
        }
        rtc.adjust(NetTime);
        MODE_CON = mode;
        mqttState = true;
        safePrintln(devIPAddress);
        return true;  // stop jika sukses
      }
    }
  }

  safePrintln("[CONNECTION] ALL CONNECTION FAILED. CONTINUE WITHOUT INTERNET.");
  return false;
}

void resetMQTT() {
  safePrintln("[MQTT] Hard reset MQTT client");

  mqtt.disconnect();  // tutup koneksi MQTT
  delay(50);

  // pastikan socket TCP dilepas
  switch (modeCon) {
    case MODE_GSM:
      GSMclient.stop();  // TinyGSMClient
      break;
    case MODE_WIFI:
    case MODE_ETH:
      netClient.stop();  // WiFiClient (juga untuk ETH)
      break;
  }

  delay(50);
}

void mqttCallback(char *topic, byte *payload, unsigned int len);
void attachMqttTransport() {
  switch (modeCon) {
    case MODE_GSM:
      safePrintln("[MQTT] Using GSM client");
      mqtt.setClient(GSMclient);
      break;
    case MODE_WIFI:
    case MODE_ETH:
      safePrintln("[MQTT] Using NET (WiFi/ETH) client");
      mqtt.setClient(netClient);
      break;
    default:
      safePrintln("[MQTT] No net mode, cannot attach client");
      return;
  }

  mqtt.setServer(MQSERV.c_str(), MQPORT.toInt());
  mqtt.setCallback(mqttCallback);
}

void handleCommand(const String &cmd);
void mqttCallback(char *topic, byte *payload, unsigned int len) {
  String topicStr = String(topic);

  String payloadStr;
  payloadStr.reserve(len + 1);
  for (unsigned int i = 0; i < len; i++) {
    payloadStr += (char)payload[i];
  }

  // Pastikan ini topic command
  if (!topicStr.startsWith(MQSUB)) {
    return;
  }

  safePrint("[MQTT] MESSAGE ARRIVE\t: ");
  safePrintln(payloadStr);
  handleCommand(payloadStr);
  configLoad(false);
}

uint8_t failedMQTT = 0;
bool connectMqtt(String broker, String user, String pass, String subs, String pubs) {
  if (mqtt.connected()) return true;

  resetMQTT();

  safePrint("[MQTT] CONNECTING TO BROKER\t: ");
  bool ok = mqtt.connect(broker.c_str(), user.c_str(), pass.c_str());

  if (!ok) {
    if (mqttState == true) {
      safePrint("FAILED, state=");
      safePrintln(String(mqtt.state()));
      if (failedMQTT >= 10) {
        modeCon += 1;
        mqttState = false;
      }
      failedMQTT++;
    }
    return false;
  }
  String payload;
  DynamicJsonDocument doc(1024);
  doc["type"] = "alert";
  doc["key"] = "device_online";
  safePrintln("CONNECTED");
  mqtt.subscribe(subs.c_str());
  safePrint("[MQTT] Subscribed: ");
  safePrintln(broker + ", Topic : " + subs);
  serializeJson(doc, payload);
  safePrintln("[MQTT] Publish to " + pubs + " : " + payload);
  mqtt.publish(pubs.c_str(), payload.c_str());

  return true;
}

void handleCommand(const String &cmd) {
  String echoPayload;

  DynamicJsonDocument doc(2048);
  DynamicJsonDocument echo(1024);

  // =============================
  // Parse JSON
  // =============================
  DeserializationError error = deserializeJson(doc, cmd);
  if (error) {
    safePrint("[MQTT] JSON parse failed: ");
    safePrintln(error.c_str());

    echo["status"] = "ERROR_PARSING";
    echo["reason"] = error.c_str();
    echo["raw"] = cmd;

    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // ✅ Ambil sebagai String, lalu normalisasi
  String typeS = String((const char *)(doc["type"] | ""));
  String keyS = String((const char *)(doc["key"] | ""));

  typeS.trim();
  keyS.trim();
  typeS.toLowerCase();  // biar "GET" juga kebaca
  // keyS boleh tidak toLowerCase kalau kamu ingin case-sensitive
  // tapi biasanya aman:
  keyS.toLowerCase();

  JsonVariant v = doc["value"];

  // Debug (optional)
  // safePrint("[CMD] type='"); safePrint(typeS); safePrint("' key='"); safePrint(keyS); safePrintln("'");

  // =============================
  // TYPE: GET
  // =============================
  if (typeS == "get") {
    echo["type"] = "get";
    echo["key"] = keyS;
    echo["status"] = "OK";

    if (keyS == "tz_time") {
      echo["value"] = TZ_TIME;
    } else if (keyS == "net") {
      JsonObject out = echo.createNestedObject("value");

      if (DCON == 0) out["mode"] = "wifi";
      else if (DCON == 1) out["mode"] = "gsm";
      else if (DCON == 2) out["mode"] = "eth";
      else out["mode"] = "unknown";

      out["ssid"] = CSSID;
      out["pass_set"] = (CSSIDP.length() > 0);

      out["apn"] = CAPN;
      out["apn_user"] = CAPNU;
      out["apn_pass_set"] = (CAPNP.length() > 0);
    } else if (keyS == "config_server") {
      JsonObject out = echo.createNestedObject("value");
      out["CSERV"] = CSERV;
      out["CDIR"] = CDIR;
      out["CUSER"] = CUSER;
      out["CPORT"] = CPORT;
      out["CPASS_set"] = (CPASS.length() > 0);
    } else if (keyS == "post_server") {
      JsonObject out = echo.createNestedObject("value");
      out["PSERV"] = PSERV;
      out["PDIR"] = PDIR;
      out["PPORT"] = PPORT;
      out["PTOKEN_set"] = (PTOKEN.length() > 0);
    } else if (keyS == "mqtt_config") {
      JsonObject out = echo.createNestedObject("value");
      out["MQSERV"] = MQSERV;
      out["MQUSER"] = MQUSER;
      out["MQPORT"] = MQPORT;
      out["MQPASS_set"] = (MQPASS.length() > 0);
    } else if (keyS == "timer") {
      JsonArray out = echo.createNestedArray("value");
      for (int i = 0; i < 20; i++) out.add(TIMER[i]);
    } else if (keyS == "sensor") {
      JsonArray out = echo.createNestedArray("value");
      for (int i = 0; i < 20; i++) {
        if (SENSOR[i].length() == 0) out.add(nullptr);
        else out.add(SENSOR[i]);
      }
    } else {
      echo["status"] = "ERROR_UNKNOWN_KEY";
    }

    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // =============================
  // TYPE: SET
  // =============================
  if (typeS != "set") {
    echo["status"] = "ERROR_UNKNOWN_TYPE";
    echo["type"] = typeS;  // ✅ kirim yg sudah di-trim biar kebaca problemnya
    echo["key"] = keyS;
    echo["raw"] = cmd;  // ✅ bantu debug

    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  echo["type"] = "set";
  echo["key"] = keyS;

  // =============================
  // SET: STRING
  // =============================
  if (v.is<const char *>()) {
    const char *value = v.as<const char *>();

    echo["status"] = "OK";
    echo["value"] = value;

    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // =============================
  // SET: INT (tz_time)
  // =============================
  if (v.is<int>()) {
    int value = v.as<int>();

    if (keyS == "tz_time") {
      if (value < -12 || value > 14) {
        echo["status"] = "ERROR_TZ_OUT_OF_RANGE";
      } else {
        TZ_TIME = value;

        ConfigData.begin("ConfigData", false);
        ConfigData.putInt("TZ_TIME", value);
        ConfigData.end();

        echo["status"] = "OK";
        echo["applied"] = true;
        echo["value"] = value;
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    echo["status"] = "ERROR_INT_KEY_UNSUPPORTED";
    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // =============================
  // SET: BOOL
  // =============================
  if (v.is<bool>()) {
    bool value = v.as<bool>();
    echo["status"] = "OK";
    echo["value"] = value;

    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // =============================
  // SET: OBJECT
  // =============================
  if (v.is<JsonObject>()) {
    JsonObject obj = v.as<JsonObject>();

    if (keyS == "net") {
      const char *mode = obj["mode"] | "";
      const char *ssid = obj["ssid"] | "";
      const char *pass = obj["pass"] | "";
      const char *apn = obj["apn"] | "";
      const char *apn_user = obj["apn_user"] | "";
      const char *apn_pass = obj["apn_pass"] | "";

      String modeS = String(mode);
      modeS.trim();
      modeS.toLowerCase();

      if (modeS.length() == 0) {
        echo["status"] = "ERROR_NET_MODE_EMPTY";
      } else if (modeS == "wifi") {
        if (strlen(ssid) == 0) echo["status"] = "ERROR_NET_SSID_EMPTY";
        else {
          DCON = 0;
          CSSID = ssid;
          CSSIDP = pass;

          ConfigData.begin("ConfigData", false);
          ConfigData.putInt("DCON", 0);
          ConfigData.putString("CSSID", ssid);
          ConfigData.putString("CSSIDP", pass);
          ConfigData.end();

          echo["status"] = "OK";
          echo["applied"] = true;

          JsonObject out = echo.createNestedObject("value");
          out["mode"] = "wifi";
          out["ssid"] = ssid;
          out["pass_set"] = (strlen(pass) > 0);
        }
      } else if (modeS == "gsm") {
        if (strlen(apn) == 0) echo["status"] = "ERROR_NET_APN_EMPTY";
        else {
          DCON = 1;
          CAPN = apn;
          CAPNU = apn_user;
          CAPNP = apn_pass;

          ConfigData.begin("ConfigData", false);
          ConfigData.putInt("DCON", 1);
          ConfigData.putString("CAPN", apn);
          ConfigData.putString("CAPNU", apn_user);
          ConfigData.putString("CAPNP", apn_pass);
          ConfigData.end();

          echo["status"] = "OK";
          echo["applied"] = true;

          JsonObject out = echo.createNestedObject("value");
          out["mode"] = "gsm";
          out["apn"] = apn;
          out["apn_user"] = apn_user;
          out["apn_pass_set"] = (strlen(apn_pass) > 0);
        }
      } else if (modeS == "eth") {
        echo["status"] = "ERROR_NET_ETH_DISABLED";
      } else {
        echo["status"] = "ERROR_NET_MODE_INVALID";
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    if (keyS == "config_server") {
      const char *h = obj["CSERV"] | "";
      const char *p = obj["CDIR"] | "";
      const char *u = obj["CUSER"] | "";
      const char *pw = obj["CPASS"] | "";
      const char *pt = obj["CPORT"] | "";

      if (strlen(h) == 0) echo["status"] = "ERROR_CSERV_EMPTY";
      else if (strlen(p) == 0) echo["status"] = "ERROR_CDIR_EMPTY";
      else if (strlen(pt) == 0) echo["status"] = "ERROR_CPORT_EMPTY";
      else {
        CSERV = h;
        CDIR = p;
        CUSER = u;
        CPASS = pw;
        CPORT = pt;

        ConfigData.begin("ConfigData", false);
        ConfigData.putString("CSERV", h);
        ConfigData.putString("CDIR", p);
        ConfigData.putString("CUSER", u);
        ConfigData.putString("CPASS", pw);
        ConfigData.putString("CPORT", pt);
        ConfigData.end();

        echo["status"] = "OK";
        echo["applied"] = true;

        JsonObject out = echo.createNestedObject("value");
        out["CSERV"] = h;
        out["CDIR"] = p;
        out["CUSER"] = u;
        out["CPORT"] = pt;
        out["CPASS_set"] = (strlen(pw) > 0);
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    if (keyS == "post_server") {
      const char *h = obj["PSERV"] | "";
      const char *p = obj["PDIR"] | "";
      const char *t = obj["PTOKEN"] | "";
      const char *pt = obj["PPORT"] | "";

      if (strlen(h) == 0) echo["status"] = "ERROR_PSERV_EMPTY";
      else if (strlen(p) == 0) echo["status"] = "ERROR_PDIR_EMPTY";
      else if (strlen(pt) == 0) echo["status"] = "ERROR_PPORT_EMPTY";
      else {
        PSERV = h;
        PDIR = p;
        PTOKEN = t;
        PPORT = pt;

        ConfigData.begin("ConfigData", false);
        ConfigData.putString("PSERV", h);
        ConfigData.putString("PDIR", p);
        ConfigData.putString("PTOKEN", t);
        ConfigData.putString("PPORT", pt);
        ConfigData.end();

        echo["status"] = "OK";
        echo["applied"] = true;

        JsonObject out = echo.createNestedObject("value");
        out["PSERV"] = h;
        out["PDIR"] = p;
        out["PPORT"] = pt;
        out["PTOKEN_set"] = (strlen(t) > 0);
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    if (keyS == "mqtt_config") {
      const char *h = obj["MQSERV"] | "";
      const char *u = obj["MQUSER"] | "";
      const char *pw = obj["MQPASS"] | "";
      const char *pt = obj["MQPORT"] | "";

      if (strlen(h) == 0) echo["status"] = "ERROR_MQSERV_EMPTY";
      else if (strlen(pt) == 0) echo["status"] = "ERROR_MQPORT_EMPTY";
      else {
        MQSERV = h;
        MQUSER = u;
        MQPASS = pw;
        MQPORT = pt;

        ConfigData.begin("ConfigData", false);
        ConfigData.putString("MQSERV", h);
        ConfigData.putString("MQUSER", u);
        ConfigData.putString("MQPASS", pw);
        ConfigData.putString("MQPORT", pt);
        ConfigData.end();

        echo["status"] = "OK";
        echo["applied"] = true;

        JsonObject out = echo.createNestedObject("value");
        out["MQSERV"] = h;
        out["MQUSER"] = u;
        out["MQPORT"] = pt;
        out["MQPASS_set"] = (strlen(pw) > 0);
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    if (keyS == "ota_update") {
      
    }

    echo["status"] = "ERROR_OBJECT_KEY_UNSUPPORTED";
    echo["hint"] = "Only key='net','config_server','post_server','mqtt_config' supports object";
    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  // =============================
  // SET: ARRAY
  // =============================
  if (v.is<JsonArray>()) {
    JsonArray arr = v.as<JsonArray>();

    if (keyS == "timer") {
      if (arr.size() != 20) {
        echo["status"] = "ERROR_TIMER_LENGTH";
        echo["expected"] = 20;
        echo["received"] = arr.size();
        serializeJson(echo, echoPayload);
        mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
        return;
      }

      TimerData.begin("TimerData", false);
      for (int i = 0; i < 20; i++) {
        long val = arr[i] | 0L;
        if (val < 0) val = 0;

        TIMER[i] = val;

        char k[12];
        sprintf(k, "TIMER[%d]", i);
        TimerData.putLong(k, val);
      }
      TimerData.end();

      echo["status"] = "OK";
      echo["applied"] = true;

      JsonArray out = echo.createNestedArray("value");
      for (int i = 0; i < 20; i++) out.add(TIMER[i]);

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    if (keyS == "sensor") {
      if (arr.size() != 20) {
        echo["status"] = "ERROR_SENSOR_LENGTH";
        echo["expected"] = 20;
        echo["received"] = arr.size();
        serializeJson(echo, echoPayload);
        mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
        return;
      }

      SensorData.begin("SensorData", false);

      for (int i = 0; i < 20; i++) {
        char k[12];
        sprintf(k, "SENSOR[%d]", i);

        if (arr[i].isNull()) {
          SENSOR[i] = "";
          SensorData.putString(k, "");
          continue;
        }

        if (!arr[i].is<const char *>()) {
          SensorData.end();
          echo["status"] = "ERROR_SENSOR_TYPE";
          echo["index"] = i;
          serializeJson(echo, echoPayload);
          mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
          return;
        }

        const char *cfg = arr[i].as<const char *>();

        int commaCount = 0;
        for (const char *p = cfg; *p; p++)
          if (*p == ',') commaCount++;
        if (commaCount < 2) {
          SensorData.end();
          echo["status"] = "ERROR_SENSOR_FORMAT";
          echo["index"] = i;
          echo["value"] = cfg;
          serializeJson(echo, echoPayload);
          mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
          return;
        }

        SENSOR[i] = cfg;
        SensorData.putString(k, cfg);
      }

      SensorData.end();

      echo["status"] = "OK";
      echo["applied"] = true;

      JsonArray out = echo.createNestedArray("value");
      for (int i = 0; i < 20; i++) {
        if (SENSOR[i].length() == 0) out.add(nullptr);
        else out.add(SENSOR[i]);
      }

      serializeJson(echo, echoPayload);
      mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
      return;
    }

    echo["status"] = "ERROR_ARRAY_KEY_UNSUPPORTED";
    echo["hint"] = "Only key='timer' or 'sensor' supports array";
    serializeJson(echo, echoPayload);
    mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
    return;
  }

  echo["status"] = "ERROR_VALUE_TYPE";
  serializeJson(echo, echoPayload);
  mqtt.publish(MQPUBM.c_str(), echoPayload.c_str());
}
