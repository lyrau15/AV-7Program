#pragma once

#include <FFat.h>
#include <esp_system.h>   //untuk mengambil uniq id alat
#include <Preferences.h>  //Library penyimpanan data Konfigurasi
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"

Preferences ConfigData;  //Set Object Konfigurasi
Preferences SensorData;  //Set Object Konfigurasi Sensor
Preferences TimerData;

String IDSTATION;                               //Untuk get ID Station
String DAPN, DAPNU, DAPNP;                      //Untuk get Parameter Default APN
String CAPN, CAPNU, CAPNP;                      //Untuk get Parameter Custom APN
String DSSID, DSSIDP;                           //Untuk get Parameter Default SSID
String CSSID, CSSIDP;                           //Untuk get Parameter Custom SSID
int DCON;                                       //Untuk get Default koneksi prioritas
String MSERV, MDIR, MPORT;                      //Untuk get Master HTTPS Server
String CSERV, CDIR, CUSER, CPASS, CPORT;        //Untuk get Config HTTPS Server
String PSERV, PDIR, PTOKEN, PPORT;              //Untuk get POST HTTPS Server
String MQSERV, MQUSER, MQPASS, MQPORT;          //Untuk get MQTT Config
String MQSUB, MQPUBD, MQPUBM, TZ_TIME, MQBUFF;  //Untuk get MQTT Topic
long TIMER[20];
String SENSOR[20];

// {
//   "type": "set",
//   "key": "apn",
//   "value": "ethernet"
// }
void safePrint(const String& msg);
void safePrintln(const String& msg);
size_t safePrintf(const char* format, ...);

String getUniqueDeviceCode() {          //Get String unique device code
  uint64_t chipid = ESP.getEfuseMac();  // 64-bit unique ID

  // Format jadi 12 hex (mirip MAC, tanpa titik dua)
  char id[17];  // 12 hex + null
  sprintf(id, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);

  return String(id);  // contoh: "3F0A1BCD2345"
}

void resetSensorConfig() {  //!!!PERHATIAN!!! *HANYA GUNAKAN BILA INGIN MENGOSONGKAN CONFIG
  safePrint("[SENSOR] RESETTING SENSOR CONFIG DATA\t: ");



  safePrintln("DONE!");
}

bool configLoad(bool reset) {  //kasih true bila ingin reset data internal kembali ke dafault
  if (reset == true) {
    safePrint("[CONFIG] Load Default Config Data\t: ");
    ConfigData.begin("ConfigData", false);
    ConfigData.clear();
    ConfigData.putString("IDSTATION", "SPKU");                                        //ID STATION
    ConfigData.putString("DAPN", "internet");                                         //Default APN
    ConfigData.putString("DAPNU", "");                                                //Default APN User
    ConfigData.putString("DAPNP", "");                                                //Default APN Password
    ConfigData.putString("CAPN", "internet");                                         //Custom APN
    ConfigData.putString("CAPNU", "");                                                //Custom APN User
    ConfigData.putString("CAPNP", "");                                                //Custom APN Pass
    ConfigData.putString("DSSID", "Hotspot New");                                     //Default SSID
    ConfigData.putString("DSSIDP", "admin123");                                       //Default SSID Password
    ConfigData.putString("CSSID", "AMMD56RCS <2>");                                   //Default SSID Password
    ConfigData.putString("CSSIDP", "NgoPiDiD56");                                     //Default SSID Password
    ConfigData.putInt("DCON", 0);                                                     //Default Connection 0=WiFi, 1=GSM, 2=Ethernet
    ConfigData.putString("MSERV", "samxyz.my.id");                                    //Server Utama untuk update data
    ConfigData.putString("MDIR", "/sparing/testSparing.php");                         //Directory Server Utama
    ConfigData.putString("MPORT", "443");                                             //Port Server utama
    ConfigData.putString("CSERV", "ispu.kemenlh.go.id");                              //Server HTTPS Get Data Authentifikasi
    ConfigData.putString("CDIR", "/api/v1/auth");                                     //HTTPS Get Data Authentifikasi
    ConfigData.putString("CUSER", "pt_rekayasa_cahaya_solusi");                       //Username Authentifikasi
    ConfigData.putString("CPASS", "d5R5Mh&6VVmHkmjg");                                //Password Authentifikasi
    ConfigData.putString("CPORT", "443");                                             //PORT Authentifikasi
    ConfigData.putString("PSERV", "ispu.kemenlh.go.id");                              //Server HTTPS Post Data
    ConfigData.putString("PDIR", "/api/v1/aqmdata");                                  //Dir HTTPS Post Data
    ConfigData.putString("PTOKEN", "A");                                              //BEARER TOKEN HTTPS Post DatA
    ConfigData.putString("PPORT", "443");                                             //PORT Authentifikasi
    ConfigData.putString("MQSERV", "103.94.238.40");                                  //MQTT Server
    ConfigData.putString("MQUSER", "siresin");                                        //MQTT User
    ConfigData.putString("MQPASS", "s1res1npw");                                      //MQTT Pass
    ConfigData.putString("MQPORT", "1883");                                           //MQTT Port
    ConfigData.putString("MQSUB", "rcs/av7/" + getUniqueDeviceCode() + "/cmd");    //MQTT Subscribe topic
    ConfigData.putString("MQPUBD", "rcs/av7/" + getUniqueDeviceCode() + "/data");  //MQTT Publish for Data
    ConfigData.putString("MQPUBM", "rcs/av7/" + getUniqueDeviceCode() + "/msg");   //MQTT Publish for cmd
    ConfigData.putString("TZ_TIME", "7");                                             //Set timezone default
    ConfigData.end();
    delay(200);

    TimerData.begin("TimerData", false);
    TimerData.clear();
    for (int i = 0; i < 20; i++) {
      TimerData.putLong(("TIMER[" + String(i) + "]").c_str(), 1);
    }
    TimerData.putLong("TIMER[1]", 60);
    TimerData.end();
    delay(200);

    SensorData.begin("SensorData", false);
    SensorData.clear();
    for (int i = 0; i < 20; i++) {
      SensorData.putString(("SENSOR[" + String(i) + "]").c_str(), "null");
    }
    SensorData.end();
    delay(200);
  } else {
    safePrint("[CONFIG] Load Saved Config Data\t: ");
  }

  ConfigData.begin("ConfigData", true);
  ConfigData.clear();
  IDSTATION = ConfigData.getString("IDSTATION", "");
  DAPN = ConfigData.getString("DAPN", "");
  DAPNU = ConfigData.getString("DAPNU", "");
  DAPNP = ConfigData.getString("DAPNP", "");
  CAPN = ConfigData.getString("CAPN", "");
  CAPNU = ConfigData.getString("CAPNU", "");
  CAPNP = ConfigData.getString("CAPNP", "");
  DSSID = ConfigData.getString("DSSID", "");
  DSSIDP = ConfigData.getString("DSSIDP", "");
  CSSID = ConfigData.getString("CSSID", "");
  CSSIDP = ConfigData.getString("CSSIDP", "");
  DCON = ConfigData.getInt("DCON", 0);
  MSERV = ConfigData.getString("MSERV", "");
  MDIR = ConfigData.getString("MDIR", "");
  MPORT = ConfigData.getString("MPORT", "");
  CSERV = ConfigData.getString("CSERV", "");
  CDIR = ConfigData.getString("CDIR", "");
  CUSER = ConfigData.getString("CUSER", "");
  CPASS = ConfigData.getString("CPASS", "");
  CPORT = ConfigData.getString("CPORT", "");
  PSERV = ConfigData.getString("PSERV", "");
  PDIR = ConfigData.getString("PDIR", "");
  PTOKEN = ConfigData.getString("PTOKEN", "");
  PPORT = ConfigData.getString("PPORT", "");
  MQSERV = ConfigData.getString("MQSERV", "");
  MQUSER = ConfigData.getString("MQUSER", "");
  MQPASS = ConfigData.getString("MQPASS", "");
  MQPORT = ConfigData.getString("MQPORT", "");
  MQSUB = ConfigData.getString("MQSUB", "");
  MQPUBD = ConfigData.getString("MQPUBD", "");
  MQPUBM = ConfigData.getString("MQPUBM", "");
  TZ_TIME = ConfigData.getString("TZ_TIME", "7");
  ConfigData.end();

  TimerData.begin("TimerData", true);
  TimerData.clear();
  for (int i = 0; i < 20; i++) {
    TIMER[i] = TimerData.getLong(("TIMER[" + String(i) + "]").c_str(), 0);
  }
  TimerData.end();

  SensorData.begin("SensorData", true);
  SensorData.clear();
  for(int i=0; i<20; i++){
    SENSOR[i] = SensorData.getString(("SENSOR[" + String(i) + "]").c_str(), "");
  }
  SensorData.end();
  
  safePrintln("SUCCESS");
  return true;
}

void printConfig() {
  safePrintln("===== CONFIGURATION =====");

  safePrintln("IDSTATION        : " + IDSTATION);

  safePrintln("DAPN             : " + DAPN);
  safePrintln("DAPNU            : " + DAPNU);
  safePrintln("DAPNP            : " + DAPNP);

  safePrintln("CAPN             : " + CAPN);
  safePrintln("CAPNU            : " + CAPNU);
  safePrintln("CAPNP            : " + CAPNP);

  safePrintln("DSSID            : " + DSSID);
  safePrintln("DSSIDP           : " + DSSIDP);

  safePrintln("CSSID            : " + CSSID);
  safePrintln("CSSIDP           : " + CSSIDP);

  safePrintln("DCON             : " + String(DCON));

  safePrintln("MSERV            : " + MSERV);
  safePrintln("MDIR             : " + MDIR);
  safePrintln("MPORT            : " + MPORT);

  safePrintln("CSERV            : " + CSERV);
  safePrintln("CDIR             : " + CDIR);
  safePrintln("CUSER            : " + CUSER);
  safePrintln("CPASS            : " + CPASS);
  safePrintln("CPORT            : " + CPORT);

  safePrintln("PSERV            : " + PSERV);
  safePrintln("PDIR             : " + PDIR);
  safePrintln("PTOKEN           : " + PTOKEN);
  safePrintln("PPORT            : " + PPORT);

  safePrintln("MQSERV           : " + MQSERV);
  safePrintln("MQUSER           : " + MQUSER);
  safePrintln("MQPASS           : " + MQPASS);
  safePrintln("MQPORT           : " + MQPORT);

  safePrintln("MQSUB            : " + MQSUB);
  safePrintln("MQPUBD           : " + MQPUBD);
  safePrintln("MQPUBM           : " + MQPUBM);

  safePrintln("TZ_TIME          : " + TZ_TIME);

  safePrintln("----- TIMER -----");
  for (int i = 0; i < 20; i++) {
    safePrintln("TIMER[" + String(i) + "]        : " + TIMER[i]);
  }

  safePrintln("----- SENSOR -----");
  for (int i = 0; i < 20; i++) {
    safePrintln("SENSOR[" + String(i) + "]       : " + SENSOR[i]);
  }

  safePrintln("=============================");
}