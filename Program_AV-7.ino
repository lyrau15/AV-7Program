#define BOARD_TYPE "PM2.5 MAINBOARD V1.4"
#define BOARD_SERIAL ""
#define FIRMWARE_VERSION "1.0.0"  // Versi Firmware *Jangan lupa update setiap ada perubahan yang di publish

//Get Other program Tab
#include <Wire.h>
#include <Update.h>           // library OTA ESP32
#include "Storage.h"          // Mengambil Storage.h
#include "esp_task_wdt.h"     // Mengambil untuk Watchdog Timer
#include "Communication.h"    // Mengambil Communication.h
#include "Data_Collection.h"  // Mengambil Data_Collection.h
#include "freertos/task.h"

// Editable Variable
#define WDT_TIME 60000  // Define WDT untuk 1 menit hang

RTC_PCF8563 rtc;
DateTime NetTime;

String hari, tanggal, bulan, tahun, jam, menit, detik;

TaskHandle_t networkTaskHandle = nullptr;
SemaphoreHandle_t serialMutex;

unsigned long LastTime[20];


float pm10 = 0, pm25 = 0, so2 = 0, co = 0, o3 = 0, no2 = 0, hc = 0, ws = 0, wd = 0, pm10_ppm = 0, pm25_ppm = 0, so2_ppm = 0, co_ppm = 0, o3_ppm = 0, no2_ppm = 0, hc_ppm = 0;
int stat_pm10 = 0, stat_pm25 = 0, stat_so2 = 0, stat_co = 0, stat_o3 = 0, stat_no2 = 0, stat_hc = 0;
float humidity = 0, temperature = 0, pressure = 0, sr = 0, rain_intensity = 0, FlowPM = 0;


void setup() {
  analogReadResolution(12);  //Set Resolusi Read Analog In ke 12 bit, atau 4096
  Serial.begin(115200);
  delay(2000);

  serialMutex = xSemaphoreCreateMutex();  //Untuk SafePrint

  // Memulai Program
  safePrintln("======= PROGRAM BEGIN =======");
  safePrintln(String("[SETUP] VERSION \t\t: ") + FIRMWARE_VERSION);
  safePrintln(String("[SETUP] DEVICE ID\t\t: ") + getUniqueDeviceCode());

  Serial1.begin(9600, SERIAL_8N1, 35, 36, false, 1024);

  // Matikan config default (kalau ada) lalu set baru
  esp_task_wdt_deinit();

  esp_task_wdt_config_t WDT_CONFIG = {
    .timeout_ms = WDT_TIME,  // 60 detik
    .idle_core_mask = 0,     // 0 = JANGAN awasi IDLE0/IDLE1 -> biar nggak error "IDLE0"
    .trigger_panic = true    // kalau timeout, print backtrace + reset
  };
  esp_err_t err = esp_task_wdt_init(&WDT_CONFIG);
  if (err != ESP_OK) {
    safePrintln("[WDT] INIT FAILED");
    ESP.restart();
  } else {
    safePrintln("[WDT] INIT OK");
  }
  esp_task_wdt_add(NULL);

  configLoad(false);
  // printConfig();

  // Inisialisasi Pin I2C
  Wire.begin(SDA_I2C, SCL_I2C);
  //RTC Startup
  if (rtc.begin()) {
    safePrint("[RTC] RTC\t\t: ");
    rtc.start();
    delay(500);
    DateTime rtcTime = rtc.now();
    safePrintf("%02d-%02d-%04d %02d:%02d:%02d\n", rtcTime.day(), rtcTime.month(), rtcTime.year(), rtcTime.hour(), rtcTime.minute(), rtcTime.second());
  } else {
    safePrint("[RTC] RTC\t\t: ERROR!");
    ESP.restart();
  }

  if (!FFat.begin(true)) {
    safePrintln("[STORAGE] FFat mount failed");
    ESP.restart();
  } else {
    safePrintln("[STORAGE] FFat mount success!");
  }


  safePrintln("\n=====START RUNTIME=====");
  //Add Task di Core 0 untuk handle program
  xTaskCreatePinnedToCore(
    NetworkTask,         // function task
    "NetworkTask",       // nama task
    8192,                // stack size
    NULL,                // parameter
    1,                   // priority
    &networkTaskHandle,  // handle (bisa NULL kalau ga butuh)
    0                    // <-- CORE 0
  );
  if (networkTaskHandle != nullptr) {
    esp_task_wdt_add(networkTaskHandle);
  }
}

void loop() {
  //Update Every Loop
  getRTCParameter();

  if ((UnixTime - LastTime[0]) >= TIMER[0]) {
    //PRINT TIME NOW
    safePrintln(tanggal + "/" + bulan + "/" + tahun + " " + jam + ":" + menit + ":" + detik);
    generateRandomSensorData();
    LastTime[0] = UnixTime;
  }

  if ((UnixTime - LastTime[1]) >= TIMER[1]) {
    for (int i = 0; i < 20; i++) {
      // String cfg = SENSOR[i];
      // cfg.trim();

      // // ✅ skip kalau kosong atau "null"
      // if (cfg.length() == 0) continue;
      // if (cfg.equalsIgnoreCase("null")) continue;

      // // ✅ validasi format harus ada 2 koma
      // int p1 = cfg.indexOf(',');
      // int p2 = cfg.indexOf(',', p1 + 1);
      // if (p1 < 0 || p2 < 0) {
      //   safePrintln("[SENSOR] INVALID FORMAT idx=" + String(i) + " cfg='" + cfg + "'");
      //   continue;
      // }

      // String sensorType = cfg.substring(0, p1);
      // int baudrate = cfg.substring(p1 + 1, p2).toInt();
      // int address = cfg.substring(p2 + 1).toInt();

      // uint32_t t0 = millis();
      // bool ok = readSensor(sensorType, baudrate, address);
      // uint32_t dt = millis() - t0;

      // printReadResult(sensorType, baudrate, address, ok, i, dt);
    }

    LastTime[1] = UnixTime;
  }


  // readNextion();

  // // "Page 3 confirmed" hanya jika baru terlihat dalam 2 detik terakhir
  // bool page3Confirmed = (lastPageIdSeen == 3) && (millis() - lastPageSeenMs < 2000);

  // // Timeout kalau numeric gak dibalas-balas
  // if (currentNumericTarget != NUM_NONE && millis() - lastNumericRequest > NUMERIC_TIMEOUT) {
  //   Serial.print("Timeout waiting numeric, reset target from ");
  //   Serial.println((int)currentNumericTarget);

  //   currentNumericTarget = NUM_NONE;
  //   nxTimeoutStreak++;

  //   // kalau timeout beruntun, paksa resync supaya dapet page id bener
  //   if (nxTimeoutStreak >= 2 && millis() - lastForceSendme > 300) {
  //     Serial.println("[NX] Force resync: sendme");
  //     sendCommand("sendme");
  //     lastForceSendme = millis();
  //   }
  // }

  // // Mode baca setting (setelah save.val==1): baca n0..n4 satu per satu, retry
  // if (page3Confirmed && readingSettings && currentNumericTarget == NUM_NONE && millis() - lastNxRequest > NX_INTERVAL) {
  //   int nextIdx = -1;
  //   for (int i = 0; i < 5; i++) {
  //     if (!nValid[i]) {
  //       nextIdx = i;
  //       break;
  //     }
  //   }

  //   if (nextIdx >= 0) {
  //     Serial.print("Next invalid index: ");
  //     Serial.println(nextIdx);
  //     requestNx(nextIdx);
  //     lastNxRequest = millis();
  //   } else {
  //     Serial.println("Semua n0..n4 valid, simpan ke NXcfg[] & reset save");
  //     for (int i = 0; i < 5; i++) NXcfg[i] = nVals[i];

  //     // reset tombol save (lebih aman eksplisit)
  //     sendCommand("save.val=0");
  //     delay(CMD_DELAY_MS);
  //     sendCommand("vis p1,0");
  //     readingSettings = false;
  //   }
  // }

  // // Poll read/save bergantian hanya saat page 3 TERKONFIRMASI dan bukan sedang baca setting
  // if (page3Confirmed && !readingSettings && currentNumericTarget == NUM_NONE && millis() - lastPoll > POLL_INTERVAL) {
  //   lastPoll = millis();

  //   if (pollReadNext) {
  //     requestRead();
  //     pollReadNext = false;
  //   } else {
  //     requestSave();
  //     pollReadNext = true;
  //   }
  // }

  // // sendme untuk deteksi page:
  // // - tetap jalan, tapi jangan terlalu cepat
  // // - saat readingSettings, lebih lambat
  // unsigned long sendmeInterval = readingSettings ? 1500 : 700;
  // if (currentNumericTarget == NUM_NONE && millis() - lastRequest > sendmeInterval) {
  //   lastRequest = millis();
  //   // jangan spam; cukup untuk memastikan pageId selalu ke-update
  //   sendCommand("sendme");
  // }
  esp_task_wdt_reset();  //buffer untuk mereset wdt
}

void NetworkTask(void *pvParameters) {
  (void)pvParameters;


  modeCon = DCON;

  SIMSerial.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  mqtt.setBufferSize(1024);  // PubSubClient dari Communication.h :contentReference[oaicite:2]{index=2}

  for (;;) {

    // ===== PERIODIC CONNECTION CHECK =====
    if (UnixTime - LastTime[10] >= TIMER[10]) {
      if (!mqtt.connected()) {
        if (connectToInternet(modeCon)) {
          attachMqttTransport();
          connectMqtt(getUniqueDeviceCode().c_str(), MQUSER, MQPASS, MQSUB, MQPUBM);
        }
      }

      LastTime[10] = UnixTime;
    }

    // // ===== MQTT ONLINE =====
    if (mqtt.connected()) {
      mqtt.loop();
    }

    esp_task_wdt_reset();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void adjustRTC(int mode) {
  if (mode == 0) {
    time_t epoch = time(nullptr);
    rtc.adjust(DateTime(epoch));
  } else {
  }
}

void safePrint(const String &msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);  // lock
  Serial.print(msg);
  xSemaphoreGive(serialMutex);  // unlock
}

void safePrintln(const String &msg) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);  // lock
  Serial.println(msg);
  xSemaphoreGive(serialMutex);  // unlock
}

size_t safePrintf(const char *format, ...) {
  xSemaphoreTake(serialMutex, portMAX_DELAY);

  char buffer[256];
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);

  size_t ret = Serial.print(buffer);

  xSemaphoreGive(serialMutex);
  return ret;
}