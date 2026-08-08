/***********************************************************************
 * Water‑Quality Logger – ESP32‑WROOM‑32 (30 pins)     (rev‑C: GPS demo)
 *  ▸ Sensors  : Turbidity, TDS, DO, pH, ORP (Grove), DS18B20
 *  ▸ GPS      : TinyGPS++ on UART2 (pins 16/17, 9600 baud)
 *  ▸ Outputs  : SD‑card  |  Firebase RTDB  |  Serial table
 *  ▸ Flags    : USE_SD, USE_FIREBASE, USE_SERIAL
 ***********************************************************************/

// ── USER FLAGS ─────────────────────────────────────────────────────────
#define USE_SD 1
#define USE_FIREBASE 1
#define USE_SERIAL 1

// ── Wi‑Fi / Firebase ──────────────────────────────────────────────────
const char *WIFI_SSID = "";
const char *WIFI_PASS = "";
#if USE_FIREBASE
#define FB_API_KEY ""
#define FB_DATABASE_URL ""
#endif

// ── Logging interval ──────────────────────────────────────────────────
const unsigned long LOG_INTERVAL_MS = 20'000;
unsigned long lastLog = 0;

// ── Analog pins ───────────────────────────────────────────────────────
const int PIN_TURBIDITY = 36;
const int PIN_TDS = 39;
const int PIN_DO = 34;
const int PIN_ORP = 35;
const int PIN_ORP_CAL = 32;
const int PIN_PH = 33;

// DS18B20
const int PIN_DS18B20 = 4;

// SD
#if USE_SD
#include <SD.h>
#include <SPI.h>
const int SD_CS_PIN = 5;
File logFile;
#endif

// ── GPS (identical to your demo) ──────────────────────────────────────
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
const unsigned GPS_READ_MS = 1000;  // same 1‑second window

// ── DS18B20 ───────────────────────────────────────────────────────────
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

// ── Wi‑Fi + NTP ───────────────────────────────────────────────────────
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60'000);  // IST GMT+5:30

// ── Firebase ──────────────────────────────────────────────────────────
#if USE_FIREBASE
#include <Firebase_ESP_Client.h>
#include "addons/RTDBHelper.h"
#include "addons/TokenHelper.h"
FirebaseData fbdo;
FirebaseAuth fbAuth;
FirebaseConfig fbConfig;
#endif

// ── Other sensor calibration factors ──────────────────────────────────
const float TURBIDITY_K = 1.0;
const float TDS_K = 1.0;
const float DO_K = 1.0;
const float PH_OFFSET = 0.0;

/*───────────────────────  D O   M e t e r  ────────────────────────────
 *  • Supports 1‑ or 2‑point calibration (set DO_TWO_POINT_CAL ↑)
 *  • Non‑blocking: tick() every loop, getDO() whenever you need the value
 */
class DOMeter {
public:
  void begin(uint8_t pin, float vref = 3.3,
             bool twoPoint = false,  // set true for 2‑point
             uint16_t cal1_mv = 131, uint8_t cal1_t = 25,
             uint16_t cal2_mv = 1300, uint8_t cal2_t = 15) {
    _pin = pin;
    _vref = vref;
    pinMode(_pin, INPUT);
    _twoPoint = twoPoint;
    _cal1_mv = cal1_mv;
    _cal1_t = cal1_t;
    _cal2_mv = cal2_mv;
    _cal2_t = cal2_t;
  }

  void tick(float waterTempC)  // call every loop()
  {
    _temp = (uint8_t)waterTempC;
    if (millis() - _tSample >= 40) {  // 25 Hz buffer fill
      _tSample = millis();
      _buf[_idx++] = analogRead(_pin);
      if (_idx >= SCOUNT) _idx = 0;
    }

    if (millis() - _tCompute >= 800) {  // update value ~1×/s
      _tCompute = millis();
      memcpy(_tmp, _buf, sizeof(_buf));
      uint16_t adc = median(_tmp);
      uint32_t mv = (uint32_t)(_vref * 1000) * adc / 4095;  // mV

      uint16_t Vsat;
      if (!_twoPoint) {  // 1‑point
        Vsat = _cal1_mv + 35 * _temp - 35 * _cal1_t;
      } else {  // 2‑point
        Vsat = (int16_t)(_temp - _cal2_t) * (_cal1_mv - _cal2_mv)
                 / (_cal1_t - _cal2_t)
               + _cal2_mv;
      }
      _do = mv * DO_Table[_temp] / Vsat / 1000.0f;  // mg/L
    }
  }

  float getDO() const {
    return _do;
  }

private:
  static const uint8_t SCOUNT = 30;
  uint8_t _pin;
  float _vref;
  bool _twoPoint;
  uint16_t _cal1_mv, _cal2_mv;
  uint8_t _cal1_t, _cal2_t;

  int _buf[SCOUNT] = { 0 }, _tmp[SCOUNT];
  uint8_t _idx = 0;
  uint32_t _tSample = 0, _tCompute = 0;
  uint8_t _temp = 25;
  float _do = 0;

  /* median of 30 ints */
  static uint16_t median(int *a) {
    for (int i = 0; i < SCOUNT - 1; i++)
      for (int j = 0; j < SCOUNT - i - 1; j++)
        if (a[j] > a[j + 1]) std::swap(a[j], a[j + 1]);
    return (SCOUNT & 1) ? a[(SCOUNT - 1) / 2]
                        : (a[SCOUNT / 2] + a[SCOUNT / 2 - 1]) / 2;
  }

  /* same saturation table as original sketch (mg/L *1000) */
  static const uint16_t DO_Table[41];
};
/* table stored outside the class */
const uint16_t DOMeter::DO_Table[41] PROGMEM = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};


/*─────────────────────────  T D S   S e n s o r  ───────────────────────
 *  – Median‑filter on a 30‑sample rolling buffer (40 ms per sample)
 *  – Temperature compensation using current DS18B20 value
 *  – getTDS() is non‑blocking; call every loop()   */
class TdsMeter {
public:
  void begin(uint8_t pin, float vref = 3.3) {
    _pin = pin;
    _vref = vref;
    pinMode(_pin, INPUT);
  }

  void tick(float waterTempC) {  // call every loop
    unsigned long now = millis();

    /* 1) 25 Hz raw sampling into ring‑buffer */
    if (now - _tSample >= 40) {  // 40 ms
      _tSample = now;
      _buf[_idx++] = analogRead(_pin);
      if (_idx >= SCOUNT) _idx = 0;
    }

    /* 2) Every 800 ms calculate TDS */
    if (now - _tCompute >= 800) {
      _tCompute = now;
      // copy to temp array for median sort
      memcpy(_tmp, _buf, sizeof(_buf));
      _avgVolt = median(_tmp) * _vref / 4096.0;

      float compCoeff = 1.0 + 0.02 * (waterTempC - 25.0);
      float compVoltage = _avgVolt / compCoeff;

      _tds = (133.42 * pow(compVoltage, 3)
              - 255.86 * pow(compVoltage, 2)
              + 857.39 * compVoltage)
             * 0.5;  // ppm
    }
  }

  float getTDS() const {
    return _tds;
  }
  float getVoltage() const {
    return _avgVolt;
  }

private:
  static const uint8_t SCOUNT = 30;
  uint8_t _pin;
  float _vref = 3.3;
  int _buf[SCOUNT] = { 0 };
  int _tmp[SCOUNT];
  uint8_t _idx = 0;
  float _avgVolt = 0;
  float _tds = 0;
  uint32_t _tSample = 0;
  uint32_t _tCompute = 0;

  /* median of 30 ints */
  static float median(int *a) {
    for (int i = 0; i < SCOUNT - 1; i++)
      for (int j = 0; j < SCOUNT - i - 1; j++)
        if (a[j] > a[j + 1]) std::swap(a[j], a[j + 1]);
    return (SCOUNT & 1)
             ? a[(SCOUNT - 1) / 2]
             : (a[SCOUNT / 2] + a[SCOUNT / 2 - 1]) / 2.0;
  }
};

// ── Grove ORP helper (unchanged from rev‑B) ───────────────────────────
class GroveORP {
public:
  void begin(uint8_t sig, uint8_t cal) {
    _sig = sig;
    _cal = cal;
    pinMode(_cal, OUTPUT);
    digitalWrite(_cal, HIGH);
  }
  void tick() {
    _buf[_idx++] = analogRead(_sig);
    if (_idx >= LEN) _idx = 0;
  }
  int read() {
    if (!_done && millis() - _t0 > START_MS) {
      _offset = calc();
      _done = true;
      digitalWrite(_cal, LOW);
    }
    return calc() - _offset;
  }
private:
  static const int LEN = 40;
  static const int VOLT = 3370;
  static const int START_MS = 5000;
  uint8_t _sig, _cal;
  int _buf[LEN] = { 0 };
  uint8_t _idx = 0;
  int _offset = 0;
  bool _done = false;
  unsigned _t0 = millis();
  int calc() {
    long sum = 0;
    int minv = 4095, maxv = 0;
    for (int v : _buf) {
      sum += v;
      if (v < minv) minv = v;
      if (v > maxv) maxv = v;
    }
    sum -= minv + maxv;
    float avg = sum / float(LEN - 2);
    return ((30 * VOLT) - (75 * avg * VOLT / 1024.0)) / 75;
  }
};
GroveORP orp;

// ── Serial header ─────────────────────────────────────────────────────
#if USE_SERIAL
void printHeader() {
  Serial.println(F("\nTime                | Turb | TDS  | DO   | ORP | pH  | °C  | Lat      | Lon       | km/h | Alt | HDOP | Sat"));
  Serial.println(F("--------------------+------+-----+------+-----+-----+-----+----------+-----------+------+-----+------+----"));
}
#endif

// SD line
#if USE_SD
void logToSD(const String &csv) {
  logFile = SD.open("/waterlog.csv", FILE_APPEND);
  if (logFile) {
    logFile.println(csv);
    logFile.close();
  }
}
#endif

// Firebase push
#if USE_FIREBASE
void pushFirebase(const String &ts, float turb, float tds, float dO, int orp_mV, float ph, float tempC,
                  double lat, double lon, float spd, float alt, float hdop, int sats) {
  FirebaseJson j;
  j.set("time", ts);
  j.set("turbidity", turb);
  j.set("tds", tds);
  j.set("dissolvedO2", dO);
  j.set("orp", orp_mV);
  j.set("pH", ph);
  j.set("tempC", tempC);
  j.set("lat", lat);
  j.set("lon", lon);
  j.set("speedKmph", spd);
  j.set("altM", alt);
  j.set("hdop", hdop);
  j.set("sat", sats);
  Firebase.RTDB.pushJSON(&fbdo, "/waterLogs", &j);
}
#endif

TdsMeter tdsMeter;
DOMeter doMeter;


// ── setup ─────────────────────────────────────────────────────────────
void setup() {
#if USE_SERIAL
  Serial.begin(115200);
  while (!Serial)
    ;
#endif
  tdsMeter.begin(PIN_TDS, 3.3);  // Pin 27, 3.3 V reference
  doMeter.begin(/*pin=*/PIN_DO, /*vref=*/3.3, /*2‑pt?*/ false);
  //            ^ use same GPIO 34 you already wired

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(300);
  timeClient.begin();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  ds18b20.begin();
  orp.begin(PIN_ORP, PIN_ORP_CAL);

#if USE_SD
  bool isSDDone = SD.begin(SD_CS_PIN);
  Serial.print("SD Card Initialized: ");
  Serial.println(isSDDone);
  delay(isSDDone ? 1000 : 5000);
#endif
#if USE_FIREBASE
  #if USE_FIREBASE
  fbConfig.api_key      = FB_API_KEY;
  fbConfig.database_url = FB_DATABASE_URL;
  fbConfig.token_status_callback = tokenStatusCallback;

  // ① get a valid token (anonymous sign‑up)
  if (Firebase.signUp(&fbConfig, &fbAuth, "", ""))
      Serial.println("Firebase sign‑up OK");
  else
      Serial.printf("Sign‑up failed ➜ %s\n",
                    fbConfig.signer.signupError.message.c_str());

  // ② start client
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
#endif

#endif

#if USE_SERIAL
  printHeader();
#endif
}

// ── tiny helper ───────────────────────────────────────────────────────
float readAnalog(int pin, float k = 1.0) {
  const int N = 10;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) {
    acc += analogRead(pin);
    delay(2);
  }
  return (acc / float(N)) * (3.3 / 4095.0) * k;
}

// ── loop ──────────────────────────────────────────────────────────────
void loop() {
  timeClient.update();
  orp.tick();  // frequent ORP sampling

  // ── read GPS for a full second (like your demo) ────────────────────
  unsigned long start = millis();
  while (millis() - start < GPS_READ_MS) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  }

  unsigned long now = millis();
  if (now - lastLog < LOG_INTERVAL_MS) return;
  lastLog = now;

  // ── gather sensor data ─────────────────────────────────────────────
  float turbidity = map(analogRead(PIN_TURBIDITY), 0, 2800, 5, 1);
  ;
  float tds = tdsMeter.getTDS();        // ppm (already temp‑compensated)
  float dissolvedO2 = doMeter.getDO();  // mg / L
  int orp_mV = orp.read();
  float voltage = analogRead(PIN_PH) * (3.3 / 4095.0);
  float pH_raw = (3.3 * voltage);
  float pH = pH_raw;

  ds18b20.requestTemperatures();
  float tempC = ds18b20.getTempCByIndex(0);
  tdsMeter.tick(tempC);
  doMeter.tick(tempC);  // non‑blocking DO update

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  float spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  float alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  float hdop = gps.hdop.isValid() ? gps.hdop.value() / 100.0 : 0.0;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // ── timestamp ──────────────────────────────────────────────────────
  char buf[25];
  time_t epoch = timeClient.getEpochTime();
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", localtime(&epoch));
  String ts = buf;

// ── Serial table ───────────────────────────────────────────────────
#if USE_SERIAL
  Serial.printf("%-19s | %4.1f | %4.1f | %4.1f | %4d | %4.2f | %4.1f | %.5f | %.5f | %4.1f | %4.0f | %4.1f | %2d\n",
                buf, turbidity, tds, dissolvedO2, orp_mV, pH, tempC,
                lat, lon, spd, alt, hdop, sats);
#endif

// ── SD CSV ─────────────────────────────────────────────────────────
#if USE_SD
  String csv = ts + "," + String(turbidity, 1) + "," + String(tds, 1) + "," + String(dissolvedO2, 1) + "," + String(orp_mV) + "," + String(pH, 2) + "," + String(tempC, 2) + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(spd, 1) + "," + String(alt, 0) + "," + String(hdop, 1) + "," + String(sats);
  logToSD(csv);
#endif

// ── Firebase ───────────────────────────────────────────────────────
#if USE_FIREBASE
  pushFirebase(ts, turbidity, tds, dissolvedO2, orp_mV, pH, tempC,
               lat, lon, spd, alt, hdop, sats);
#endif
}
