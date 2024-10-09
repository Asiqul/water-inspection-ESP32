#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <time.h>
#include <GravityTDS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "ADC_TABLE.h"

// TASKS HANDLES SETUP
TaskHandle_t TDSPHTemperature;
TaskHandle_t NTUPHDatabase;
// TASKS HANDLES SETUP END

// WIFI CONFIG
const char *WIFI_SSID = "IoT_CONNECTION";
const char *WIFI_PASSWORD = "11111111";
// WIFI CONFIG END

// FIREBASE CONFIG
const char *FIREBASE_API_KEY = "AIzaSyB6tJgppL7vOyeY_fLqzuwjvPMlGr-GxQE";
const char *FIREBASE_PROJECT_ID = "water-inspection";
const char *FIREBASE_DATABASE_URL = "https://water-inspection-default-rtdb.firebaseio.com/";
// FIREBASE CONFIG END

// FIREBASE CREDENTIALS
#define USER_EMAIL "esp32firebase@gmail.com"
#define USER_PASSWORD "esp32firebase123"
// FIREBASE CREDENTIALS END

// PIN DEFINITION SENSOR
#define TEMP_PIN 25
#define TDS_PIN 35
#define PH_PIN 34
#define NTU_PIN 33

#define WiFiLED 16
#define errWiFiLED 17
#define sensorLED 18
#define errSensorLED 19
#define databaseLED 21
#define errDatabaseLED 22
// PIN DEFINITION SENSOR END

// FIREBASE SETUP
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
// FIREBASE SETUP END

// SENSOR DATA STORAGE AND GLOBAL VARIABLES
float TDSvalue;
float Temp;
float PHvalue;
float NTUvalue;
int DateTime;
String dataStreamPath = "/DATASTREAM";
const char *ntpServer = "id.pool.ntp.org";
// SENSOR DATA STORAGE AND GLOBAL VARIABLES END

// SENSOR SETUP
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
GravityTDS gravityTds;
// SENSOR SETUP END

void TDS_PH_Temperature(void *pvParameters);
void NTU_Database(void *pvParameters);
int getADC(int pin);
int getDate();
float round_to_dp(float in_value, int decimal_place);

void setup()
{
  // GENERAL SETUP
  Serial.begin(115200);
  EEPROM.begin(32);
  pinMode(TEMP_PIN, INPUT_PULLUP);
  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(NTU_PIN, INPUT);
  pinMode(WiFiLED, OUTPUT);
  pinMode(errWiFiLED, OUTPUT);
  pinMode(sensorLED, OUTPUT);
  pinMode(errSensorLED, OUTPUT);
  pinMode(databaseLED, OUTPUT);
  pinMode(errDatabaseLED, OUTPUT);
  analogReadResolution(12);
  // GENERAL SETUP END

  // LED STATUS CONTROL
  digitalWrite(WiFiLED, LOW);
  digitalWrite(errWiFiLED, HIGH);
  digitalWrite(sensorLED, LOW);
  digitalWrite(errSensorLED, HIGH);
  digitalWrite(databaseLED, LOW);
  digitalWrite(errDatabaseLED, HIGH);
  // LED STATUS CONTROL END

  // BEGINNING WIFI SETUP
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(WiFiLED, LOW);
    digitalWrite(errWiFiLED, HIGH);
    Serial.println("Connecting to WiFi..");
    delay(500);
  }

  digitalWrite(WiFiLED, HIGH);
  digitalWrite(errWiFiLED, LOW);
  Serial.println("----------------");
  Serial.print("Connected with local IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("----------------");
  // BEGINNING WIFI SETUP END

  configTime(7 * 3600, 0, ntpServer);

  // BEGINNING SENSOR SETUP
  sensors.begin();
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  // BEGINNING FIREBASE SETUP
  config.api_key = FIREBASE_API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = FIREBASE_DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  Firebase.begin(&config, &auth);
  // BEGINNING FIREBASE SETUP END

  while (!Firebase.ready())
  {
    digitalWrite(databaseLED, LOW);
    digitalWrite(errDatabaseLED, HIGH);
    Serial.println("Firebase connection error.. Retrying...");
    delay(500);
  }

  DateTime = getDate();

  // MULTITASKING SETUP
  xTaskCreatePinnedToCore(
      TDS_PH_Temperature,
      "TDS_PH_Temperature",
      15000,
      NULL,
      1,
      &TDSPHTemperature,
      0);
  vTaskDelay(1500);
  xTaskCreatePinnedToCore(
      NTU_Database,
      "NTU_Database",
      15000,
      NULL,
      1,
      &NTUPHDatabase,
      1);
  vTaskDelay(500);
}

void loop()
{
}

// TASK FUNCTION
void TDS_PH_Temperature(void *pvParameters)
{
  float PH35 = 3.256;
  float PH7 = 2.311;
  float PH_Step;

  while (true)
  {
    // GET TEMPERATURE
    sensors.requestTemperatures();
    Temp = sensors.getTempCByIndex(0);

    if (Temp != DEVICE_DISCONNECTED_C)
    {
      digitalWrite(sensorLED, HIGH);
      digitalWrite(errSensorLED, LOW);

      // GET TDS
      gravityTds.setTemperature(Temp);
      gravityTds.update();
      TDSvalue = gravityTds.getTdsValue() * 1.8;

      // GET pH
      float PH_Voltage = 0;

      for (int i = 0; i < 800; i++)
      {
        PH_Voltage += ((float)getADC(PH_PIN) * (3.3 / 4096.0));
      }

      PH_Voltage = PH_Voltage / 800;
      PH_Step = (PH35 - PH7) / 3.5;
      PHvalue = 7.00 + ((PH7 - PH_Voltage) / PH_Step);

      delay(1000);
    }
    else
    {
      // ERROR
      digitalWrite(errSensorLED, HIGH);
      digitalWrite(sensorLED, LOW);
      Serial.println("Error: Could not read temperature data");
      delay(1000);
    }
  }
}

void NTU_Database(void *pvParameters)
{
  while (true)
  {
    // GET NTU
    float NTUvoltage = 0;
    for (int i = 0; i < 800; i++)
    {
      NTUvoltage += ((float)getADC(NTU_PIN) * 5.752) / 4096.0;
    }
    NTUvoltage = NTUvoltage / 800;
    NTUvoltage = round_to_dp(NTUvoltage, 3);

    if (NTUvoltage < 1.650)
    {
      NTUvalue = 3000;
    }
    else if (NTUvoltage > 2.772)
    {
      NTUvalue = 0;
    }
    else
    {
      NTUvalue = -2572.2 * sq(NTUvoltage) + 8700.5 * NTUvoltage - 4352.9;
    }

    NTUvalue = round(NTUvalue);
    Serial.println(NTUvalue);
    Serial.println(PHvalue);
    Serial.println(TDSvalue);
    Serial.println(Temp);
    delay(500);

    if (Firebase.ready())
    {
      // JSON DATA SET
      json.set("DateTime", DateTime);
      json.set("pH_value", round_to_dp(PHvalue, 2));
      json.set("TDS_value", round(int(TDSvalue)));
      json.set("Temperature", round_to_dp(Temp, 2));
      json.set("NTU_value", round(int(NTUvalue)));
      // JSON DATA SET END

      // SEND DATA TO FIREBASE
      Firebase.RTDB.setJSON(&fbdo, dataStreamPath, &json);
      Serial.println("Data sent to database...");
      Serial.print("Status code: ");
      Serial.println(fbdo.httpCode());
      digitalWrite(databaseLED, HIGH);
      digitalWrite(errDatabaseLED, LOW);

      while (fbdo.errorCode() != 200)
      {
        digitalWrite(errDatabaseLED, HIGH);
        digitalWrite(databaseLED, LOW);
        vTaskSuspend(TDSPHTemperature);
        Serial.println("Error sending data to database...");
        Serial.print("Error code: ");
        Serial.println(fbdo.errorCode());
        Firebase.RTDB.setJSON(&fbdo, dataStreamPath, &json);
        Serial.println("Resend data to database");
        delay(500);
      }
      digitalWrite(databaseLED, HIGH);
      digitalWrite(errDatabaseLED, LOW);
      vTaskResume(TDSPHTemperature);
      // SEND DATA TO FIREBASE END
    }
    else
    {
      digitalWrite(errDatabaseLED, HIGH);
      digitalWrite(databaseLED, LOW);
      Serial.println("Please put sensor probe to water.");
      delay(500);
    }

    delay(1000);
  }
}
// TASK FUNCTION END

// GET DATE FUNCTION
int getDate()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return 0;
  }
  time(&now);
  return now;
}
// GET DATE FUNCTION END

// ADC CALIBRATION FUNCTION
int getADC(int pin)
{
  int _analogValue = analogRead(pin);
  int _adcCalibration = int(ADC_LUT[_analogValue]);
  return _adcCalibration;
}
// ADC CALIBRATION FUNCTION END

// ROUND TO DECIMAL PLACE FUNCTION
float round_to_dp(float in_value, int decimal_place)
{
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}
// ROUND TO DECIMAL PLACE FUNCTION END