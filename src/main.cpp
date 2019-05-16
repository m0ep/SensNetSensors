
#define SEND_DEVICE_INFO 1
#define HAS_BETTERY_MONITORING 0
#define BME280_ENABLED 0
#define GUVA_S12SD_ENABLED 0
#define LOG_ENABLED 0
#define TPL511X_ENABLED 0
#define DHT_ENABLED 1
#define WATER_LEVEL_ENABLED 1
#define LIGHT_LEVEL_ENABLED 0

#include "config.h"

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//#define uS_TO_S_FACTOR 1000000

//int DEEPSLEEP_SECONDS = 3600; // 1h
//int DEEPSLEEP_SECONDS = 1800; // 30 min
//int DEEPSLEEP_SECONDS = 180; // 3 min
//int DEEPSLEEP_SECONDS = 60; // 1 min


#if HAS_BETTERY_MONITORING
  #define BATTERY_PIN A13 // HUZZAH32
#endif

#if BME280_ENABLED
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  #define BME280_I2C_ADDRESS 0x76
  #define BME280_SAMPLES 5
#endif

#if GUVA_S12SD_ENABLED
  #define GUVA_S12SD_PORT A2
  #define GUVA_S12SD_SAMPLES 5
#endif

#if TPL511X_ENABLED
  #define TPL511X_DONE_PIN 12
#endif

#if DHT_ENABLED
  #include <DHT.h>

  #define DHT_TYPE DHT11
  #define DHT_PIN 22

  DHT dht(DHT_PIN, DHT_TYPE);
#endif

#if WATER_LEVEL_ENABLED
  #define WATER_LEVEL_PIN 32
  #define WATER_LEVEL_SAMPLES 5
#endif

#if LIGHT_LEVEL_ENABLED
  #define LIGHT_LEVEL_PIN 33
  #define LIGHT_LEVEL_SAMPLES 5
#endif

#if LOG_ENABLED
  #define LOG_PRINTLN(value) Serial.println(value);
  #define LOG_PRINT(value) Serial.print(value);
  #define LOG_PRINTF(pattern, ...) Serial.printf(pattern, __VA_ARGS__);
#else
  #define LOG_PRINTLN(value)
  #define LOG_PRINT(value)
  #define LOG_PRINTF(pattern, ...)
#endif

//#define LED_PIN LED_BUILTIN //Huzzah32
#define LED_PIN 16 // HiGrow


#define DEVICE_INFO_TOPIC "sensnet/device/%s/info"
#define SENSOR_DATA_TOPIC "sensnet/sensor/%s/data"

WiFiClientSecure tlsClient;
PubSubClient mqttClient( tlsClient);

#if BME280_ENABLED
  Adafruit_BME280 bme;
#endif

char deviceId[20];
char jsonMessage[128];
char topic[64];

bool wifiConnect() {
  LOG_PRINTF("Attempting to connect to '%s'", WIFI_SSID)

  unsigned long wifiConnectStart = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    LOG_PRINT(".")
    delay(1000);

    if(10000 < millis() - wifiConnectStart){
      return false;
    }
  }

  LOG_PRINTLN("Connected")
  return true;
}

bool mqttConnect(){
  tlsClient.setCACert(MQTT_HOST_ROOT_CA);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  uint8_t tries = 0;
  while(!mqttClient.connected()){
    LOG_PRINTF("Attempting to connect to server '%s'...", MQTT_HOST)
    if(mqttClient.connect(deviceId, MQTT_USER, MQTT_PASSWORD)){
      LOG_PRINTLN("Connected.")
    } else {
      LOG_PRINTF("ERROR %d\n", mqttClient.state())
      tries++;
      if(5 <= tries){
        LOG_PRINTLN("give up")
        return false;
      }

      delay(5000);
    }
  }

  return true;
}

#if SEND_DEVICE_INFO
void sendDeviceInfo(){
  const int capacity = JSON_OBJECT_SIZE(3);
  
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  #if HAS_BETTERY_MONITORING
    root["bat"] =  float(analogRead(BATTERY_PIN)) / 4095.0f * 3.30f * 2.0f * 1.02f;
  #else  
    root["bat"] = 0.0f;
  #endif

  serializeJson(root, jsonMessage, 128);
  
  sprintf(topic, DEVICE_INFO_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)
  
  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

#if BME280_ENABLED
void sendBme280SensorData() {
  if(!bme.begin(BME280_I2C_ADDRESS)){
    LOG_PRINT("Failed to open BME280")
    return;
  }

  // calculate average over BME280_SAMPLES samples to get better results
  long temperature = 0;
  long pressure = 0;
  long humidity = 0;
  for(int i = 0; i < BME280_SAMPLES; i++){
    temperature += long(bme.readTemperature() * 1000.0f);
    pressure += long(bme.readPressure() * 1000.0f);
    humidity += long(bme.readHumidity() * 1000.0f);
    delay(2);
  }

  const int capacity = JSON_OBJECT_SIZE(6);
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  root["knd"] = "tph";
  root["t"] = (float(temperature / BME280_SAMPLES) / 1000.0f);
  root["p"] = (float(pressure / BME280_SAMPLES) / 1000.0f);
  root["h"] = (float(humidity / BME280_SAMPLES) / 1000.0f);
  serializeJson(root, jsonMessage, 128);

  sprintf(topic, SENSOR_DATA_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)

  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

#if GUVA_S12SD_ENABLED
void sendGuvaS12sdSensorData(){

  // calculate average over GUVA_S12SD_SAMPLES samples to get better results
  uint16_t sum = 0;
  for(int i = 0; i < GUVA_S12SD_SAMPLES; i++) {
    sum += analogRead(GUVA_S12SD_PORT);
    delay(2);
  }
  uint16_t sensor_value_average = sum / GUVA_S12SD_SAMPLES;
  float volatage = float(sensor_value_average) / 4095.0f * 3.3f * 1.1f;
  int uvIndex = int(volatage / 0.1f);

  LOG_PRINTF("avg=%d mV=%f index=%d\n", sensor_value_average, volatage, uvIndex)

  const int capacity = JSON_OBJECT_SIZE(5);
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  root["knd"] = "uvi";
  root["uvi"] = uvIndex;
  root["umv"] = volatage;
  serializeJson(root, jsonMessage, 128);

  sprintf(topic, SENSOR_DATA_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)

  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

#if DHT_ENABLED
void sendDhtSensorData() {
  dht.begin();

  long temperature = dht.readTemperature();
  long humidity = dht.readHumidity();

  const int capacity = JSON_OBJECT_SIZE(5);
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  root["knd"] = "th";
  root["t"] = temperature;
  root["h"] = humidity;
  serializeJson(root, jsonMessage, 128);

  sprintf(topic, SENSOR_DATA_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)

  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

#if WATER_LEVEL_ENABLED
void sendWaterLevelSensorData() {
  uint16_t sum = 0;
  for(int i = 0; i < WATER_LEVEL_SAMPLES; i++) {
    sum += analogRead(WATER_LEVEL_PIN);
    delay(2);
  }
  uint16_t sensor_value_average = sum / WATER_LEVEL_SAMPLES;
  float waterLevel = map(sensor_value_average, 0, 4095, 0, 1023);
  waterLevel = constrain(waterLevel, 0, 1023);
  if (isnan(waterLevel)) {
    return;
  }

  const int capacity = JSON_OBJECT_SIZE(4);
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  root["knd"] = "wtl";
  root["lvl"] = waterLevel;
  serializeJson(root, jsonMessage, 128);

  sprintf(topic, SENSOR_DATA_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)

  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

#if LIGHT_LEVEL_ENABLED
void sendLightLevelSensorData() {
  uint16_t sum = 0;
  for(int i = 0; i < LIGHT_LEVEL_SAMPLES; i++) {
    sum += analogRead(LIGHT_LEVEL_PIN);
    delay(2);
  }
  uint16_t sensor_value_average = sum / LIGHT_LEVEL_SAMPLES;

  float lightLevel = map(sensor_value_average, 0, 4095, 0, 1023);
  lightLevel = constrain(lightLevel, 0, 1023);
  if (isnan(lightLevel)) {
    return;
  }

  const int capacity = JSON_OBJECT_SIZE(4);
  StaticJsonDocument<capacity> root;
  root["did"] = deviceId;
  root["knd"] = "ltl";
  root["lvl"] = lightLevel;
  serializeJson(root, jsonMessage, 128);

  sprintf(topic, SENSOR_DATA_TOPIC, deviceId);
  LOG_PRINTF("%s -> %s\n", topic, jsonMessage)

  if(!mqttClient.publish(topic,jsonMessage)){
    LOG_PRINTLN("Failed to send message")
  }
}
#endif

void esp32Sleep(){
  #if TPL511X_ENABLED
    for (int i = 0; i < 10; i++) {
      digitalWrite(TPL511X_DONE_PIN, HIGH);
      delay(100);
      digitalWrite(TPL511X_DONE_PIN, LOW);
      delay(100);
    }
  #else
    esp_sleep_enable_timer_wakeup(3600000000);
    //esp_sleep_enable_timer_wakeup(DEEPSLEEP_SECONDS * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  #endif
}

#if LOG_ENABLED
void logAnalogPins() {
  for(int i = 12; i <= 39;i++){
    pinMode(i, INPUT);
    float value = analogRead(i);
    LOG_PRINTF("Pin %d -> %f\n", i, value);
  }
}
#endif

void setup() {
  delay(500);

  #if LOG_ENABLED
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    
    // logAnalogPins();
  #endif


  #if TPL511X_ENABLED
    pinMode(TPL511X_DONE_PIN, OUTPUT);
    digitalWrite(TPL511X_DONE_PIN, LOW);
    delay(100);
  #endif

  uint64_t chipid = ESP.getEfuseMac();

  sprintf(deviceId, "ESP32-%04X",(uint16_t)(chipid>>32));
  LOG_PRINTF("ESP32 MQTT-ID =%s\n",deviceId)

  if(!wifiConnect()){
    esp32Sleep();
    return;
  }

  if(!mqttConnect()){
    esp32Sleep();
    return;
  }

  #if SEND_DEVICE_INFO
  sendDeviceInfo();
  #endif

  #if BME280_ENABLED
    sendBme280SensorData();
  #endif

  #if GUVA_S12SD_ENABLED
    sendGuvaS12sdSensorData();
  #endif

  #if DHT_ENABLED
    sendDhtSensorData();
  #endif

  #if WATER_LEVEL_ENABLED
    sendWaterLevelSensorData();
  #endif
  
  #if LIGHT_LEVEL_ENABLED
    sendLightLevelSensorData();
  #endif

  mqttClient.disconnect();
  WiFi.disconnect();

  esp32Sleep();
}

void loop() {
  #if TPL511X_ENABLED
    digitalWrite(TPL511X_DONE_PIN, HIGH);
    delay(5000);
    digitalWrite(TPL511X_DONE_PIN, LOW);
    delay(5000);
  #endif
}
