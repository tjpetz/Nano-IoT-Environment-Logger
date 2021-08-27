/**
 * Read environment measurements from off board sensors and log them to
 * my mqtt service.
 *
 * Features:
 *  Low power - sleep for extended periods of time.
 *  BME280 - temp, pressure, humidity sensor (at present)
 *  BLE configurable WiFi
 *  Configuration settings in flash memory
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ArduinoMqttClient.h>
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <time.h>
#include <Adafruit_SleepyDog.h>
// #include <Adafruit_BME280.h>
#include <Adafruit_SHTC3.h>

// #include "ConfigService.h"
// #include "PagingOLEDDisplay.h"
#include "secret.h"

#define _DEBUG_
#include "Debug.h"

#undef max
#undef min
#include <map>

#define DEFAULT_HOSTNAME "iot_nano_001"
#define DEFAULT_MQTTBROKER "mqtt.bb.tjpetz.com"
#define DEFAULT_MQTTROOTTOPIC "tjpetz.com/sensor"
#define DEFAULT_SAMPLEINTERVAL 60
#define DEFAULT_LOCATION "dinning room"

#define BATTERY_SENSE A0

struct {
  char ssid[64];
  char wifiPassword[64];
  char mqttBroker[128];
  char hostName[128];
  char topicRoot[128];
  char location[128];
} config;

Adafruit_SHTC3 envSensor;


// We will not attempt connections to peripherals with a less then -90 dBm RSSI.
#define RSSI_LIMIT -95

WiFiClient wifiClient;             // Our wifi client
MqttClient mqttClient(wifiClient); // Our MQTT client
const int mqttTxDelay = 2500; // Time (mS) between the last call to mqtt and turning off the wifi.
const int port = 1883;
const unsigned int MAX_TOPIC_LENGTH = 255;
byte macAddress[6]; // MAC address of the Wifi which we'll use in reporting

RTCZero rtc; // Real Time Clock so we can time stamp data
unsigned long lastRTCSync;

const int maxTries = 10; // Retry counters
int retryCounter = 0;    // counter to keep track of failed connections
bool success = false;

const int watchdogTimeout = 16384;    // max timeout is 16.384 S


/** @brief Update the RTC by calling NTP, requires WiFi to be available */
bool updateRTC() {
  int maxTries = 10;
  int tries = 0;

  if (WiFi.status() != WL_CONNECTED) {  
    DEBUG_PRINTF("updateRTC: WiFi is not connected.\n")
    return false;
  }

  Watchdog.reset();
  time_t ntpTime = WiFi.getTime();

  // try a few times to get the time, it takes a moment to reach the NTP
  // servers.
  while (ntpTime == 0 && tries < maxTries) {
    Watchdog.reset();
    ntpTime = WiFi.getTime();
    tries++;
    delay(500 * (tries + 1));
  }

  if (ntpTime == 0) {
    DEBUG_PRINTF("updateRTC: Failed to contact the NTP Server, not updating the RTC.\n");
    return false;
  }

  struct tm *t = gmtime(&ntpTime); // convert Unix epoch time to tm struct format
  DEBUG_PRINTF("NTP Time = %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900,
               t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

  rtc.setEpoch(ntpTime);
  DEBUG_PRINTF("RTC Time = %04d-%02d-%02d %02d:%02d:%02d\n", rtc.getYear(),
               rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(),
               rtc.getSeconds());

  lastRTCSync = millis();

  return true;
}

/** @brief Connect to wifi with retries, force reboot if unsuccessful */
bool connectWiFi() {
  int maxTries = 10;
  int tries = 0;

  while ((WiFi.begin(config.ssid, config.wifiPassword) != WL_CONNECTED) &&
         (tries < maxTries)) {
    DEBUG_PRINTF("connectWiFi(): Connection failed, reason = %d.\n",
                 WiFi.reasonCode());
    tries++;
    WiFi.disconnect();
    WiFi.end();
    Watchdog.reset();
    delay(1500 * tries);
  }

  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTF("Failed to connect to wifi after %d tries.  Rebooting!\n",
                 tries);
    Serial.flush();
    delay(500);
    NVIC_SystemReset();
  }

  return true;
}

/** @brief Disconnect and end WiFi.  This is necessary before using BLE. */
void disconnectWiFi() {
  WiFi.disconnect();
  WiFi.end();
}

/** @brief Initialize the RTC by calling NTP and setting the initial time in the RTC */
void initializeRTC() {

  connectWiFi();
  updateRTC();

  // Record the boot
  Watchdog.reset();
  if (!mqttClient.connect(config.mqttBroker, port)) {
    DEBUG_PRINTF("Error Connecting to mqtt broker\n");
  } else {
    char topic[MAX_TOPIC_LENGTH];
    snprintf(topic, sizeof(topic), "%s/%s/boot", config.topicRoot,
             config.hostName);
    DEBUG_PRINTF("Topic = %s\n", topic);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "{ \"boot\": \"%04d-%02d-%02dT%02d:%02d:%02d\", \"IP\": "
             "\"%d.%d.%d.%d\", \"rssi\": %ld, \"reset_reason\": %0x }",
             rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(),
             rtc.getMinutes(), rtc.getSeconds(), WiFi.localIP()[0],
             WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3],
             WiFi.RSSI(), PM->RCAUSE.reg);
    DEBUG_PRINTF("Sending msg = %s\n", msg);
    mqttClient.beginMessage(topic);
    mqttClient.print(msg);
    mqttClient.endMessage();
    delay(mqttTxDelay);
    mqttClient.stop();
    DEBUG_PRINTF("Sent message\n");
  }
  Watchdog.reset();
  disconnectWiFi();
}

/** @brief send the measurements via mqtt
 *  @return true if success */
bool sendMeasurementsToMQTT(float temperature, float humidity, // float pressure, int battery,
  const char* name, const char *location) {
  
  bool status = false;

  DEBUG_PRINTF("Attempting to send measurement\n");
  Watchdog.reset();

  char topic[MAX_TOPIC_LENGTH];
  snprintf(topic, sizeof(topic), "%s/%s/environment", config.topicRoot,
            name);
  DEBUG_PRINTF("Topic = %s\n", topic);

  mqttClient.beginMessage(topic);
  char dateTime[32];
  snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02dT%02d:%02d:%02d",
            rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(),
            rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTF("Sample Time = %s\n", dateTime);
  char msg[255];
  // snprintf(
  //     msg, sizeof(msg),
  //     "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", "
  //     "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f, \"battery\": %d }",
  //     name, location, dateTime, temperature, humidity, pressure, battery);
  snprintf(
      msg, sizeof(msg),
      "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", "
      "\"temperature\": %.2f, \"humidity\": %.2f }",
      name, location, dateTime, temperature, humidity);
  DEBUG_PRINTF("Message = %s\n", msg);
  mqttClient.print(msg);
  mqttClient.endMessage();
  status = true;
  return status;
}

// void onCentralConnected(BLEDevice central) {
//   DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n",
//                central.address().c_str(), central.rssi(), millis());
//   DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
// }

// void onCentralDisconnected(BLEDevice central) {
//   DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(),
//                millis());
//   DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
// }


void setup() {

  // Initialize the configuration using dummies
  strncpy(config.ssid, SECRET_SSID, sizeof(config.ssid));
  strncpy(config.wifiPassword, SECRET_PASSWORD, sizeof(config.wifiPassword));
  strncpy(config.mqttBroker, DEFAULT_MQTTBROKER, sizeof(config.mqttBroker));
  strncpy(config.hostName, DEFAULT_HOSTNAME, sizeof(config.hostName));
  strncpy(config.topicRoot, DEFAULT_MQTTROOTTOPIC, sizeof(config.topicRoot));
  strncpy(config.location, DEFAULT_LOCATION, sizeof(config.location));

  Serial.begin(115200);
  delay(3000); // Give serial a moment to start

  DEBUG_PRINTF("RESET Register = 0x%0x\n", PM->RCAUSE.reg);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print("Battery = "); Serial.println(analogRead(A0));
  Watchdog.enable(watchdogTimeout);

  // Start the sensors and devices
  rtc.begin();
  envSensor.begin();

  initializeRTC();

  digitalWrite(LED_BUILTIN, LOW);
  Watchdog.reset();

}

/** @brief sleep for a long duration of time, turning off peripherals to save power
*/
void longDeepSleep(unsigned long sleepDuration_ms) {

  Serial.print("Beginning long sleep of ");
  Serial.print(sleepDuration_ms);
  Serial.println(" ms");
  Serial.flush(); 
  USBDevice.detach();

  const int maxSleepDuration = 16000;
  if (sleepDuration_ms <= maxSleepDuration) {
    // We can sleep in 1 interval
    Watchdog.sleep(sleepDuration_ms);
  } else {
    // Sleep in 16 second intervals and briefly wake up repeatedly until we're done sleeping.
    while (sleepDuration_ms > 0) {
      auto sleptFor = Watchdog.sleep(sleepDuration_ms <= maxSleepDuration ? sleepDuration_ms : maxSleepDuration);
      sleepDuration_ms -= sleptFor;
    }
  }

  USBDevice.attach();   // Reattach the USB device after we wake up
  delay(1500);
  Serial.println("Waking from a long sleep!");
}

void loop() {

  Watchdog.reset();  
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);

  // auto temp = envSensor.readTemperature();
  // auto humidity = envSensor.readHumidity();
  // auto pressure = envSensor.readPressure() / 100.0 * 0.0145037738;

  sensors_event_t temp, humidity;
  envSensor.getEvent(&humidity, &temp);

  connectWiFi();
  mqttClient.connect(config.mqttBroker, port);
  // sendMeasurementsToMQTT(temp, humidity, pressure, analogRead(BATTERY_SENSE), config.hostName, config.location);    
  sendMeasurementsToMQTT(temp.temperature, humidity.relative_humidity, config.hostName, config.location);    
  mqttClient.flush();
  delay(3000);
  mqttClient.stop();
  disconnectWiFi();

  Watchdog.reset();
  longDeepSleep(15 * 60 * 1000);

}
