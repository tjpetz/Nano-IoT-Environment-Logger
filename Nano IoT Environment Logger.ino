/**
 * Read environment measurements from off board sensors and log them to
 * my mqtt service.
 *
 * Features:
 *  Low power - sleep for extended periods of time.
 *  BLE configurable WiFi
 *  Configuration settings in flash memory
 *  Currently supports the following environmental sensors
 *    BME280 - Temp, Humidity, Pressure
 *    SHTC3 - Temp, Humidity
 *    ADT7410 - Temp
 *  Battery level reporting
 * 
 *  BLE Configuration
 *
 *  After reset the system will start in an initializing state.  In this state BLE is enabled
 *  and configuration setting changes can be made.  After 1 minutes if BLE is not connected 
 *  the system will transition to the running state. 
 *
 *  Sensor Selection
 *    define one of SENSOR_TYPE_BME280, SENSOR_TYPE_SHTC3, SENSOR_TYPE_ADT7410
 *  
 *  Battery measurement
 *    define BATTERY_SENSE as the analog pin to measure the battery voltage with
 */

/** Begin configuration defines */
// Define SECRET_SSID, SECRET_PASSWORD, and SECRET_MQTTBROKER in the secret.h file.
#define DEFAULT_SSID SECRET_SSID
#define DEFAULT_PASSWORD SECRET_PASSWORD
#define DEFAULT_MQTTBROKER SECRET_MQTTBROKER
#define DEFAULT_HOSTNAME "iot_nano_001"
#define DEFAULT_MQTTROOTTOPIC "tjpetz.com/sensor"
#define DEFAULT_SAMPLEINTERVAL 60
#define DEFAULT_LOCATION "unknown"

// Define only one of the following for the sensor type connected to the Nano.
#define SENSOR_TYPE_BME280
// #define SENSOR_TYPE_SHTC3
// #define SENSOR_TYPE_ADT7410

// Define only if measuring the voltage of the battery
#define BATTERY_SENSE A0

#define _DEBUG_
/** End configuration defines */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ArduinoMqttClient.h>
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <time.h>
#include <Adafruit_SleepyDog.h>

#ifdef SENSOR_TYPE_BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 envSensor;
#endif
#ifdef SENSOR_TYPE_SHTC3
#include <Adafruit_SHTC3.h>
Adafruit_SHTC3 envSensor;
#endif
#ifdef SENSOR_TYPE_ADT7410
#include <Adafruit_ADT7410.h>
Adafruit_ADT7410 envSensor;
#endif

#include "secret.h"
#include "ConfigService.h"
#include "Debug.h"

ConfigService config(DEFAULT_SSID, DEFAULT_PASSWORD,
           DEFAULT_HOSTNAME, DEFAULT_LOCATION, 
           DEFAULT_MQTTBROKER, DEFAULT_MQTTROOTTOPIC,
           DEFAULT_SAMPLEINTERVAL);

// We will not attempt connections to peripherals with a less then -90 dBm RSSI.
#define RSSI_LIMIT -95

WiFiClient wifiClient;             // Our wifi client
MqttClient mqttClient(wifiClient); // Our MQTT client
const int mqttTxDelay = 2500; // Time (mS) between the last call to mqtt and turning off the wifi.
const int port = 1883;
const unsigned int MAX_TOPIC_LENGTH = 255;
byte macAddress[6]; // MAC address of the Wifi which we'll use in reporting

RTCZero rtc; // Real Time Clock so we can time stamp data
uint32_t lastRTCSync;     // Track the last time we synced the clock
const uint32_t resyncClock = 8 * 60 * 60;    // Resynch the clock to NTP every 8 hours.

const int maxTries = 10; // Retry counters
int retryCounter = 0;    // counter to keep track of failed connections
bool success = false;

const int watchdogTimeout = 16384;    // max timeout is 16.384 S

enum {
  initializing = 0,
  running  
} currentState;

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

  lastRTCSync = rtc.getEpoch();

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

void onCentralConnected(BLEDevice central) {
  DEBUG_PRINTF("Connection from: %s, rssi = %d, at %lu\n",
               central.address().c_str(), central.rssi(), millis());
  DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
}

void onCentralDisconnected(BLEDevice central) {
  DEBUG_PRINTF("Disconnected from: %s, at %lu\n", central.address().c_str(),
               millis());
  DEBUG_PRINTF("  BLE Central = %s\n", BLE.central().address().c_str());
}

/** @brief sleep for a long duration of time, turning off peripherals to save power
*/
void longDeepSleep(unsigned long sleepDuration_ms) {

  DEBUG_PRINTF("Beginning long sleep of %d ms\n", sleepDuration_ms);
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
  DEBUG_PRINTF("Waking from a long sleep!\n");
}

void setup() {

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

  digitalWrite(LED_BUILTIN, LOW);
  Watchdog.reset();

  config.debug_print_configuration();
  
  DEBUG_PRINTF("Initializing BLE\n");
  BLE.begin(); 
  BLE.setLocalName(config.hostName);
  config.begin();
  BLE.setAdvertisedService(config.getConfigService());
  BLE.setEventHandler(BLEConnected, onCentralConnected);
  BLE.setEventHandler(BLEDisconnected, onCentralDisconnected);
  BLE.advertise();
  DEBUG_PRINTF("BLE Initialized\n");

  currentState = initializing;
}

void loop() {

  Watchdog.reset();  
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);

  switch (currentState) {
    case initializing:
      if ((millis() < (1 * 60 * 1000)) || BLE.connected()) {
        BLE.poll(500);
      } else {
        DEBUG_PRINTF("Exiting initialization\n");
        config.debug_print_configuration();
        BLE.disconnect();
        BLE.end();
        delay(1750);      // Allow the Nina radio to reset
        Watchdog.reset();
        initializeRTC();
        currentState = running;
      }
      break;

    case running:    
      char topic[MAX_TOPIC_LENGTH], dateTime[32], msg[255], sensor_msg[200];
      snprintf(topic, sizeof(topic), "%s/%s/environment", config.topicRoot, config.hostName);
      snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02dT%02d:%02d:%02d",
                rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(),
                rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

      connectWiFi();

      // Update the RTC with the current time every resynchClock seconds.  Note use RTC because millis() does not advance while in deep sleep
      if ((lastRTCSync + resyncClock) <= rtc.getEpoch()) {
        updateRTC();        
      }
            
      mqttClient.connect(config.mqttBroker, port);
      mqttClient.beginMessage(topic);

#ifdef SENSOR_TYPE_BME280
      auto temp = envSensor.readTemperature();
      auto humidity = envSensor.readHumidity();
      auto pressure = envSensor.readPressure() / 100.0 * 0.0145037738;
      snprintf(sensor_msg, sizeof(sensor_msg), "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f", temp, humidity, pressure);
#endif

#ifdef SENSOR_TYPE_SHTC3
      sensors_event_t temp, humidity;
      envSensor.getEvent(&humidity, &temp);
      snprintf(sensor_msg, sizeof(sensor_msg), "\"temperature\": %.2f, \"humidity\": %.2f", temp.temperature, humidity.relative_humidity);
#endif

#ifdef SENSOR_TYPE_ADT7410
      auto temp = envSensor.readTempC();
      snprintf(sensor_msg, sizeof(sensor_msg), "\"temperature\": %.2f", temp);
#endif

#ifdef BATTERY_SENSE
  snprintf(
      msg, sizeof(msg),
      "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", "
      "%s, \"battery\": %d }",
      config.hostName, config.location, dateTime, sensor_msg, map(analogRead(BATTERY_SENSE), 465, 651, 0, 100));
#else
  snprintf(
      msg, sizeof(msg),
      "{ \"sensor\": \"%s\", \"location\": \"%s\", \"sampleTime\": \"%s\", "
      "%s}",
      config.hostName, config.location, dateTime, sensor_msg);
#endif

      DEBUG_PRINTF("Message = %s\n", msg);
      mqttClient.print(msg);
      mqttClient.endMessage();
      mqttClient.flush();
      delay(3000);
      mqttClient.stop();
      disconnectWiFi();

      Watchdog.reset();
      longDeepSleep(config.sampleInterval * 1000);
      break;
    }

}
