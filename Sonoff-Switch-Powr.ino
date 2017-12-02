/*
   Sonoff-Touch-TeHu
   Firmware to use a Sonoff TH10/16 device as a remote relay and temp/humidity sensor with MQTT and WiFi capabilities.

   Supports OTA update
   Mats Ekberg (C) 2017 GNU GPL v3

   Supports sensors (has to be recompiled if change):
     DHT 11
     DHT 21  (AM2301)  This is the default sensor type
     DHT 22  (AM2302)

   Runs on this harware:
   https://www.itead.cc/wiki/Sonoff_TH_10/16

   Uses these libraries:
   https://github.com/adafruit/DHT-sensor-library
   https://github.com/adafruit/Adafruit_Sensor

   Flashed via USB/OTA in Arduino IDE with these parameters:
   Board:       Generic ESP8266 Module
   Flash size:  1M (64K SPIFFS)

*/

// DO EDIT
#define CONFIG_VERSION "POWR001"
#define POW
// END - DO EDIT


// DO NOT CHANGE
#include "sensorlibs.h"
#include "support/wifi-manager.h"
#include "support/mqtt-support.h"

#include "topics.h"
#include "support/wifi-manager.cpp"
#include "support/mqtt-support.cpp"
// END - DO NOT CHANGE



// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed
#define CURRENT_MODE                    HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

HLW8012 hlw8012;

float current = NAN;
float voltage = NAN;
float power = NAN;
float calPower = 60.0;
float energy = NAN;  // kWh
unsigned long lastEnergySample = 0;

boolean sendSensors = false;
void calibrate(void);

//
// MQTT message arrived, decode
//
void mqttCallbackHandle(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("MQTT sub: "));
  Serial.println(topic);
  if (!strcmp(topic, calibrateSTopic.c_str()))
  {
    char buf[10];
    strncpy(buf, (char*)payload, length);
    buf[length] = 0;
    calPower = atof(buf);
    Serial.print(F("Calibration power: "));
    Serial.println(calPower);
  }
  calibrate();
}


//
// Handle short touch
//
void shortPress() {
  desiredRelayState = !desiredRelayState; //Toggle relay state.
  sendGroupEventTopic = false;
  sendEvent = true;
  noOfConfigTouches = 0;
}

//
// Handle long touch
//
void longPress() {
  desiredRelayState = !desiredRelayState; //Toggle relay state.
  sendGroupEventTopic = true;
  sendEvent = true;
  noOfConfigTouches = 0;
}

//
// Handle looong config touch
//
void configWifiPress() {
  noOfConfigTouches++;
  if (noOfConfigTouches >= CONFIG_TOUCHES_COUNT)
    configWifi = true;
}


//
// This is executed on touch
//
void buttonChangeCallback() {
  if (digitalRead(0) == 1) {

    // Button has been released, trigger one of the two possible options.
    if (millis() - millisSinceChange > CONFIG_WIFI_PRESS_MS) {
      configWifiPress();
    }
    else if (millis() - millisSinceChange > LONG_PRESS_MS) {
      longPress();
    }
    else if (millis() - millisSinceChange > SHORT_PRESS_MS) {
      shortPress();
    }
    else {
      //Too short to register as a press
    }
  }
  else {
    //Just been pressed - do nothing until released.
  }
  millisSinceChange = millis();
}

DynamicJsonBuffer jsonBuffer(250);

//
// This routine handles state changes and MQTT publishing
//
void handleStatusChange() {

  // publish relay state, pong, event and status messages
  mqttPublish();

  if (sendSensors)
  {
    if (isnan(current) || isnan(voltage) || isnan(power) || isnan(energy))
    {
      Serial.println(F("No sensor data"));
    }    {
      JsonObject& json = jsonBuffer.createObject();
      json["i"] = current;
      json["u"] = voltage;
      json["p"] = power;
      json["e"] = energy;
      String jsonStr;
      json.printTo(jsonStr);
      mqttPublishMessage(sensorTopic.c_str(), jsonStr.c_str());
    }
    sendSensors = false;
  }
}


//
// callback to create custom topics
//
void mqttCallbackCreateTopics() {
  sensorTopic = String(F("sensor/")) + custom_unit_id.getValue() + String(F("/value"));

  // pointer of topics
  subscribedTopics[0] = &calibrateSTopic;
  noSubscribedTopics = 1;
}


//
////////// SETUP //////////
//
void setup() {
  Serial.begin(115200);
  Serial.println(F("Initialising"));
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LEDOFF); //LED off.


  // setup wifi
  wifiSetup(CONFIG_VERSION, false);

  // setup mqtt
  mqttSetup();

  // Initialize HLW8012
  // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
  // * cf_pin, cf1_pin and sel_pin are GPIOs to the HLW8012 IC
  // * currentWhen is the value in sel_pin to select current sampling
  // * set use_interrupts to false, we will have to call handle() in the main loop to do the sampling
  // * set pulse_timeout to 500ms for a fast response but losing precision (that's ~24W precision :( )
  hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, false, 500000);
  // These values are used to calculate current, voltage and power factors as per datasheet formula
  // These are the nominal values for the Sonoff POW resistors:
  // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
  // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the HLW8012
  // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the HLW8012
  hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);
  // Show default (as per datasheet) multipliers
  Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
  Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
  Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
  Serial.println();

  // Enable interrupt for button press
  Serial.println(F("Enabling touch switch interrupt"));
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonChangeCallback, CHANGE);
}

void unblockingDelay(unsigned long mseconds) {
  unsigned long timeout = millis();
  while ((millis() - timeout) < mseconds) delay(1);
}

void calibrate() {

  // Let's first read power, current and voltage
  // with an interval in between to allow the signal to stabilise:

  hlw8012.getActivePower();

  hlw8012.setMode(MODE_CURRENT);
  unblockingDelay(2000);
  hlw8012.getCurrent();

  hlw8012.setMode(MODE_VOLTAGE);
  unblockingDelay(2000);
  hlw8012.getVoltage();

  // Calibrate using a 60W bulb (pure resistive) on a 230V line
  hlw8012.expectedActivePower(60.0);
  hlw8012.expectedVoltage(230.0);
  hlw8012.expectedCurrent(60.0 / 230.0);

  // Show corrected factors
  Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
  Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
  Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
  Serial.println();

}

////////// LOOP //////////
//
void loop() {

  // handle wifi
  wifiLoop();

  // handle mqtt
  mqttLoop();

  // Check MQTT connection
  if (millis() - lastMQTTCheck >= MQTT_CHECK_MS) {
    lastMQTTCheck = millis();

    uptime += MQTT_CHECK_MS / 1000;
    mqttCheckConnection();

    Serial.print(F("sensor read... "));

    voltage = hlw8012.getVoltage();
    current = hlw8012.getCurrent();
    power = hlw8012.getActivePower();
    if (lastEnergySample != 0) {
      float hours = (millis() - lastEnergySample) / 1000.0 / 3600.0;
      energy += power / hours;
    }
    lastEnergySample = millis();
     
    Serial.print("[HLW] Active Power (W)    : "); Serial.println(power);
    Serial.print("[HLW] Voltage (V)         : "); Serial.println(voltage);
    Serial.print("[HLW] Current (A)         : "); Serial.println(current);
    Serial.print("[HLW] Energyt (Wh)        : "); Serial.println(energy);
    Serial.print("[HLW] Apparent Power (VA) : "); Serial.println(hlw8012.getApparentPower());
    Serial.print("[HLW] Power Factor (%)    : "); Serial.println((int) (100 * hlw8012.getPowerFactor()));
    Serial.println();

    // When not using interrupts we have to manually switch to current or voltage monitor
    // This means that every time we get into the conditional we only update one of them
    // while the other will return the cached value.
    hlw8012.toggleMode();
    Serial.println(F("done"));

    sendSensors = true;
  }


  // Handle any state change and MQTT publishing
  handleStatusChange();


  delay(50);
}
