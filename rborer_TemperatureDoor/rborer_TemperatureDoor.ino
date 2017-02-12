
#define MY_DEBUG
//#define MY_SPECIAL_DEBUG

#define USE_INTERRUPT
#define MY_NODE_ID AUTO
#define MY_BAUD_RATE 57600
#define MY_RADIO_NRF24


#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <SI7021.h>
#ifndef MY_OTA_FIRMWARE_FEATURE
#include "drivers/SPIFlash/SPIFlash.cpp"
#endif
#include <EEPROM.h>
#include <RunningAverage.h>


#define BATT_SENSOR    199

#define RELEASE "1.5"

#define AVERAGES 2

// Child sensor ID's
#define CHILD_ID_TEMP   1
#define CHILD_ID_HUM    2
#define CHILD_ID_DOOR    3


#define MEASURE_INTERVAL 60000

#define OTA_WAIT_PERIOD 30000

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30

#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5

// Pin definitions
#define TEST_PIN       A0
#define LED_PIN        A2
#define DOOR_PIN       3
#define MOTION_PIN     4

SI7021 humiditySensor;
SPIFlash flash(8, 0x1F65);

// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgDoor(CHILD_ID_DOOR, V_TRIPPED);

#ifdef BATT_SENSOR
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
#endif

// Global settings
int measureCount = 0;
int sendBattery = 0;
boolean isMetric = true;
boolean highfreq = true;
boolean transmission_occured = false;
#ifdef USE_BOUNCER
Bounce debouncer = Bounce();
#endif

// Storage of old measurements
float lastTemperature = -100;
int lastHumidity = -100;
long lastBattery = -100;
bool old_door_val = false;
bool old_motion_val = false;

RunningAverage raHum(AVERAGES);

/****************************************************

   Setup code

 ****************************************************/
void setup() {
 
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(MY_BAUD_RATE);
  Serial.print(F("Rborer Sensebender Micro temp/door unit"));
  Serial.print(RELEASE);
  Serial.flush();


  digitalWrite(LED_PIN, HIGH);

  humiditySensor.begin();

  digitalWrite(LED_PIN, LOW);

  Serial.flush();
  Serial.println(F(" - Online!"));

  isMetric = true;
  Serial.print(F("isMetric: ")); Serial.println(isMetric);
  raHum.clear();
  sendTempHumidityMeasurements(false);
  sendBattLevel(false);


}

void presentation()  {
  sendSketchInfo("Sensebender Micro", RELEASE);

  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_HUM, S_HUM);


#ifdef BATT_SENSOR
  present(BATT_SENSOR, S_POWER);
#endif
}


/***********************************************

    Main loop function

 ***********************************************/
void loop() {

  bool motion;
  bool door_val;
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;

  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true;
    measureCount = 0;
  }

  door_val = digitalRead(DOOR_PIN) == HIGH ? true : false;
 Serial.print(F("Door  :")); Serial.println(door_val);
  sendTempHumidityMeasurements(forceTransmit);

  if (door_val != old_door_val) {
    // Send in the new value
    send(msgDoor.set(door_val ? 1 : 0));
    old_door_val = door_val;
  }
  sleep(digitalPinToInterrupt(DOOR_PIN),CHANGE,MEASURE_INTERVAL);
}


/*********************************************

   Sends temperature and humidity from Si7021 sensor

   Parameters
   - force : Forces transmission of a value (even if it's the same as previous measurement)

 *********************************************/
void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;

  si7021_env data = humiditySensor.getHumidityAndTemperature();

  raHum.addValue(data.humidityPercent);

  float diffTemp = abs(lastTemperature - (isMetric ? data.celsiusHundredths : data.fahrenheitHundredths) / 100.0);
  float diffHum = abs(lastHumidity - raHum.getAverage());

  Serial.print(F("TempDiff :")); Serial.println(diffTemp);
  Serial.print(F("HumDiff  :")); Serial.println(diffHum);

  if (isnan(diffHum)) tx = true;
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;
    float temperature = (isMetric ? data.celsiusHundredths : data.fahrenheitHundredths) / 100.0;

    int humidity = data.humidityPercent;
    Serial.print("T: "); Serial.println(temperature);
    Serial.print("H: "); Serial.println(humidity);

    send(msgTemp.set(temperature, 1));
    send(msgHum.set(humidity));
    lastTemperature = temperature;
    lastHumidity = humidity;
    transmission_occured = true;
    if (sendBattery > 60) {
      sendBattLevel(true); // Not needed to send battery info that often
      sendBattery = 0;
    }
  }
}
/*******************************************

   Internal TEMP sensor

 *******************************************/
float readMCP9700(int pin, float offset)

{
  analogReference(INTERNAL);

  analogRead(A0); //perform a dummy read to clear the adc
  delay(20);

  for (int n = 0; n < 5; n++)
    analogRead(pin);

  int adc = analogRead(pin);
  float tSensor = ((adc * (1.1 / 1024.0)) - 0.5 + offset) * 100;
  float error = 244e-6 * (125 - tSensor) * (tSensor - -40.0) + 2E-12 * (tSensor - -40.0) - 2.0;
  float temp = tSensor - error;

  return temp;
}

/********************************************

   Sends battery information (battery percentage)

   Parameters
   - force : Forces transmission of a value

 *******************************************/
void sendBattLevel(bool force)
{
  if (force) lastBattery = -1;
  long vcc = readVcc();
  if (vcc != lastBattery) {
    lastBattery = vcc;

#ifdef BATT_SENSOR
    float send_voltage = float(vcc) / 1000.0f;
    send(msgBatt.set(send_voltage, 3));
#endif

    // Calculate percentage

    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at

    long percent = vcc / 14.0;
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}

/*******************************************

   Internal battery ADC measuring

 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADcdMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts

}

