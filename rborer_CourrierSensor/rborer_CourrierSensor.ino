
//#define MY_DEBUG
#define MY_RADIO_NRF24
#include <SPI.h>
#include <MySensors.h>
#include <SparkFun_APDS9960.h>

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint8_t proximity_data = 0;
#define APDS9960_INT    3  // Needs to be an interrupt pin

#define BATT_SENSOR   195
#define MAIL_SENSOR    194

#define SLEEP_TIME 600000

int MailInBox;
int sendBattery = 0;
long lastBattery = -100;

// Constants
#define PROX_INT_HIGH   255 // Proximity level for interrupt
#define PROX_INT_LOW    200  // No far interrupt

MyMessage msgMailBox(MAIL_SENSOR, V_TRIPPED);
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);

void setup() {
  //pinMode(APDS9960_INT, INPUT_PULLUP);
  apds.init();
  
 // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
 /*
  
  // Set proximity interrupt thresholds
  if ( !apds.setProximityIntLowThreshold(PROX_INT_LOW) ) {
    Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setProximityIntHighThreshold(PROX_INT_HIGH) ) {
    Serial.println(F("Error writing high threshold"));
  }
  */
  // Start running the APDS-9960 proximity sensor (interrupts)
  if ( apds.enableProximitySensor(true) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
  
  MailInBox = 0;
  sendBattLevel(true);
}

void presentation() {
  sendSketchInfo("Courrier Sensor", "1.0");
  present(MAIL_SENSOR, S_DOOR);
  present(BATT_SENSOR, S_POWER);
}

void loop() {

   
    if ( apds.readProximity(proximity_data) ) {

      Serial.println(F("======VALUE: "));
      Serial.println(proximity_data);
      if (proximity_data >= 245 && MailInBox == 0)
      {
        MailInBox = 1;
        send(msgMailBox.set(MailInBox));
        Serial.println(F("================Courrier disponible"));
      }

      if (proximity_data < 245 && MailInBox == 1)
      {
        MailInBox = 0;
        send(msgMailBox.set(MailInBox));
        Serial.println(F("================Courrier releve"));
      }
        sendBattLevel(true); // Not needed to send battery info that often
    }
    //apds.clearProximityInt();


  sleep(SLEEP_TIME);
  Serial.println(F("wake up"));
}


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



