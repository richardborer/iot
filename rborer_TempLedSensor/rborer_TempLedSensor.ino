// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE

#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

#define NUMPIXELS 150   // Number of connected pixels on a single datapin
#define PIN 8         // Digital output pin

#define node 254  //254 for testing purpose
#define CHILD_ID 0

#define HUMIDITY_SENSOR_DIGITAL_PIN 3
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
DHT dht;
float lastTemp;
float lastHum;
int countTemp=0;
boolean metric = true;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_RGB 21
#define CHILD_ID_LIGHT 22

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
long RGB_values[3] = {0, 0, 0};


void setup()
{
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

}

void presentation()
{

  sendSketchInfo("MultiSensor", "1.1");
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_LIGHT, S_LIGHT);
  strip.begin();
  strip.show(); // Update the strip, to start they are all 'off'
}


void loop()
{
  
    SendTemp();
  wait(10000); 
}

void SendTemp()
{
  delay(dht.getMinimumSamplingPeriod());

  // Fetch temperatures from DHT sensor
  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT");
  } else {
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    send(msgTemp.set(temperature, 1));
#ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
#endif
  }

  // Fetch humidity from DHT sensor
  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } else  {
    lastHum = humidity;
    send(msgHum.set(humidity, 1));
#ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
#endif
  }
}


void receive(const MyMessage &message) {
  if (message.type == V_RGB) {
    String hexstring = message.getString(); //here goes the hex color code coming from through MySensors (like FF9A00)
    long number = (long) strtol( &hexstring[0], NULL, 16);
    RGB_values[0] = number >> 16;
    RGB_values[1] = number >> 8 & 0xFF;
    RGB_values[2] = number & 0xFF;

    colorWipe(Color(RGB_values[0], RGB_values[1], RGB_values[2]), 60);
  }

  if (message.type == V_DIMMER) {
    strip.setBrightness(round((2.55 * message.getInt())));
    strip.show();
  }

  if (message.type == V_LIGHT) {
    // Incoming on/off command sent from controller ("1" or "0")
    int lightState = message.getString()[0] == '1';

    if (lightState == 1) {
      rgbShowOn();
    }
    else if (lightState == 0) {
      rgbShowOff();
    }
  }

}
void rgbShowOff()
{
  strip.begin();
  Serial.println(": Off");
  for (int i = 0; i < 149; i++ ) {
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}
void rgbShowOn()
{
  int r = 255;
  int g = 0;
  int b = 255;
  Serial.println(": On");
  strip.begin();
  for (int i = 0; i < 149; i++ ) {
    strip.setPixelColor(i, r, g, b);
  }
  strip.show();
}

void colorWipe(uint32_t c, uint8_t wait) {
  int i;

  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

/* Helper functions */

// Create a 15 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

