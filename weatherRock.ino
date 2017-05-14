// Author: Casey Zandbergen
// Description: Particle Photon Weathershield for weatherRock
// This #include statement was automatically added by the Particle IDE.

//TODO enable MQTT callback from Node Red
#include "SparkFun_Photon_Weather_Shield_Library.h"
#include "SparkJson.h"
#include "MQTT.h"
#include "neopixel.h"

/* ======================= includes ================================= */

#include "application.h"
#include "neopixel.h" // use for Build IDE
// #include "neopixel.h" // use for local build

/* ======================= prototypes =============================== */

void colorAll(uint32_t c, uint8_t wait);
void colorWipe(uint32_t c, uint8_t wait);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);

/* ======================= extra-examples.cpp ======================== */

SYSTEM_MODE(AUTOMATIC);

// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_COUNT 32
#define PIXEL_PIN_A D6
#define PIXEL_PIN_B D7
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel strip_A = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN_A, PIXEL_TYPE);
Adafruit_NeoPixel strip_B = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN_B, PIXEL_TYPE);

char mqttBroker[32] = "98.164.33.189";    //MQTT Broker URL / IP
String mqttPub = "weatherRock/data";      //MQTT Publication Channel
String mqttSub = "weatherRock/sub";       //MQTT Subscription Channel
String mqttLog = "log/";                  //MQTT logging channel
String value = "0";                       //Initialize reporting value
int wait = 5000;                          //Time between loops, this is the heartbeat() interval
String myID;                              //Variable for the Photon device ID
String strMqtt;                           //Variable to contain the MQTT string
int str_len;                              //Variable for String Length
int counter = 0;                          //Variable to count time for reporting
int reportDelay = 15000;                  //Time Between reports
int rssi;                                 //RSSI strength variable
int led = 7;                              //Which LED to blink

// WeatherRock Variables

double humidity = 0;
double tempf = 0;
double pascals = 0;
double baroTemp = 0;
long lastPublish = 0;
char charHumidity [10];
char charTemp [10];
char charBaroTemp [10];
char charPascals [10];
int debug = 0;

String stringHumidity;
String stringTemp;
String stringPSI;
String stringBaroTemp;
String stringRssi;

Weather sensor;                             //Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor

MQTT client(mqttBroker, 1883, callback);   //Initialized MQTT broker

void setup()
    {
        strip_A.begin();
        strip_A.show(); // Initialize all pixels to 'off'

        //strip_B.begin();
        //strip_B.show(); // Initialize all pixels to 'off'

        //Set pin modes
        pinMode(led, OUTPUT);

        //Initialize the I2C sensors and ping them
        sensor.begin();

        //Choose one below
        sensor.setModeBarometer();          //Set to Barometer Mode
        //baro.setModeAltimeter();          //Set to altimeter Mode

        //These are additional MPL3115A2 functions that MUST be called for the sensor to work.
        sensor.setOversampleRate(7);        // Set Oversample rate
        sensor.enableEventFlags();          //Necessary register calls to enble temp, baro and alt

        //Get the deviceID
        myID = System.deviceID();
        strMqtt = mqttPub;

        //Convert the string to Char for MQTT
        str_len = myID.length() + 1;
        char char_array3[str_len];
        myID.toCharArray(char_array3, str_len);

        //Create and publish the JsonObject
        StaticJsonBuffer<200> jsonBuffer;
        char buffer [200];

        JsonObject& root = jsonBuffer.createObject();
        root["deviceID"] = char_array3;
        root["status"] = "Connected at startup";
        root.printTo(buffer, sizeof(buffer));

        //Connect to the MQTT broker
        client.connect("connect");
        client.publish(strMqtt, buffer);
        client.subscribe(mqttSub);
    }

void loop()
    {
        if (millis()>counter)   //Report Timer
            {
                getWeather();
                mqttPublish();
                particlePublish();

                counter = counter + reportDelay;

                colorWipe(strip_A.Color(255, 0, 0), 50); // Red
            }

        client.loop();


          //colorWipe(strip.Color(255, 0, 0), 50); // Red

          //colorWipe(strip.Color(0, 255, 0), 50); // Green

          //colorWipe(strip.Color(0, 0, 255), 50); // Blue

          rainbow_A(10);
          //rainbow_B(5);

          //rainbowCycle(20);

          //colorAll(strip.Color(0, 255, 255), 50); // Cyan
        //delay(wait);

    }

//Function handles reporting to the MQTT broker
void mqttPublish()
    {
        String feed;

        //Convert the string to Char for MQTT
        int str_len;

        str_len = stringHumidity.length() + 1;
        char charHumidity[str_len];
        stringHumidity.toCharArray(charHumidity, str_len);

        str_len = stringTemp.length() + 1;
        char charTemp[str_len];
        stringTemp.toCharArray(charTemp, str_len);

        str_len = stringBaroTemp.length() + 1;
        char charBaroTemp[str_len];
        stringBaroTemp.toCharArray(charBaroTemp, str_len);

        str_len = stringPSI.length() + 1;
        char charPSI[str_len];
        stringPSI.toCharArray(charPSI, str_len);

        str_len = myID.length() + 1;
        char char_myID[str_len];
        myID.toCharArray(char_myID, str_len);

        //Get rssi
        rssi = WiFi.RSSI();
        stringRssi = String(rssi);
        str_len = stringRssi.length() + 1;
        char char_rssi[str_len];
        stringRssi.toCharArray(char_rssi, str_len);

        //Build json REPORT object

        StaticJsonBuffer<200> jsonBuffer;
        char bufferReport [200];

        JsonObject& root = jsonBuffer.createObject();
        root["deviceID"] = char_myID;
        root["rssi"] = char_rssi;
        root["temp"] = charTemp;
        root["baroTemp"] = charBaroTemp;
        root["humidity"] = charHumidity;
        root["PSI"] = charPSI;
        root["status"] = "Connected during report";
        root.printTo(bufferReport, sizeof(bufferReport));

        client.connect("connect");      //Connect to the MQTT Broker

        if (client.isConnected())
            {
                client.publish(strMqtt, bufferReport);
                Particle.publish("weatherRock", bufferReport, 60, PRIVATE);
                blink(3);

            }

        }

// Allows us to recieve a message from the subscription
void callback(char* topic, byte* payload, unsigned int length)
    {
        char p[length + 1];
        memcpy(p, payload, length);
        p[length] = NULL;
        String message(p);

    //This is where you put code to handle any message recieved from the broker

    }

void blink(int blinks)
    {

        int x = 0;

        do
        {
          digitalWrite(led, HIGH);
          delay(100);
          digitalWrite(led, LOW);
          delay(100);
          x = x + 1;

        } while (x < blinks);
    }

void heartbeat()
    {
        //Convert the string to Char for MQTT

        str_len = myID.length() + 1;
        char char_myID[str_len];
        myID.toCharArray(char_myID, str_len);

        //Create json status object
        StaticJsonBuffer<200> jsonBuffer;
        char buffer [200];

        JsonObject& root = jsonBuffer.createObject();
        root["deviceID"] = char_myID;
        root["status"] = "Heartbeat";

        //Publish the JsonObject
        root.printTo(buffer, sizeof(buffer));
        client.publish(strMqtt, buffer);

    }

void getWeather()
    {
        stringHumidity = String(sensor.getRH());
        stringTemp = String((sensor.getTempF())-35);
        stringBaroTemp = String(sensor.readBaroTempF());
        pascals = sensor.readPressure();
        stringPSI = String((pascals + 2800) / 100); //Altitude offset for Tulsa

        digitalWrite(7, HIGH);
        delay(500);
        digitalWrite(7, LOW);

    }

void particlePublish()
    {

        Particle.publish("humidity", stringHumidity, 60, PRIVATE);
        Particle.publish("temp", stringTemp, 60, PRIVATE);
        Particle.publish("baroTemp", stringBaroTemp, 60, PRIVATE);
        Particle.publish("PSI", stringPSI, 60, PRIVATE);
        Particle.publish("rssi", stringRssi, 60, PRIVATE);
    }

void rainbow_A(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip_A.numPixels(); i++) {
      strip_A.setPixelColor(i, Wheel_A((i+j) & 255));
    }
    strip_A.show();
    //delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel_A(byte WheelPos) {
  if(WheelPos < 85) {
   return strip_A.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip_A.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip_A.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel_B(byte WheelPos) {
  if(WheelPos < 85) {
   return strip_B.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip_B.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip_B.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

// Fill the dots one after the other with a color, wait (ms) after each one
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip_A.numPixels(); i++) {
    strip_A.setPixelColor(i, c);
    strip_A.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout, then wait (ms)
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) { // 1 cycle of all colors on wheel
    for(i=0; i< strip_A.numPixels(); i++) {
      strip_A.setPixelColor(i, Wheel(((i * 256 / strip_A.numPixels()) + j) & 255));
    }
    strip_A.show();
    delay(wait);
  }
}
