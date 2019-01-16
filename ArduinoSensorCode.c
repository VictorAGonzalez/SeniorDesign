#include "DHT.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <OneWire.h>
#include <DallasTemperature.h>
// Create a Onewire Referenca and assign it to pin 10 on your Arduino
#include <OneWire.h>
OneWire  ds(10);  // on pin 10
// declare as sensor referenec by passing oneWire reference to Dallas Temperature. 


// declare your device address
// YOUR ADDRESS GOES HERE!!!!
DeviceAddress tempSensor = {0x28, 0xFF, 0x2B, 0x45, 0x4C, 0x04, 0x00, 0x10};

// A Variable to hold the temperature you retrieve
float tempC;
#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define redpin 3
#define greenpin 5
#define bluepin 6
float volt;
int sensorValue;
float vol;
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of cllection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;

double convertToVoltage(int ADCvalue);
void initializeLEDPins(int redPin, int greenPin, int bluePin);
void redON();
void greenON();
void blueON();
// Water Temp
 float Celcius=0;
 float Fahrenheit=0;
 //
#define commonAnode false

const int sensorPin = A0; //Analog In Pin 0
int ADCvalue = 0;
double sensorVoltage = 0;

byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHTxx test!");
  Serial.println("Color View Test!");
  Serial.println("Water Temp Test!");

  
  

  dht.begin();                        //End of DHT setup

  if (tcs.begin()) {                  //Start of ColorSensor setup
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }                                      //End of ColorSensor Setup
  pinMode(LED, OUTPUT);                //For PH Sensor


}

void loop() {
  uint16_t clear, red, green, blue;    //Beginning of ColorSensor loop
  tcs.setInterrupt(false);      // turn on LED

  delay(1000);

  float h = dht.readHumidity();      //Beginning of DHT sensor
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temp: ");
  Serial.print(f); //Serial.print(c);
  Serial.print(" *F ");
  Serial.println();                    //End of DHT loop

  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  Serial.print((int)r, DEC); Serial.print((int)g, DEC); Serial.print((int)b, DEC);
  Serial.println();

  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

  analogWrite(redpin, gammatable[(int)r]);
  analogWrite(greenpin, gammatable[(int)g]);
  analogWrite(bluepin, gammatable[(int)b]);      //End of Color Sensor Loop

                                                                    // Turbidity Start 
  int sensorValue = analogRead(A0);// read the input on analog pin 0:
  volt = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.print("Turbidity Value: ");
  Serial.println(volt); // print out the value you read:
  delay(500);

  sensorValue = analogRead(A1);               
  //Start of Nitrogen Senor Loop

  //vol=(float)sensorValue/1024*5.0;

  Serial.print("PPM=");
  Serial.println(sensorValue);                  //End of Nitrogen Sensor Loop

  static unsigned long samplingTime = millis(); //Start of PH loop
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    Serial.print("Voltage:");
    Serial.print(voltage, 2);
    Serial.print("  pH value: ");
    Serial.println(pHValue, 2);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    printTime = millis();
  }                                            
                                                    //End of PH loop


                                              // START OF WATER TEMP SENSOR
byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
   // Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  //Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
   // Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
     // Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
     // Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
     // Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
     // Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
     // Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present,HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
   // Serial.print(data[i], HEX);
  //  Serial.print(" ");
  }
  //Serial.print(" CRC=");
 // Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

  Serial.print(h);
  Serial.print(",");
  Serial.print(f);
  Serial.print(",");
  Serial.print(red);
  Serial.print(",");
  Serial.print(green);
  Serial.print(",");
  Serial.print(blue);
  Serial.print(",");
  Serial.print(volt);
  Serial.print(",");
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(pHValue, 2);
  Serial.print(",");
  Serial.print(celsius);
  Serial.print(",");
  Serial.print(fahrenheit);
  Serial.println();
