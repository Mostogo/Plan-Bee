#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <OneWire.h> //DS18B20 Intern temp
#include "HX711.h" //scale HX711 
#include "DHT.h" //DHT22 Extern humidity and temp

//Struct to send values to Sigfox
typedef struct __attribute__ ((packed)) sigfox_message {
 int16_t temp0;
 int16_t temp1;
 int16_t temp2;
 int16_t weight;
 int8_t tempDHT;
 int8_t humidity;
 int8_t etatBattery;
 int8_t humidityInside; 
} SigfoxMessage;
// stub for message which will be sent
SigfoxMessage msg;
// =================== UTILITIES ===================
void reboot() {
 NVIC_SystemReset();
 while (1);
}
// ====================================================================================== DS18B20 begin
 /*
  * Capteur DS18B20
  */
/* 1-Wire bus pin */
const byte BROCHE_ONEWIRE = 5;
/* Sensors address */
const byte SENSOR_ADDRESS_1[] = { 0x28,  0xB0,  0x26,  0x35,  0xC,  0x0,  0x0,  0xF};
const byte SENSOR_ADDRESS_2[] = { 0x28,  0xBA,  0x39,  0x53,  0xA,  0x0,  0x0,  0x9B};
const byte SENSOR_ADDRESS_3[] = { 0x28,  0x5,  0x59,  0x53,  0xA,  0x0,  0x0,  0xA9};
/* Create 1-wire object to use */
OneWire ds(BROCHE_ONEWIRE);
/**
 * Function to read temperature of DS18B20 sensor.
 */
float getTemperature(const byte addr[]) {
  byte data[9];
  // data[] : Données lues depuis le scratchpad
  // addr[] : Adresse du module 1-Wire détecté
   /* Reset 1-Wire bus and select sensor */
  ds.reset();
  ds.select(addr);
  /* Start data reaad and wait until it's done */
  ds.write(0x44, 1);
  delay(800);
  /* Reset 1-Wire bus, select sensor and started reading the scratchpad */
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);
 /* Read scratchpad */
  for (byte i = 0; i < 9; i++) {
    data[i] = ds.read();
  }
  /* Read temperature in degrees */
  return (int16_t) ((data[1] << 8) | data[0]) * 0.0625; 
}

void getDS18B20(){
  float temperature[3];
   
  /* Read all sensors*/
  temperature[0] = getTemperature(SENSOR_ADDRESS_1);
  temperature[1] = getTemperature(SENSOR_ADDRESS_2);
  temperature[2] = getTemperature(SENSOR_ADDRESS_3);
  
  /* Print temp */
  /*Serial.print(F("Temperatures : "));
  Serial.print(temperature[0], 2);
  Serial.write(176); // Caractère degré
  Serial.print(F("C, "));
  Serial.print(temperature[1], 2);
  Serial.write(176); // Caractère degré
  Serial.print(F("C, "));
  Serial.print(temperature[2], 2);
  Serial.write(176); // Caractère degré
  Serial.println('C');*/

  msg.temp0 = (temperature[0]*10); // temperature in degrees
  msg.temp1 = (temperature[1]*10); // temperature in degrees
  msg.temp2 = (temperature[2]*10); // temperature in degrees
}
// ====================================================================================== DS18B20 end

/*
 * Weight sensor HX711
 */
// ====================================================================================== HX711 begin
// HX711 circuit wiring
const int DOUT = 2; // White wire
const int PSCK = 3; // Yellow wire

HX711 scale;

float calibration_factor = 20950; // Personnal factor for calibration of the scale
float zero_factor = 777787; //Numeric value of 0 Kg

void getHX711(){
  scale.power_up(); //Turn on the scale
  msg.weight = (scale.get_units())*10; // Weight in kg
  scale.power_down(); //Turn off the scale
  if (msg.weight < 0) {
    msg.weight = 0;
  }
  /*Serial.print(msg.weight/10);
  Serial.println(" kg");*/
}
// ====================================================================================== HX711 end

// ====================================================================================== DHT22 begin
#define DHTPIN 1 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // Specify which DHTxx we use
DHT dht(DHTPIN, DHTTYPE);

void getDHT22(){
  msg.tempDHT = ((dht.readTemperature())*2); // temperature degrees
  msg.humidity = ((dht.readHumidity())); // humidity percent
  /*Serial.print(msg.tempDHT);
  Serial.println("°C -> DHT");
  Serial.print(msg.humidity);
  Serial.println("% of humidity");*/
}
// ====================================================================================== DHT22 end

// ====================================================================================== DHT22 inside begin
#define DHTPINinside 4 // Digital pin connected to the DHT sensor
#define DHTTYPEinside DHT22 // Specify which DHTxx we use
DHT dhtInside(DHTPINinside, DHTTYPEinside);

void getDHT22inside(){
  msg.humidityInside = ((dhtInside.readHumidity())); // humidity percent inside the hive
  /*
  Serial.print(msg.humidityInside);
  Serial.println("% of humidity");
  */
}
// ====================================================================================== DHT22 inside end

// ====================================================================================== Battery begin
#define ADC A1

void getBattery(){
  analogReadResolution(10);  // 10 bits accuracy
  float voltage=0,conversion=0;
  int sensorValue = analogRead(ADC); //ADC read
  //float voltage = ((float)sensorValue * (4.2/1023)); // scaling to display battery value
  // conversion = (voltage-3.2)*100;
  // 880,900,920,940,960,980,1000
  // multiplication of 12.5 of the battery
  // for adaptating the percent
  if(sensorValue < 856) msg.etatBattery = 0;
  else if((sensorValue >= 856)&&(sensorValue < 880)) msg.etatBattery = 1;   // 0    / 12.5 %                  
  else if((sensorValue >= 880)&&(sensorValue < 900)) msg.etatBattery = 2;   // 12.5 / 25 %                  
  else if((sensorValue >= 900)&&(sensorValue < 920)) msg.etatBattery = 3;   // 25   / 37.5 %                
  else if((sensorValue >= 920)&&(sensorValue < 940)) msg.etatBattery = 4;   // 37.5 / 50 %
  else if((sensorValue >= 940)&&(sensorValue < 960)) msg.etatBattery = 5;   // 50   / 62.5 %
  else if((sensorValue >= 960)&&(sensorValue < 980)) msg.etatBattery = 6;   // 62.5 / 75 %
  else if((sensorValue >= 980)&&(sensorValue < 1000)) msg.etatBattery = 7;  // 75   / 87.5 %
  else if((sensorValue >= 1000)&&(sensorValue < 1024)) msg.etatBattery = 8; // 87.5 / 100 %
  // else Serial.println("ERROR battery");
  // Serial.println(msg.etatBattery*12.5);
}

// ====================================================================================== Battery end


void sendValues(){
  // Clear all pending interrupts
  SigFox.begin(); 
  SigFox.status();
  // Send the data
  SigFox.beginPacket();
  SigFox.write((uint8_t*)&msg, sizeof(SigfoxMessage));
  /*Serial.print("Status: ");
  Serial.println(SigFox.endPacket());*/
  SigFox.endPacket();
  SigFox.end();
  /*Serial.println("\n\n\n\n\n");*/
}
/** Fonction setup() **/
void setup() {
  
  // while (!Serial);
  if (!SigFox.begin()) {
  /*Serial.println("SigFox error, rebooting");*/
  reboot();
  }
  // Enable debug prints and LED indication
  SigFox.debug();
  /* serial port initialisation */
  //Serial.begin(115200);
  /* scale Initialisation*/
  scale.begin(DOUT, PSCK); // PIN initialisation  
  scale.set_scale(calibration_factor); // Adjust to this calibration factor
  scale.set_offset(zero_factor); // Adjust to this zero factor
  dht.begin(); //DHT22 initialisation
  dhtInside.begin(); //DHT22Inside initialisation
  pinMode(ADC, INPUT); //For the battery, configuration of ADC
  pinMode(0, OUTPUT); //Show a light when the system start
  digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(3000);                       // wait for a second
  digitalWrite(0, LOW);    // turn the LED off by making the voltage LOW
}
 
 
/** Fonction loop() **/
void loop() {
  getDS18B20();
  getHX711();
  getDHT22();
  getDHT22inside();
  getBattery();
  sendValues();
  
  LowPower.deepSleep(1200000); //wait 20min (1200000)
  //delay(10000); //60 et 140 mA
}
