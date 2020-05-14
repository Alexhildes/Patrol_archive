#include <OneWire.h> //Temperature Sensors Coolant, External
#include <DallasTemperature.h> //As Above
#include <PID_v1.h>

//DS18B20 Variables

#define ONE_WIRE_BUS 2                // Data wire is plugged into digital port 8 on the Arduino (Blue Wire)
OneWire oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to communicate with any OneWire devices 
DallasTemperature sensors(&oneWire);  

//Temperature Variables
int temperature1 = 0;
int temperature2 = 0;


//Led Variables
int yellow = 3;
int green = 4;
int red = 5;

//Control Variables


void setup() {

sensors.begin();
Serial.begin(9600);

pinMode(yellow, OUTPUT);
pinMode(green, OUTPUT);
pinMode(red, OUTPUT);


}

void loop() {

  //LEDs lighting  
  digitalWrite(yellow, HIGH);
  delay(100);
  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);
  delay(100);
  digitalWrite(green, LOW);
  digitalWrite(red, HIGH);
  delay(100);
  digitalWrite(red, LOW);


  Serial.print("temp1=");
  Serial.print(temperature1);
  Serial.print(',');
  Serial.print("temp2=");
  Serial.print(temperature2);
  Serial.print('\n');


  delay(30000);
}

void tempReadings() {

//Request temperatures from sensors
  sensors.requestTemperatures(); 
  temperature1 = (int) 10*sensors.getTempCByIndex(0);
  temperature2 = (int) 10*sensors.getTempCByIndex(1);

//Printing to Raspberry Pi
  Serial.print("temp1=");
  Serial.print(temperature1);
  Serial.print(',');
  Serial.print("temp2=");
  Serial.print(temperature2);
  Serial.print('\n');

}