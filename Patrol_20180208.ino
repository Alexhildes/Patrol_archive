 //Author: Alex Hildebrand   GQ Patrol Engine and Turbo Monitoring  September 2017

#include <genieArduino.h> //Touch Screen Library Interface
#include <max6675.h>  //Temperature Sensor EGT
#include <OneWire.h> //Temperature Sensors Coolant, External
#include <DallasTemperature.h> //As Above
#include <ResponsiveAnalogRead.h>
#include <Wire.h>     //RTC Library
#include "RTClib.h"   //RTC Library

//Real time clock
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int datemonth;
int dateday;
int monthday;

int dateminute;
int datehour;
int hourminute;

//4D Systems Display
Genie genie;    //Initialises Screen Comunication
#define RESETLINE 4

//DS18B20 Sensor variables
int temperature1;
int temperature2;
int temperature3;
int boostValue;
int exhaustTemperature;

//Oil Pressure Variables
float oil1;
int oil2;

//Thermocouple Variables
int thermoDO = 13; //Brown Wire
int thermoCS = 12; //White Wire
int thermoCLK = 11;//Orange Wire
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//DS18B20 Vairables
#define ONE_WIRE_BUS 8                // Data wire is plugged into digital port 8 on the Arduino (Blue Wire)
OneWire oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to communicate with any OneWire devices 
DallasTemperature sensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature.

//Battery Monitor Variables
ResponsiveAnalogRead mainBatteryRead(A5, true);     //Responsive Analogue Read Library reading Analogue 5 Pin (Main Battery)
ResponsiveAnalogRead auxBatteryRead(A4, true);      //Responsive Analogue Read Libary reading Analogue 6 Pin, Wire #2 (Aux Battery)
ResponsiveAnalogRead solarRead(A3, true);           //Responsive Analogue Read Libary reading Analogue 7 Pin, Wire #4 (Solar input)

int mainBatteryValue;                               //Input value from voltage divider,range 0:1023
float mainBatteryVoltage;                          //Stores voltage as float to get more accuracy
float mainBatteryDivider;
int mainResistor1 = 19900;
int mainResistor2 = 10020;

int auxBatteryValue;                               //Input value from voltage divider,range 0:1023
float auxBatteryVoltage;                           //Stores voltage as float to get more accuracy
float auxBatteryDivider;
int auxResistor1 = 20000;
int auxResistor2 = 10030; 

int solarValue;
float solarVoltage;
float solarDivider;
int solarResistor1 = 29820;
int solarResistor2 = 10010;

float errorCorrection = 1.026;
float auxBatteryCurrent;

//MAP Sensor Variables
int pressureValue;  //Input value from pressure sensor, range 0:1023
float boostVoltage; //Variable for storing Vout from MAP sensor
float boostPress;   //Pressure value in KPA
int boostPress2;   //Pressure value in Psi

//Test for turbo timer
int led1 = 22;

//Touchscreen Power
int screen = 24;

//Ignition
int ignitionPin = 5;    //Ignition Pin connected to Digital 5
boolean ignitionOn;
boolean screenReset;
boolean turboHot;      //Is the exhaust temperature over a certain value to warrant turbo timer
int startPin = 26;

//Contrast
int contrast;
int dayContrast = 15;
int nightContrast = 5;

//DC-DC Charger
int chargerLED = 6;    //DC-DC Charger Status to Digital 6
boolean chargerStatus;

//Spare
int spareInput = 7;    //Spare input to Digital 7

void setup() 
{ 

  delay(100);
  //Serial configurations
  Serial.begin(9600);          //Serial1 @ 9600 for Serial Monitor
  Serial2.begin(200000);       //Serial2 @ 200000 for Genie
  genie.Begin(Serial2);        //Begins communications with Genie

  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events

  //PinMode Screen
  pinMode(screen, OUTPUT);

  //PinMode IgnitionPin
  pinMode(ignitionPin, INPUT);

  //Temperature Sensors begin
  sensors.begin();

  //Contrast
  contrast = dayContrast;

  //mainBatteryVoltage Divider
  mainBatteryDivider = (float) 10*(mainResistor1 + mainResistor2)/mainResistor1;

  //auxBatteryVoltage Divider
  auxBatteryDivider = (float) 10*(auxResistor1 + auxResistor2)/auxResistor1;

  //solarBatteryVoltage Divider
  //solarDivider = (float) 14.93*(solarResistor1 + solarResistor2)/solarResistor1;
    solarDivider = 20.0;
  //Turbo hot
  turboHot = false;

  //Begin Real Time Clock
  rtc.begin();
    
} 
void loop()
{
  
  static long waitPeriod = millis();
  
    if (millis() >= waitPeriod) {

      ignitionCheck();          //Runs ignition check
      turboTimer();             //Runs turbo turboTimer
    
      if   (ignitionOn == true && screenReset == true) {    //If ignition is on, and the screen has been reset
      //Run the screen display loops
        
      tempSensors();            //Runs Temp Sensor Script
      oilPressure();            //Runs Oil Pressure Script
      batteryVoltage();         //Runs Battery Voltage Script
      exhaustTemp();            //Runs Exhaust Temp Script
      mapSensor();              //Runs Map Sensor Script
      dateTime();
      
      genie.DoEvents();         //Check for events from Screen
      
      Serial.println("Main Loop");

      } 
      
        waitPeriod = millis() + 200;
        
    }
}
  
void tempSensors() {
          //Request temperatures form all DS18b20 sensors on the common Bus (BLUE WIRE)
          sensors.requestTemperatures(); 
          //Reading temperature values form sensors
          temperature1 = sensors.getTempCByIndex(0); //Temperature from coolant Sensor (BLUE WIRE)
          //temperature3 = sensors.getTempCByIndex(1); //Temperature form 2nd Sensor on BUS (BLUE WIRE)
  
          temperature2 = 10*temperature1;            //Multiple by 10 for Screen Display
        
        //Writing to touch screen display
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, temperature2/10); //Displays Coolant Temp as Digits (Form 1)
        genie.WriteObject(GENIE_OBJ_GAUGE, 3, temperature2/10);      //Display Coolant Temp on horizontal gauge (Form 0)
        //Serial.println(temperature2/10);

        //Serial.println("TempSensors Loop");
}

void oilPressure() {
  //Reading Analogue Pin 1 and converting to PSI

  if (ignitionOn = true) {
    
  oil1= (25.0*((5.0*(analogRead(A2)/1023.0)) - 0.5)) - 2;         //Oil Pressure sensor in PSI (GREEN WIRE)
  
  } else {

  oil1 = 0;
  }
  
      //Writing to touch screen display
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, oil1);    //Displays Oil Press as Digits (Form 0)
      genie.WriteObject(GENIE_OBJ_GAUGE, 2, oil1);         //Disiplays Oil Press on horizontal gauge (Form 0)
      
}

void batteryVoltage() {
  
  mainBatteryRead.update();                                             //Updates Main Battery Value
  mainBatteryValue = mainBatteryRead.getValue();                       //Reads signal between 0 - 1023 from Analogue Pin 1 (YELLOW WIRE)
  mainBatteryVoltage = 10*mainBatteryValue*(mainBatteryDivider/1023.0); //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, mainBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 0, Engine Monitor Screen)
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, mainBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Mangement Screen) 
   
  auxBatteryRead.update();                                              //Updates Aux Battery Value
  auxBatteryValue = auxBatteryRead.getValue();                          //Reads signal between 0 - 1023
  auxBatteryVoltage = 10*auxBatteryValue*(auxBatteryDivider/1023.0);    //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, auxBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 0, Engine Monitor Screen)
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, auxBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Mangement Screen) 
   
  solarRead.update();                                                   //Updates Solar Battery value
  solarValue = solarRead.getValue();                                    //Reads singal between 0 - 1023
  solarVoltage = 10*solarValue*(solarDivider/1023.0);                   //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 13, solarVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Management Screen)

  //Serial.println("Solar Battery ");
  //Serial.println(solarValue);
  //Serial.println(solarDivider);

  //Serial.println("Main Battery");
  //Serial.println(mainBatteryValue);
  //Serial.println(mainBatteryDivider);

  //Serial.println("Aux Battery ");
  //Serial.println(auxBatteryVoltage);
}

void batteryCharger() {

//  for (int i = 0; i < 1000; i++) {

//      auxBatteryCurrent = auxBatteryCurrent + (0.044*analogRead(A0) - 13.51) / 1000;      //Current Sensor
//  }


  if (digitalRead(chargerLED) == HIGH) {
        
      genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 1);     //If pin is High - battery is charging
    
  }
      genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0);    //If pin is Low - battery is not charging
      
}


void exhaustTemp() {
  //Reading Exhaust Temperature from Digital pins 7,6,and 5
  exhaustTemperature = thermocouple.readCelsius();

    //Writing to touch screen display
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, exhaustTemperature);    //Displays EGT as Digits (Form 0)
    genie.WriteObject(GENIE_OBJ_GAUGE, 1, exhaustTemperature);         //Displays EGT on horizontal gauge (Form 0)

    while (exhaustTemperature > 150) {

        turboHot = true;

    }

        turboHot = false;


    //Serial.println("Exhaust Temp ");
    //Serial.println(exhaustTemperature);
}

void mapSensor () {
  //Reading Analogue Pin 1  
        pressureValue = analogRead(A1);               //Reads signal between 0 - 1023 from Analogue Pin 1 (YELLOW WIRE)
       

         boostPress =(((pressureValue/1023.0)+0.04)/0.004) * 0.145; //by 0.145 to calc psi 
         boostPress2 = boostPress - 1;

         if (boostPress2 < 0) {
          boostPress2 = 0;
         } else {
          boostPress2 = boostPress2;
         }
        
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, boostPress2); //Displays MAP on LED Digits (Form 0)
          genie.WriteObject(GENIE_OBJ_GAUGE, 0, boostPress2);      //Displays MAP on horizontal gauge (Form 0)

          //Serial.println("Map ");
          //Serial.println(boostPress2);
} 
    
// Event Handler for comms from Screen
 void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below
  
  
  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a WIN Button
    {
      if (Event.reportObject.index == 9)                              // If Button (Index = 9)
      {

        turboHot = false;                                             //Overide the turbo timer if OFF button is pressed
        digitalWrite(startPin, LOW); 
        
      }   
        
    }
   if (Event.reportObject.object == GENIE_OBJ_USERBUTTON)
    {
      if (Event.reportObject.index == 2)
      {
        if  (contrast == dayContrast) 
        {
          genie.WriteContrast(nightContrast);
          contrast = nightContrast;
          
        } else if (contrast == nightContrast) {
          
          genie.WriteContrast(dayContrast);
          contrast = dayContrast;
          
        }
          
        }
    
      }

    }

  } 

void turboTimer () {

  if (turboHot == true && ignitionOn == true) {

      return;
    
  } else if (ignitionOn == true && turboHot == true) {
                                                  
      //Turbo is at a temperature greater than 150
      digitalWrite(startPin, HIGH);             //Write startPin High supplying 12V to Start circuit

      Serial.println("Turbo is hot, car is on");
      
  } else if (ignitionOn == false && turboHot == true) {

      digitalWrite(startPin, HIGH);
      genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0);      //Turn on the Turbo Timer form

      Serial.println("Turbo is hot, ignition is off");
      
  } else if (ignitionOn == false && turboHot == false) {

  
      digitalWrite(startPin, LOW);

      Serial.println("Turbo is cool, ignition is off, car should be off");
    
  }
  
}

  /********** This can be expanded as more objects are added that need to be captured *************
  *************************************************************************************************
  Event.reportObject.cmd is used to determine the command of that event, such as an reported event
  Event.reportObject.object is used to determine the object type, such as a Slider
  Event.reportObject.index is used to determine the index of the object, such as Slider0
  genie.GetEventData(&Event) us used to save the data from the Event, into a variable.
  *************************************************************************************************/

void ignitionCheck () {

  if  (ignitionOn == true && screenReset == true && digitalRead(ignitionPin) == HIGH) {

       //Serial.print(ignitionOn);
       return;

  } else if  (digitalRead(ignitionPin) == HIGH) {
        
          digitalWrite(screen, HIGH);
          ignitionOn = true;
          
          pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
          digitalWrite(RESETLINE, 1);  // Reset the Display via D4
          delay(2000);
          digitalWrite(RESETLINE, 0);  // unReset the Display via D4
          delay(5000); //let the display start up after the reset (This is important)

          screenReset = true;
          
    
          
    } else if (digitalRead(ignitionPin) == LOW && turboHot == false) {

          digitalWrite(screen, LOW);    //Turn off screen
          ignitionOn = false;
          screenReset = false;
         
    }
  
}

void dateTime () {

DateTime now = rtc.now();

    datemonth = now.month();
    dateday = now.day();
    datehour = now.hour();
    dateminute = now.minute();

    monthday = dateday*100 + datemonth;
    hourminute = datehour*100 + dateminute;

    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, monthday);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, hourminute);

}

