 /*
 * @author: Dhruv Parikh
 * Date: 25th September 2020
 * Project Title: Acrylic Bending Machine
 * Purpose: Temperature State Feedback and control using P controller
 * Loop Frequency:
*/

//Max 6675 Adafruit Library 
//Downloaded from https://robojax.com/learn/arduino/?vid=robojax_MAX6675_thermocouple
#include "max6675.h"
#include <Wire.h>
//Also requires Adafruit_BusIO Library - download all from master
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//Variable Declarations
//I/Os
#define heater 3 
#define LED_R 2
#define LED_G 13
#define knob A0
  
//Defining MAX6675 Pins
#define SCK 6
#define CS 5
#define SO 4

float temperature; //temperature reading from sensoe
float pwm; //PWM to Motor driver
//Control Parameters
float temp_d = 160; //desired temperature
float error = 0; //Error

//PID function block
float PID(float);

//Intro Function
void intro();

//display temperature
void display_temperature(float,float);

//init for thermocouple instance
MAX6675 thermocouple(SCK, CS, SO);


//Intialisation Loop
void setup()
{
  pinMode(heater,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_R,OUTPUT);
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,HIGH);
  
  delay(500);
  temperature = thermocouple.readCelsius();
  intro();
  analogWrite(heater,10);  
}

void loop()
{
  digitalWrite(2,0);
  //Temperature for thermocouple
  temperature = thermocouple.readCelsius();
  temperature = (int) temperature;

  //getting temperature from potentiometer
  temp_d =0.078*analogRead(knob)+75; 
  temp_d = (int) temp_d;  

  //pwm heater with minimum value 100
  pwm = 100+PID(temp_d);

  //Displaying on OLED
  display_temperature(pwm,temperature);

  //Writing Values to driver
  analogWrite(heater,pwm);

  //Delay of 250 ms for matching thermocouple clock speed
  delay(250);
}

float PID(float error){
  /*
  PID Params
  Input: Error
  Output: 8-bit PID PWM value from 0,255 
  Purpose: Calculates PID from errors, set the gain values
  as requried

  */

  //proportional
  float Kp = 0.7;

  //PID output
  float pid = Kp*error;
  
  //Saturation
  if(pid>=88){
    pid = 88;
  }
  else if(pid<=0){
    pid = 10;
  }

  //Return after saturation signal
  return pid;
}

void intro(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  //X,Y
  display.setCursor(0,20);
  display.println("Robust");
  display.display();
  delay(750);
  
  display.clearDisplay();
  display.setCursor(0,20);
  display.println("Asthetic");
  display.display();
  delay(750);

  display.clearDisplay();
  display.setCursor(0,20);
  display.println("Precise");
  display.display();
  delay(750);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Acrylic");
  display.setCursor(0,20);
  display.println("Bending");
  display.setCursor(0,40);
  display.println("Machine");
  display.display();
  delay(2000);
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,20);
  display.println("Controller");
  display.setCursor(0,40);
  display.println("ON");
  display.display();
  delay(1000);
}


void display_temperature(float set, float current){
  current = (int) current;
  set = (int) set;
  display.clearDisplay();
  display.setCursor(0, 0);
  
  // Display static text
  display.println("Set:");
  display.setCursor(0, 20);
  display.println(set);
  display.setCursor(0, 40);
  display.print ("Wire: ");
  display.println(current);
  display.display();
} 
