 /*
 * @author: Dhruv Parikh
 * Date: 25th October 2020
 * Project Title: Acrylic Bending Machine
 * Purpose: Temperature State Feedback and control using P controller
 * Loop Frequency: 4Hz
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
#define LED_R 11
#define LED_G 12

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
float error_i = 0; //Integral Error

/*
Time Values
*/
float start,time_prev,dt;


//display functions
void max_power(float);
void controller_off(float);
void extreme_temp(float);

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
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
  
  delay(500);
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,HIGH);
  temperature = thermocouple.readCelsius();
  intro();
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
  analogWrite(heater,10);  
}

void loop()
{
      //Finding Loop Frequency
    time_prev = start;
    start = millis();
    dt = (start-time_prev)/1000;

  //Temperature for thermocouple
  temperature = thermocouple.readCelsius();
  
  //getting temperature from potentiometer
  if(analogRead(knob)<273)
  {
    temp_d = -100;
    controller_off(temperature);
    pwm = 0;
  }
  else if(analogRead(knob)>800){
    temp_d = -100;
    max_power(temperature);
    pwm = 188;
  }
  else{
    temp_d =0.104*analogRead(knob)+76.6; 
    error = temp_d - temperature;
    error_i+=error*dt;

    //pwm heater with minimum value 100
    pwm = 110+PID(error,error_i);
    
    if(abs(PID(error,error_i))>100){
      error_i=0;
    }
    if(pwm>188){
      pwm=188;
    }
    if(pwm<110){
      pwm=110;
    }
    //Displaying on OLED
    display_temperature(temp_d,temperature);
  }
  if(temperature>200){
    pwm = 0;
    extreme_temp(temperature);  
  }
  //Writing Values to driver
  analogWrite(heater,pwm);

  if(abs(temperature-temp_d)<5){
    digitalWrite(LED_G,LOW);
    digitalWrite(LED_R,HIGH);
  }
  else{
    digitalWrite(LED_G,HIGH);
    digitalWrite(LED_R,LOW);
  }
  
  //Delay of 250 ms for matching thermocouple clock speed
  delay(250);
}

float PID(float error,float error_i){
  /*
  PID Params
  Input: Error
  Output: 8-bit PID PWM value from 0,255 
  Purpose: Calculates PID from errors, set the gain values
  as requried

  */

  //proportional
  float Kp = 3.75; //0.7;
  float Ki = 0.05;
  if(error<0){
    Kp = 0.5;
  }
  //PID output
  float pid = Kp*error+Ki*error_i;
 
  //Saturation
  if(pid>=105){
    pid = 105;
  }
  
  else if(pid<=0){
    pid = 0;
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
  display.println("Welcome...");
  display.display();
  delay(1500);
/*  
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
*/
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
  int cur = (int) current;
  int set_temp = (int) set;
  display.clearDisplay();
  display.setCursor(0, 0);
  
  // Display static text
  display.println("Set:");
  display.setCursor(0, 20);
  display.println(set_temp);
  display.setCursor(0, 40);
  display.print ("Wire:");
  display.println(cur);
  display.display();
} 

void controller_off(float temp){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("HEAT OFF");
  display.setCursor(0, 20);
  int disp = (int) temp;
  display.print("Wire:");
  display.println(temp);
  display.display();
}

void max_power(float temp){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MAX POWER");
  display.setCursor(0, 20);
  int disp = (int) temp;
  display.print("Wire:");
  display.println(temp);
  display.display();
}

void extreme_temp(float temp){
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,HIGH);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("OVERHEAT!");
  display.display();
  delay(200);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW); 
  display.clearDisplay();
  display.display();
  delay(200);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,HIGH);
  
  display.setCursor(0, 0);
  display.println("OVERHEAT!");
  display.setCursor(0, 20);
  int disp = (int) temp;
  display.print("Wire:");
  display.println(temp);
  display.display();
  delay(500);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
}  
