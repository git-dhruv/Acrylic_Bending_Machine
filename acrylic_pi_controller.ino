/*
 * @author: Dhruv Parikh
 * Date: 25th September 2020
 * Project Title: Acrylic Bending Machine
 * Purpose: Temperature State Feedback and control using PI controller
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
#define pb 2
#define heater 3 
#define LED_G 12
#define LED_R 13
#define knob A0

//Defining MAX6675 Pins
#define SCK 6
#define CS 5
#define SO 4

bool state; //state of pushbutton
bool machine = 0; //state of machine on/off
float temperature; //temperature reading from sensoe
float pwm; //PWM to Motor driver
//Control Parameters
float temp_d = 160; //desired temperature
float error = 0; //Error
float error_i = 0; //Integral Error
float alpha = 0.05;

/*
Time Values
*/
float start,time_prev,dt;

//PID function block
float PID(float,float,float);

//Intro Function
void intro();

//display temperature
void display_temperature(float,float);

//init for thermocouple instance
MAX6675 thermocouple(SCK, CS, SO);


//Intialisation Loop
void setup()
{
  pinMode(pb, INPUT);
  pinMode(heater,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_R,OUTPUT);
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  delay(500);
  temperature = thermocouple.readCelsius();
  delay(500);
  intro();
}

void loop()
{
  Serial.println(state);
  //Temperature for thermocouple
  temperature = alpha*temperature+(1-alpha)*thermocouple.readCelsius();
  temperature = (int) temperature;

  //getting temperature from potentiometer
  temp_d =0.244*analogRead(knob)+50; 
  temp_d = (int) temp_d;  
  
  display_temperature(temp_d,temperature);
  
  //capturing state of pb
  state = digitalRead(pb);
  //if pb is pressed, inverse machine 
  if(state==1){
    while(digitalRead(pb)==1);
    machine = !machine;
  }
  //when machine is high
  if(machine==1){
    digitalWrite(LED_R,HIGH);
    digitalWrite(LED_G,LOW);
    

    
    //Finding Loop Frequency
    time_prev = start;
    start = millis();
    dt = (start-time_prev)/1;
    
    /*Temperature FOR TINKERCAD
    temperature = analogRead(temp_sensor);
    temperature = (0.494*temperature)-49.88;
    Serial.print("Temperature:");
    Serial.println(temperature);
    */
    
    //Control Block
    error = temp_d - temperature;
    error_i+=error*dt;
    pwm = PID(error,error_i,0);

    //Writing Output
    analogWrite(heater,pwm);

    //Windup for PWM saturation
    if(pwm==0||pwm==255){
      error_i = 0;
    }
  }
  //When pb is pressed again
  else{
    digitalWrite(LED_G,HIGH);
    digitalWrite(LED_R,LOW);
    analogWrite(heater,0);
    Serial.println("Machine off!");
  }
  delay(250);
}

float PID(float error,float error_i,float error_d){
  /*
  PID Params
  Input: Error, Integral Error and Derivative Error
  Output: 8-bit PID PWM value from 0,255 
  Purpose: Calculates PID from errors, set the gain values
  as requried

  Note: Windup to be done in main loop
  */

  //integral
  float Ki = 0;
  //proportional
  float Kp = 0.98;
  //derivative
  float Kd = 0;
  //PID output
  float pid = Kp*error+Ki*error_i+Kd*error_d;

  //Saturation
  if(pid>=255){
    pid = 255;
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
}


void display_temperature(float set, float current){
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
