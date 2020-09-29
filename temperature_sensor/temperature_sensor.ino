/*
 * @author: Dhruv Parikh
 * Date: 29th September 2020
 * Project Title: Acrylic Bending Machine
 * Purpose: Temperature State Feedback and control using PI controller
 * Loop Frequency:
*/

//Variable Declarations
//I/Os
#define pb 2
#define heater 3
#define temp_sensor A0 
#define LED_G 12
#define LED_R 13
#define set_temperature A1

bool state; //state of pushbutton
bool machine = 0; //state of machine on/off
float temperature; //temperature reading from sensoe
float pwm; //PWM to Motor driver
float temp_set;
//Control Parameters
float temp_d = 160; //desired temperature
float error = 0; //Error
float error_i = 0; //Integral Error

/*
Time Values
*/
float start,end,dt;

//PID function block
float PID(float,float,float);


//Intialisation Loop
void setup()
{
  pinMode(pb, INPUT);
  pinMode(heater,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_R,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  //capturing state of pb
  state = digitalRead(pb);
  //if pb is pressed, inverse machine 
  if(state==1){
    while(digitalRead(pb)==1);
    machine = !machine;
  }
  //when machine is high
  if(machine==1){
    digitalWrite(LED_R,LOW);
    digitalWrite(LED_G,HIGH);
    temp_d = 0.244*analogRead(set_temperature)+50;
    
    //Finding Loop Frequency
    end = start;
    start = millis();
    dt = (end-start)/1000;

    //Temperature reading
    temperature = analogRead(temp_sensor);
    temperature = (0.494*temperature)-49.88;
    Serial.print("Temperature:");
    Serial.println(temp_d);

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
    digitalWrite(LED_G,LOW);
    digitalWrite(LED_R,HIGH);
    analogWrite(heater,0);
    Serial.println("Machine off!");
  }
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
