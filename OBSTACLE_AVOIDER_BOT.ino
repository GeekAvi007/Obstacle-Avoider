/* © Robocell,CCA */
/* This code is given during ROBOZIDO robotics workshop organised by Robocell,CCA on 28 & 29th January,2023. */
/* Follow us on https://www.facebook.com/ccanitd.in/ 
/* For queries/help mail us at robocell@ccanitd.in 
/* Visit us at-www.ccanitd.in*/

#include <Servo.h>          // Including library for Servo
#include <NewPing.h>

/* Defining motor control pins */
#define left_motor_forward    D1   // D1->IN1
#define left_motor_backward   D2   // D2->IN2
#define right_motor_backward  D3   // D3->IN3
#define right_motor_forward   D4   // D4->IN4

#define trigger_pin  D7     // Ultrasonic sensor pins
#define echo_pin     D8

#define MAX_DISTANCE 250

Servo ultrasonic_servo_motor;
float distance = 100.0;   // This will store the distance measured by the ultrasonic sensor

void setup() 
{
  pinMode(left_motor_backward, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);

  // pinMode(D5, OUTPUT);    // As a preventive measure, since in some might give unexpected behaviour
  // digitalWrite(D5, LOW);  // Uncomment if arm gives unexpected behaviour

  ultrasonic_servo_motor.attach(D6);
  ultrasonic_servo_motor.write(90);   // U.S. sensor faces front

  NewPing sonar(trigger_pin, echo_pin, MAX_DISTANCE);
  Serial.begin(9600);   // Setting the up the Serial Monitor
}

void loop() 
{
  distance = measure_distance();

  while(distance<=30)
  { /*  If forward distance is less than 30 cms, then
        wait for 50 ms and check distance at right side and update the distance.
        If the distance is greater than 30 cms, then turn to right side
        else if distance is greater than 5 cms, turn to left side,
        else go backwards. */                         
    delay(50);
    ultrasonic_servo_motor.write(0);    // Ultrasonic sensor faces right side
    delay(150);
    distance = measure_distance();
  
    ultrasonic_servo_motor.write(90);   // Ultrasonic sensor is back to position i.e., faces front
    delay(150);
    
    if(distance>30)   // If right distance is greater than 30 cm than turn right
    {
      Right(180);
      delay(275);
    }
    else if( distance>5 and distance<30)    // If right distance is not greater than 30 cm it turns left
    {
      Left(180);
      delay(275);
    }
    else{   // If distance is less than 5 cms, it goes backwards
      Backward(180);
      delay(200);
    }
  }

  distance = measure_distance();

  while(distance>30)  
  { /* Untill the forward distance is greater than 30 cms it goes forward */
    ultrasonic_servo_motor.write(90);   // U.S. sensor is at original position ,i.e faces front
    delay(150);
    Forward(125);
    distance = measure_distance();    // update the distance
  }
}

float measure_distance()    // This function measures the distance of obstacle from the Ultrasonic sensor
{
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  float duration = pulseIn(echo_pin, HIGH);   // Calculates after how much time we are getting the response
  float distance = (duration * 0.034) / 2;    // Speed of Sound=340m/s i.e. 0.034cm/microseconds

  Serial.print("Distance=");
  Serial.print(distance);
  Serial.print(" ");

  return distance;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************** MOTOR FUNCTIONS ***********************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*************************************************** FORWARD ***************************************************/

void Forward(int speed)
{
  analogWrite(left_motor_forward,speed);
  analogWrite(right_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  Serial.println("Forward//////");
}

/*************************************************** BACKWARD **************************************************/
void Backward(int speed)
{
  analogWrite(left_motor_backward,speed);
  analogWrite(right_motor_backward,speed);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
  Serial.println("Backward//////");
}

/************************************************** TURN LEFT **************************************************/
void Left(int speed)
{
  analogWrite(right_motor_forward,speed);
  analogWrite(left_motor_backward,speed);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  Serial.println("Left//////");
}

/************************************************** TURN RIGHT *************************************************/
void Right(int speed)
{
  analogWrite(left_motor_forward,speed);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  analogWrite(right_motor_backward,speed);
  Serial.println("Right//////");
}
/**************************************************** STOP *****************************************************/
void Stop()
{
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_forward,LOW);
  digitalWrite(right_motor_backward,LOW);
}
