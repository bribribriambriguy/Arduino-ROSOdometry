#include <PID_v2.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the accumulated ticks for each wheel using the 
 * built-in encoder (forward = positive; reverse = negative) 
 */

ros::NodeHandle nh;

std_msgs::Int16 right_wheel_tick_count_pub;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count_pub);
 
std_msgs::Int16 left_wheel_tick_count_pub;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count_pub);

// PID parameters
double setpointLeft=0, inputLeft, outputLeft, setpointRight=0, inputRight, outputRight;

PID motorLeft(&inputLeft, &outputLeft, &setpointLeft, 0, 20,0, DIRECT);
PID motorRight(&inputRight, &outputRight, &setpointRight, 0, 20,0, DIRECT);
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_LEFT_B 4

#define ENC_IN_RIGHT_A 3
#define ENC_IN_RIGHT_B 11

int oldpositionLeft = 0;
int oldpositionRight = 0;

int oldtime = 0;
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.


const int TICKS_PER_METER = 661;
double velLeftWheel = 0;

double leftWheelReq = 0;
double rightWheelReq = 0;

/*
const int enA = 5;
const int in1 = 6;
const int in2 = 7;
*/

// Motor A connections
const int enA = 9;
const int in1 = 5;
const int in2 = 6;
  
// Motor B connections
const int enB = 10; 
const int in3 = 7;
const int in4 = 8;
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

void setMotorSpeeds(const geometry_msgs::Twist& cmdvel){
   // Record the time that the last velocity command was received
   lastCmdVelReceived = (millis()/1000);
   leftWheelReq = cmdvel.linear.x - (cmdvel.angular.z * (0.13335/2.0));
   rightWheelReq = cmdvel.linear.x + (cmdvel.angular.z * (0.13335/2.0));

   setpointLeft = ((leftWheelReq * 60.0)/0.2042)*-1;
   setpointRight = ((rightWheelReq * 60.0)/0.2042)*-1;
}

ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", &setMotorSpeeds);
 
void setup() {
  // Open the serial port at 9600 bps
  
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT_PULLUP);

  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT_PULLUP);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  /*
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 150);
  */

  //motorLeft.SetMode(DIRECT);
  motorLeft.SetOutputLimits(-255,255);
  motorRight.SetOutputLimits(-255,255);
  motorLeft.SetMode(AUTOMATIC);
  motorRight.SetMode(AUTOMATIC);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subTwist);
}
 
void loop() {
nh.spinOnce();
int newpositionLeft = left_wheel_tick_count;
int newpositionRight = right_wheel_tick_count;
int newtime = millis();
/*
int ticks =  (65536 + newposition - oldposition) % 65536;

if(ticks > 10000){
    ticks = 0 - (65535 - ticks);
}
*/


inputLeft = (newpositionLeft - oldpositionLeft) * 1000.0/(newtime-oldtime);
inputRight = (newpositionRight - oldpositionRight) * 1000/(newtime-oldtime);

inputLeft = (inputLeft/135.0) * 60.0;
inputRight = (inputRight/135.0) * 60.0;

//input=vel;
motorLeft.Compute();
motorRight.Compute();

if (outputLeft > 0){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
}
else if(outputLeft < 0){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);  
}

if (outputRight > 0){
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}
else if(outputRight < 0){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  
}

analogWrite(enA, abs(outputLeft));
analogWrite(enB, abs(outputRight));
//Serial.print("Setpoint: ");
//Serial.println(setpoint);
//Serial.print("RPM: ");
//Serial.println(inputRight);

left_wheel_tick_count_pub.data = left_wheel_tick_count;
right_wheel_tick_count_pub.data = right_wheel_tick_count;

leftPub.publish( &left_wheel_tick_count_pub );
rightPub.publish( &right_wheel_tick_count_pub );

//Serial.print ("speed = ");
//Serial.println(setpoint);
//Serial.println (vel);
oldpositionLeft = newpositionLeft;
oldpositionRight = newpositionRight;
oldtime = newtime;

if((millis()/1000) - lastCmdVelReceived > 1){
    setpointLeft = 0;
    setpointRight = 0;
}

delay(100);
}
/*
void setMotorSpeeds(const geometry_msgs::Twist& cmdvel){
   leftWheelReq = cmdvel.linear.x - (cmdvel.angular.z * (0.13335/2.0));
   rightWheelReq = cmdvel.linear.x + (cmdvel.angular.z * (0.13335/2.0));

   setpointLeft = (leftWheelReq * 60.0)/0.2042;
   setpointRight = (rightWheelReq * 60.0)/0.2042;
}
*/

void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count == encoder_maximum) {
      left_wheel_tick_count = encoder_minimum;
    }
    else {
      left_wheel_tick_count++;  
    }  
  }
  else {
    if (left_wheel_tick_count == encoder_minimum) {
      left_wheel_tick_count = encoder_maximum;
    }
    else {
      left_wheel_tick_count--;  
    }   
  } 
}

void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count == encoder_maximum) {
      right_wheel_tick_count = encoder_minimum;
    }
    else {
      right_wheel_tick_count++;  
    }    
  }
  else {
    if (right_wheel_tick_count == encoder_minimum) {
      right_wheel_tick_count = encoder_maximum;
    }
    else {
      right_wheel_tick_count--;  
    }   
  }
}


/*
void calc_speed(){
// Previous timestamp

  int newtime = millis();
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count - prevLeftCount) % 65535;
  
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }

 
  // Calculate wheel velocity in meters per secon
  Serial.println(numOfTicks/TICKS_PER_METER/((newtime/1000)-prevTime));
  //Serial.println(velLeftWheel);
 
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count;
 
  // Update the timestamp
  prevTime = newtime;  
}
*/
