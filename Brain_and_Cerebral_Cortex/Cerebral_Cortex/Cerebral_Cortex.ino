

/*    NOTE ON CLASHING LIBRARIES:
 *    NewPing seems to clash with PID_v1
 *    Not found any online reference to this secific clash, but seems likely due to timer use
 *    Result is that ultrasonics cannot be used within PID loops
 */

#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include "CmdMessenger.h"

#define CENTER_TRIGGER_PIN  10
#define CENTER_ECHO_PIN     11
#define LEFT_TRIGGER_PIN  8
#define LEFT_ECHO_PIN     9
#define RIGHT_TRIGGER_PIN  12
#define RIGHT_ECHO_PIN     13
#define MAX_DISTANCE 200

NewPing centerSonar(CENTER_TRIGGER_PIN, CENTER_ECHO_PIN, MAX_DISTANCE);
NewPing leftSonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing rightSonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
Servo mytiltservo;  // create tilt servo object to control a servo
Servo mypanservo;  // create pan servo object to control a servo

const byte right_encoder = 18;
const byte left_encoder = 19;

const int rightForward = 2;
const int rightBackward = 4;
const int rightEnable =5;

const int leftForward = 3;
const int leftBackward = 7;
const int leftEnable = 6;


int tiltpos = 90;    // variable to store the servo position
int panpos =90;
int centerDistance = 100;
int leftDistance = 100;
int rightDistance = 100;

int t1; // byte 1 from lidar
int t2; // byte 2 from lidar
int t3; // byte 3 from lidar
int t4; // byte 4 from lidar
int t5; // byte 5 from lidar
int t6; // byte 6 from lidar

int lidar_distance;
int lidar_strength;

const int BAUD_RATE = 9600;
const long LIDAR_BAUD_RATE = 115200;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
float rotX, rotY, rotZ;

float angelX = 0, angelY = 0, angelZ = 0;

long timePast = 0;
long timePresent = 0;

//  Objects for PID motor control
double left_pulse_count,left_abs_pulse_count;//the number of the pulses
double right_pulse_count,right_abs_pulse_count;
double left_total_distance;
double right_total_distance;
boolean left_result;  //boolean PID result
boolean right_result;
double left_val_output; //PID outputs - supplied to the motor PWM value.
double right_val_output;
double left_setpoint;
double right_setpoint;
double left_Kp=0.6, left_Ki=5, left_Kd=0; // PID gain settings  
double right_Kp=0.6, right_Ki=5, right_Kd=0;  

PID leftPID(&left_abs_pulse_count, &left_val_output, &right_setpoint, left_Kp, left_Ki, left_Kd, DIRECT); 
PID rightPID(&right_abs_pulse_count, &right_val_output, &left_setpoint, right_Kp, right_Ki, right_Kd, DIRECT); 
 
// Objects for CMD Messenger
enum {
    spin,
    anticlockwise,
    anticlockwise_until,
    clockwise,
    clockwise_until,
    backup,
    forward,
    forward_until,
    ping_all,
    pan_tilt_test,
    lidar_read,
    follow_right_wall,
    follow_left_wall,
    pings,
    lidar_val,
    error,
};

/* Initialize CmdMessenger -- this should match PyCmdMessenger instance */
CmdMessenger c = CmdMessenger(Serial,',',';','/');

void setup(){
  delay(200);
  pinMode(leftForward , OUTPUT);
  pinMode(leftBackward , OUTPUT);
  pinMode(rightForward , OUTPUT);
  pinMode(rightBackward , OUTPUT);
  pinMode(rightEnable , OUTPUT);
  pinMode(leftEnable , OUTPUT);

  Serial.begin(BAUD_RATE);
  Serial2.begin(LIDAR_BAUD_RATE);

//Gyro setup
  Wire.begin();
  setUpMPU();
  callibrateGyroValues();
  timePresent = millis();

//Encoders & PID setup
  leftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  rightPID.SetMode(AUTOMATIC);//PID is set to automatic mode
   
  rightPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
  leftPID.SetSampleTime(100);//Set PID sampling frequency is 100ms

  pinMode(left_encoder,INPUT);  
  pinMode(right_encoder,INPUT);
   
  attachInterrupt(4, left_wheel_int, CHANGE);
  attachInterrupt(5, right_wheel_int, CHANGE);
  
  attach_callbacks();  
}

void attach_callbacks(void) { 
    c.attach(spin,on_spin);
    c.attach(anticlockwise,on_anticlockwise);    
    c.attach(anticlockwise_until,on_anticlockwise_until_free);      
    c.attach(clockwise,on_clockwise);          
    c.attach(clockwise_until,on_clockwise_until_free);      
    c.attach(backup,on_backup);    
    c.attach(forward,on_forward);
    c.attach(forward_until,on_forward_until_object);    
    c.attach(ping_all,on_ping_all);
    c.attach(pan_tilt_test,on_pan_tilt_test);  
    c.attach(lidar_read,on_lidar_read);        
    c.attach(follow_right_wall,on_follow_right_wall);
    c.attach(follow_left_wall,on_follow_left_wall);   
    c.attach(on_unknown_command);
}


void loop() {
    c.feedinSerialData();
}


void on_unknown_command(void){
    c.sendCmd(error,"Command without callback.");
}


void on_anticlockwise() 
{                
  int turn_angle = c.readBinArg<int>();
  robot_anticlockwise_deg (turn_angle);
  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}          
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_clockwise() 
{     
  int turn_angle = c.readBinArg<int>();
  robot_clockwise_deg (turn_angle);
  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}     
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_ping_all()
{
  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 

  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}     
    
  // return sonar distances
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_forward_until_object()
{
  robot_fwd_until_object ();
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_forward()
{
  int requested_distance = c.readBinArg<int>();
  robot_fwd_dist (requested_distance);

  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}     
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_backup()
{
  int requested_distance = c.readBinArg<int>();
  robot_bkwd_dist (requested_distance);
 
  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}     
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_clockwise_until_free()
{
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , HIGH);
  analogWrite(leftEnable,240); 
  analogWrite(rightEnable,240);
  leftDistance =10;
  centerDistance =10;
  rightDistance =10;
  while(centerDistance < 40 || leftDistance < 30 || rightDistance < 30)
  {
    leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
    centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
    rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
    if (leftDistance == 0)
      leftDistance = 200;
    if (centerDistance == 0) 
      centerDistance = 200;
    if (rightDistance == 0) 
      rightDistance = 200;
  }
  analogWrite(leftEnable,0); 
  analogWrite(rightEnable,0);
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , LOW);  
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();
}


void on_anticlockwise_until_free()
{
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , HIGH);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);
  analogWrite(leftEnable,240); 
  analogWrite(rightEnable,240);
  leftDistance =10;
  centerDistance =10;
  rightDistance =10;
  while(centerDistance < 40 || leftDistance < 30 || rightDistance < 30)
  {
    leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
    centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
    rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
    if (leftDistance == 0)
      leftDistance = 200;
    if (centerDistance == 0) 
      centerDistance = 200;
    if (rightDistance == 0) 
      rightDistance = 200;
  }    
  analogWrite(leftEnable,0); 
  analogWrite(rightEnable,0);
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , LOW);
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();  
}


void on_spin(void){
  analogWrite(leftEnable,255); 
  analogWrite(rightEnable,255);
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , HIGH);

  delay(2500);

  analogWrite(leftEnable,255); 
  analogWrite(rightEnable,255);
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , HIGH);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);

  delay(2500);

  analogWrite(leftEnable,0); 
  analogWrite(rightEnable,0);
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , LOW);

  leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(5)))); 
  centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(5)))); 
  rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(5)))); 
  if (leftDistance == 0) { leftDistance = 200;} 
  if (centerDistance == 0) { centerDistance = 200;} 
  if (rightDistance == 0) { rightDistance = 200;}     

// return sonar distances
  c.sendCmdStart(pings);
  c.sendCmdBinArg(leftDistance);
  c.sendCmdBinArg(centerDistance);
  c.sendCmdBinArg(rightDistance);
  c.sendCmdEnd();

}


void on_pan_tilt_test(void){
  mytiltservo.attach(44);  // attaches the tilt servo on pin 44 to the servo object [44 = tilt]
  mypanservo.attach(45);  // attaches the pan servo on pin 45 to the servo object  [45 = pan]
  mytiltservo.write(50); 
  mypanservo.write(30);  


  for (tiltpos = 50; tiltpos <= 90; tiltpos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mytiltservo.write(tiltpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (panpos = 30; panpos <= 120; panpos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mypanservo.write(panpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
 
  for (tiltpos = 90; tiltpos >= 50; tiltpos -= 1) { // goes from 180 degrees to 0 degrees
    mytiltservo.write(tiltpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

   for (panpos = 120; panpos >= 30; panpos -= 1) { // goes from 180 degrees to 0 degrees
    mypanservo.write(panpos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  mytiltservo.detach();  // dettaches the tilt servo on pin 44 to the servo object [44 = tilt]
  mypanservo.detach();  // dettaches the pan servo on pin 45 to the servo object  [45 = pan]     
}


void on_lidar_read()
{
  return_lidar_values ();
}


void on_follow_right_wall()
{
  double remaining_distance;
  double start_odometer;
  int wall_offset = 85;
  int wall_error;
  int speed_offset;
  int Kp =4; // Proportional gain
  int left_speed;
  int right_speed;
  int reflective_timer;
  int far_target_timer;
  bool reflective_surface = false;
  bool far_from_target = false;
  double distance = c.readBinArg<double>();
  
  distance = distance * 154; // convert cm to pulse count
  remaining_distance = distance;
  start_odometer = left_total_distance;  // just need to use 1 encoder to measure distance

// Set pan / tilt to optimum angle position
  mytiltservo.attach(44);  // attaches the tilt servo on pin 44 to the servo object [44 = tilt]
  mypanservo.attach(45);  // attaches the pan servo on pin 45 to the servo object  [45 = pan]
  mytiltservo.write(80);  
  mypanservo.write(50);  
  
// Set motors for forward direction
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);

  lidar_distance = wall_offset;

  while (remaining_distance >=0)
  {
    lidar_distance = read_lidar_distance ();
    
    if (lidar_distance == -1)
       {lidar_distance = wall_offset;
       reflective_surface = true;
       mypanservo.write(15);
       wall_offset = 45;}      

    if (lidar_distance >= 140)
       {far_from_target = true;
       mypanservo.write(15);
       wall_offset = 45;}   
       
    if (reflective_surface == true)
       {reflective_timer+=1;}  

    if (far_from_target == true)
       {far_target_timer+=1;}  

    if((reflective_timer>=20)||(far_target_timer>=30)) // about 1 second / 1.5 seconds
       {mypanservo.write(50);
       reflective_timer=0;
       far_target_timer = 0;
       wall_offset = 85;
       reflective_surface = false;
       far_from_target = false;}  

    wall_error = wall_offset - lidar_distance;
    speed_offset = wall_error*Kp;

    left_speed = 240 - speed_offset;
    right_speed = 240 + speed_offset;

    if (left_speed >255) 
      {left_speed = 255;}
    if (right_speed >255) 
      {right_speed = 255;}

    if (left_speed <100) 
      {left_speed = 10;}
    if (right_speed <80) 
      {right_speed = 80;} 
        
    analogWrite(leftEnable,left_speed); 
    analogWrite(rightEnable,right_speed);
    delay (50);
    remaining_distance = start_odometer+distance - left_total_distance;
  }  
  robot_stop ();
  mytiltservo.detach();  // dettaches the tilt servo 
  mypanservo.detach();  // dettaches the pan servo    
  return_lidar_values (); // return lidar values to Brain
}


void on_follow_left_wall()
{
  double remaining_distance;
  double start_odometer;
  int wall_offset = 85;
  int wall_error;
  int speed_offset;
  int Kp =4; // Proportional gain
  int left_speed;
  int right_speed;
  int reflective_timer;
  int far_target_timer;
  bool reflective_surface = false;
  bool far_from_target = false;
  double distance = c.readBinArg<double>();

  distance = distance * 154; // convert cm to pulse count
  remaining_distance = distance;
  start_odometer = left_total_distance;  // just need to use 1 encoder to measure distance
  
// Set pan / tilt to optimum angle position
  mytiltservo.attach(44);  // attaches the tilt servo on pin 44 to the servo object [44 = tilt]
  mypanservo.attach(45);  // attaches the pan servo on pin 45 to the servo object  [45 = pan]
  mytiltservo.write(80);  
  mypanservo.write(115);  
  
// Set motors for forward direction
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);

  lidar_distance = wall_offset;

  while (remaining_distance >=0)
  {
    lidar_distance = read_lidar_distance ();
    
    if (lidar_distance == -1)
       {lidar_distance = wall_offset;
       reflective_surface = true;
       mypanservo.write(150);
       wall_offset = 45;}   // changed from 45

    if (lidar_distance >= 130)
       {far_from_target = true;
       mypanservo.write(150);
       wall_offset = 45;  // changed from 45
       }

    if (reflective_surface == true)
       {reflective_timer+=1;}  

   if (far_from_target == true)
       {far_target_timer+=1;}  
       
    if((reflective_timer>=20)||(far_target_timer>=30)) // about 1 second / 1.5 seconds
       {mypanservo.write(115);
       reflective_timer=0;
       far_target_timer = 0;
       wall_offset = 85;
       reflective_surface = false;
       far_from_target = false;}  
     
    wall_error = wall_offset - lidar_distance;
    speed_offset = wall_error*Kp;

    left_speed = 240 + speed_offset;
    right_speed = 240 - speed_offset;

    if (left_speed >255) 
      {left_speed = 255;}
    if (right_speed >255) 
      {right_speed = 255;}

    if (right_speed <100) 
      {right_speed = 10;}
    if (left_speed <80) 
      {left_speed = 80;} 
        
    analogWrite(leftEnable,left_speed); 
    analogWrite(rightEnable,right_speed);
    delay (50);
    remaining_distance = start_odometer+distance - left_total_distance;
  }  
  robot_stop ();
  //detach servos
  mytiltservo.detach();  // dettaches the tilt servo on pin 44 to the servo object [44 = tilt]
  mypanservo.detach();  // dettaches the pan servo on pin 45 to the servo object  [45 = pan]     
  return_lidar_values (); // return lidar values to Brain
}


/********************************/
/*    Lidar Functions   */
/********************************/

int read_lidar_distance()
{
  t1 = 0;
  t2 = 0;  
  while(Serial2.available() > 0)  // flush old data from lidar buffer
  {
    char t = Serial2.read();
  }  
  while(Serial2.available()<18)   // wait for 18 new bytes - ie at least one full message
  {
      delay(1);
  }
  while ((t1 != 0x59)||(t2!=0x59))
  {
    t1 = Serial2.read();
    t2 = Serial2.read();
  }
  t3 = Serial2.read(); //Byte 3
  t4 = Serial2.read(); //Byte 4

  t4 <<= 8;
  t4 += t3;
  
  return t4;
}


int read_lidar_strength()
{
  t1 = 0;
  t2 = 0;  
  while(Serial2.available() > 0)  // flush old data from lidar buffer
  {
    char t = Serial2.read();
  }  
  while(Serial2.available()<18)   // wait for 18 new bytes - ie at least one full message
  {
      delay(1);
  }
  while ((t1 != 0x59)||(t2!=0x59))
  {
    t1 = Serial2.read();
    t2 = Serial2.read();
  }
  t3 = Serial2.read(); //Byte 3
  t4 = Serial2.read(); //Byte 4
  t5 = Serial2.read(); //Byte 4
  t6 = Serial2.read(); //Byte 4

  t6 <<= 8;
  t6 += t5;

return t6;
}


void return_lidar_values ()
{
  lidar_distance = read_lidar_distance ();
  lidar_strength = read_lidar_strength ();

  c.sendCmdStart(lidar_val);
  c.sendCmdBinArg(lidar_distance);
  c.sendCmdBinArg(lidar_strength);
  c.sendCmdEnd();
}


/********************************/
/*    Gyro Support Functions    */
/********************************/

void setUpMPU() {
  // power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();  
}

void callibrateGyroValues() {
    for (int i=0; i<5000; i++) {
      getGyroValues();
      gyroXCalli = gyroXCalli + gyroXPresent;
      gyroYCalli = gyroYCalli + gyroYPresent;
      gyroZCalli = gyroZCalli + gyroZPresent;
    }
    gyroXCalli = gyroXCalli/5000;
    gyroYCalli = gyroYCalli/5000;
    gyroZCalli = gyroZCalli/5000;
}

void readAndProcessGyroData() {
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time
  
  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle  
}

void getGyroValues() {
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU 
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);                              // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
}

void getAngularVelocity() {
  rotX = gyroXPresent / 131.0;                                
  rotY = gyroYPresent / 131.0; 
  rotZ = gyroZPresent / 131.0;
}

void calculateAngle() {  
  angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000382;
}



/********************************************************************/
/*    Gyro, Encoder & PID Based Straight Line Movement Functions    */
/********************************************************************/

void robot_fwd_dist (double distance) // distance is in cm
{
  double initial_velocity = 210; // Set the velocity for the start of the run
  double pre_stop_velocity = 90; // Set the velocity for the start of the run
  double velocity; // current velocity
  double start_odometer;
  double remaining_distance;
  double Kp_heading = 6;  // Proportional gain for heading adjustment
  float heading;
  float heading_error;
  
  distance = (distance * 154);  
  readAndProcessGyroData();
  heading = angelZ;
  left_setpoint=right_setpoint=initial_velocity;
  start_odometer = left_total_distance;  //just need to use one encoder to measure distance
  remaining_distance = distance;
  velocity = initial_velocity;

  while(remaining_distance>=0)  
  {
      readAndProcessGyroData();      
      heading_error = angelZ-heading;
      left_setpoint = velocity - (heading_error*Kp_heading);   
      right_setpoint = velocity + (heading_error*Kp_heading);   
      robot_fwd();
      left_abs_pulse_count=abs(left_pulse_count);
      left_result=leftPID.Compute();//PID conversion is complete and returns 1
      if(left_result) {
        left_pulse_count = 0; } //Count clear, wait for the next count
      right_abs_pulse_count=abs(right_pulse_count);
      right_result=rightPID.Compute();//PID conversion is complete and returns 1
      if(right_result) {
        right_pulse_count = 0;} //Count clear, wait for the next count       
      if(remaining_distance<=1500){
         velocity=pre_stop_velocity;}
      remaining_distance= start_odometer+distance-left_total_distance;
  }
  robot_stop();
}
 

void robot_bkwd_dist (double distance) // distance is in cm
{
  double initial_velocity = 210; // Set the velocity for the start of the run
  double pre_stop_velocity = 90; // Set the velocity for the start of the run
  double velocity; // current velocity
  double start_odometer;
  double remaining_distance;
  double Kp_heading = 6;  // Proportional gain for heading adjustment
  float heading;
  float heading_error;
  
  distance = (distance * 154);  
  readAndProcessGyroData();
  heading = angelZ;
  left_setpoint=right_setpoint=initial_velocity;
  start_odometer = left_total_distance;  //just need to use one encoder to measure distance
  remaining_distance = distance;
  velocity = initial_velocity;

  while(remaining_distance>=0)  
  {
      readAndProcessGyroData();      
      heading_error = angelZ-heading;
      left_setpoint = velocity + (heading_error*Kp_heading);   
      right_setpoint = velocity - (heading_error*Kp_heading);   
      robot_bkwd();
      left_abs_pulse_count=abs(left_pulse_count);
      left_result=leftPID.Compute();//PID conversion is complete and returns 1
      if(left_result) {
        left_pulse_count = 0; } //Count clear, wait for the next count
      right_abs_pulse_count=abs(right_pulse_count);
      right_result=rightPID.Compute();//PID conversion is complete and returns 1
      if(right_result) {
        right_pulse_count = 0;} //Count clear, wait for the next count       
      if(remaining_distance<=1500){
         velocity=pre_stop_velocity;}
      remaining_distance= start_odometer+distance-left_total_distance;
  }
  robot_stop();
}
 

void robot_anticlockwise_deg(int turn_angle) 
{                
  double initial_speed = 170;
  double pre_stop_speed = 76;
  float inertia_angle = 2.9;
  float rotation_error;
  float end_angle;
  readAndProcessGyroData();
  end_angle = angelZ + turn_angle;
  rotation_error = end_angle - angelZ;
  left_setpoint = right_setpoint = initial_speed;
  while (rotation_error > inertia_angle)
  {      
    if (rotation_error < 45) 
      left_setpoint = right_setpoint = pre_stop_speed;          
    left_abs_pulse_count=abs(left_pulse_count);
    left_result=leftPID.Compute();//PID conversion is complete and returns 1
    if(left_result) {
      left_pulse_count = 0; } //Count clear, wait for the next count
    right_abs_pulse_count=abs(right_pulse_count);
    right_result=rightPID.Compute();//PID conversion is complete and returns 1
    if(right_result) {
      right_pulse_count = 0;} //Count clear, wait for the next count    
    robot_anticlkwise (); 
    readAndProcessGyroData();
    rotation_error = end_angle - angelZ;    
  } 
  robot_stop();
}


void robot_clockwise_deg(int turn_angle) 
{                
  double initial_speed = 170;
  double pre_stop_speed = 75;
  float inertia_angle = 2.9;
  float rotation_error;
  float end_angle;
  readAndProcessGyroData();
  end_angle = angelZ - turn_angle;
  rotation_error = angelZ - end_angle;
  left_setpoint = right_setpoint = initial_speed;
  while (rotation_error > inertia_angle)
  {      
    if (rotation_error < 45) 
      left_setpoint = right_setpoint = pre_stop_speed;          
    left_abs_pulse_count=abs(left_pulse_count);
    left_result=leftPID.Compute();//PID conversion is complete and returns 1
    if(left_result) {
      left_pulse_count = 0; } //Count clear, wait for the next count
    right_abs_pulse_count=abs(right_pulse_count);
    right_result=rightPID.Compute();//PID conversion is complete and returns 1
    if(right_result) {
      right_pulse_count = 0;} //Count clear, wait for the next count    
    robot_clkwise (); 
    readAndProcessGyroData();
    rotation_error = angelZ - end_angle;    
  } 
  robot_stop();
}


void robot_fwd_until_object () 
{
  int power =240;
  int left_power;
  int right_power;
  float heading;
  float heading_error;
  double Kp_heading = 14;  // Proportional gain for heading adjustment
  readAndProcessGyroData();
  heading = angelZ;
  leftDistance =100;
  centerDistance =100;
  rightDistance =100;
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);
  left_power = right_power = power;
  while((centerDistance >30) && (leftDistance>30) && (rightDistance>30))
  {
    readAndProcessGyroData();      
    heading_error = angelZ-heading;
    left_power = power + (heading_error*Kp_heading);   
    right_power = power - (heading_error*Kp_heading);   
    if (right_power>255)
      right_power = 255;
    if (left_power >255)
      left_power =255;
    analogWrite(leftEnable,left_power); 
    analogWrite(rightEnable,right_power);
    leftDistance = (leftSonar.convert_cm((leftSonar.ping_median(1)))); 
    centerDistance = (centerSonar.convert_cm((centerSonar.ping_median(1)))); 
    rightDistance = (rightSonar.convert_cm((rightSonar.ping_median(1)))); 
    if (leftDistance == 0) { leftDistance = 200;} 
    if (centerDistance == 0) { centerDistance = 200;} 
    if (rightDistance == 0) { rightDistance = 200;} 
  }
  robot_stop();
}


void robot_fwd()
{
  digitalWrite(leftForward,HIGH);
  digitalWrite(leftBackward,LOW);
  digitalWrite(rightForward,HIGH);
  digitalWrite(rightBackward,LOW);     
  analogWrite(leftEnable,left_val_output);
  analogWrite(rightEnable,right_val_output);
}


void robot_bkwd()
{
  digitalWrite(leftForward,LOW);
  digitalWrite(leftBackward,HIGH);
  digitalWrite(rightForward,LOW);
  digitalWrite(rightBackward,HIGH);     
  analogWrite(leftEnable,left_val_output);
  analogWrite(rightEnable,right_val_output);
}


void robot_anticlkwise()
{
  digitalWrite(leftForward , LOW);
  digitalWrite(leftBackward , HIGH);
  digitalWrite(rightForward , HIGH);
  digitalWrite(rightBackward , LOW);  
  analogWrite(leftEnable,left_val_output);
  analogWrite(rightEnable,right_val_output);
}


void robot_clkwise()
{
  digitalWrite(leftForward , HIGH);
  digitalWrite(leftBackward , LOW);
  digitalWrite(rightForward , LOW);
  digitalWrite(rightBackward , HIGH);  
  analogWrite(leftEnable,left_val_output);
  analogWrite(rightEnable,right_val_output);
}


void robot_stop()//Robot stops
{
  digitalWrite(leftForward,LOW);
  digitalWrite(leftBackward,LOW);
  digitalWrite(rightForward,LOW);
  digitalWrite(rightBackward,LOW); 
  digitalWrite(leftEnable, HIGH); 
  digitalWrite(rightEnable, HIGH);
}


void left_wheel_int()  //This is the ISR
{
  left_pulse_count++;
  left_total_distance++;
}


void right_wheel_int()  //This is the ISR
{
  right_pulse_count++;
  right_total_distance++;
}
