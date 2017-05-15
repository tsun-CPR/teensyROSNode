#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>

#define buttonPin 2
#define ledPin 13

#define roomba Serial1
#define euclid Serial2

// Roomba serial commands
#define R_RESET     0x07 //7
#define R_START     0x80 //128
#define R_STOP      0xAD //173
#define R_MODE_FULL 0x84 //132
#define R_MOTOR     0x8A //138

// Roomba drive methods
#define R_DRIVE        0x89 //137 vel+rad
#define R_DRIVE_DIRECT 0x91 //145 Rvel+Rvel
#define R_DRIVE_PWM    0x92 //146 RPWM+LPWM

//Roomba packet ID's
#define left_encoder_packet_ID 43
#define right_encoder_packet_ID 44

#define delay_time  10

bool fullmode = 0;
float linearlow = 0.0;
float linearhigh = 0.0;
float angularlow = 0.0;
float angularhigh = 0.0;

/*************************************************************
  NODES
*************************************************************/
ros::NodeHandle twist_nh;
geometry_msgs::Twist debugtwist;
ros::Publisher twistDebug("/turtledebug/cmd_vel", &debugtwist);

std_msgs::Int32 lastLeftEncod;
std_msgs::Int32 lastRightEncod;

std_msgs::Int32 leftEncod;
std_msgs::Int32 rightEncod;
ros::Publisher leftEncoderPub("/R_left_encode", &leftEncod);
ros::Publisher rightEncoderPub("/R_right_encode", &rightEncod);


void twist_CB(const geometry_msgs::Twist& twistmsg) {
  //Serial.println(twistmsg.linear.x);
  roombaDrive(R_DRIVE, twistmsg.linear.x, twistmsg.angular.z);
  getEncoderData();
}

void hexSplitForRoomba(int num, int& a1, int& a2) {
  num = num & 0xFFFF;
  a1 = (num >> 8) & 0xFF;
  a2 = (num & 0xFF);
}

void roombaDrive(int op_code, int lin, int ang) {
  //vel is in mm/s
  //rad positive is left turn(CCW), negative is right(CW)
  int b1 = 0;
  int b2 = 0;
  int b3 = 0;
  int b4 = 0;
  flash();

  int vel = map(lin, -1, 1, -200, 200);
  int rad = map(ang, -1, 1, -200, 200);

  //  //debug----
  //  debugtwist.linear.x = vel;
  //  debugtwist.angular.z = rad;
  //  twistDebug.publish(&debugtwist);
  //  //debug----

  hexSplitForRoomba(vel, b1, b2);
  hexSplitForRoomba(rad, b3, b4);

  if (lin == 0 && rad != 0) {
    int velright = vel;
    int velleft = vel;
    op_code = R_DRIVE_DIRECT;
    velright += map(ang, -1, 1, -100, 100);
    velleft -= map(ang, -1, 1, -100, 100);
    hexSplitForRoomba(velright, b1, b2);
    hexSplitForRoomba(velleft, b3, b4);
  }

  if (lin == 0 && ang == 0) {
    b1 = b2 = b3 = b4 = 0x00;
  }

  //debugWrite(op_code, b1, b2, b3, b4);

  roomba.write(byte(op_code));
  //delay(delay_time);
  roomba.write(byte(b1));
  //delay(delay_time);
  roomba.write(byte(b2));
  //delay(delay_time);
  roomba.write(byte(b3));
  //delay(delay_time);
  roomba.write(byte(b4));
  //delay(delay_time);
}

void flash() {
  digitalWrite(ledPin, HIGH);
  delay(2);
  digitalWrite(ledPin, LOW);
  delay(2);
}
///////////////////////////////////////
int getSensorData(byte sensorID)
{
  int returnVal;
  byte packetID = 0;

  byte MSB = 0;
  byte LSB = 0;
  roomba.write(142);
  roomba.write(packetID);

  while (!roomba.available());
  MSB = roomba.read();
  LSB = roomba.read();
  returnVal = (MSB << 7) | LSB;


  return returnVal;
}
//////////////////////////////////////

void captureEncoderData(std_msgs::Int32& encod) {
  uint16_t highbyte;
  uint16_t lowbyte;
  highbyte = roomba.read();
  //delay(delay_time);
  lowbyte = roomba.read();
  //delay(delay_time);

  encod.data = (int)(highbyte << 0xFF) | (int)(lowbyte & 0xFF);
}

void startEncodStream() {
  roomba.write(byte(148));
  roomba.write(byte(2));
  roomba.write(byte(left_encoder_packet_ID));
  roomba.write(byte(right_encoder_packet_ID));
  delay(100);
}

void getEncoderData() {
//  signed char sensorbytes[4];
//  int data = 1;
  /*signed 16bit number, high byte first. This number will roll
    over if it passes the max value at approx 14.5m*/
//  roomba.write(byte(149)); // request encoder counts
//  roomba.write(byte(2));
//  roomba.write(byte(43));
//  roomba.write(byte(44));
//  delay(10); // wait for sensors
//  int i = 0;
//  while (data == 1) {
//    sensorbytes[i++] = roomba.read();  //read values into signed char array
//    if (i == 3) {
//      data = 0;
//    }
//  }

  //merge upper and lower bytes
//  rightEncod.data = (int)(sensorbytes[2] << 8) + (int)(sensorbytes[3]);
//  leftEncod.data = (int)(sensorbytes[0] << 8) + (int)(sensorbytes[1]);

  leftEncod.data = getSensorData(43);
  rightEncod.data = getSensorData(44);

  if (abs(leftEncod.data - lastLeftEncod.data) < 100 || 
      abs(rightEncod.data - lastRightEncod.data) < 100 ) {
    lastLeftEncod = leftEncod;
    lastRightEncod = rightEncod;
    leftEncoderPub.publish(&leftEncod);
    rightEncoderPub.publish(&rightEncod);
      }

 
  //  }
  //  lastLeftEncod = leftEncod;
  //  lastRightEncod = rightEncod;
}

ros::Subscriber<geometry_msgs::Twist> twistsub("/cmd_vel_mux/input/teleop", &twist_CB);
//ros::Subscriber<geometry_msgs::Twist> twistsub("/turtle1/cmd_vel", &twist_CB); //for testing with turtle_teleop_key
/*************************************************************
  SETUP
*************************************************************/
void setup() { // will be triggered when robot is powered on
  Serial.begin(115200);
  //Serial.println("Start setup");
  delay(2000); // Needed to let the robot initialize
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // start HW1 serial
  roomba.begin(115200);
  // start HW2 serial

  // start USB serial port
  twist_nh.initNode();
  twist_nh.subscribe(twistsub);
  twist_nh.advertise(twistDebug);
  twist_nh.advertise(leftEncoderPub);
  twist_nh.advertise(rightEncoderPub);

  delay(500);
  roomba.write(R_RESET); // reset Roomba, to clear all current states
  flash();
}

/*************************************************************
  DEBUG
*************************************************************/
void debugWrite(int code, int a, int b, int c, int d) {
  //Serial.printf("%f, %f, %f, %f, %f\n", code, a, b, c, d);
}
/*************************************************************
  LOOP
*************************************************************/
void loop() {
  //  if (roomba.available() > 0) {
  //    Serial.write(roomba.read());
  //  }
  //  if (Serial.available() > 0) {
  //    roomba.write(Serial.read());
  //  }

  if (digitalRead(buttonPin) == LOW && fullmode == 0) {
    //Serial.println("Setting to Full mode");
    roomba.write(R_START); // This command starts the OI.
    delay(100);
    roomba.write(R_MODE_FULL); // set mode to full (see p.7 of OI manual)
    delay(100);
    roomba.write(R_MOTOR); // set motor power
    delay(100);
    roomba.write(0b00000100); // ^ brush motor full (sends power to on-board computer)
    delay(100);
    //Serial.println("ROOMBA IN FULL");
    digitalWrite(ledPin, HIGH);
    delay(500);
    fullmode = 1;
    Serial.flush();
    //Serial.begin(9600);
    startEncodStream();
    flash();
    delay(10);
    flash();
  }

  if (fullmode == 1) {
    //spin other nodes here
    twist_nh.spinOnce();
    delay(10);
  }
}
