#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

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

#define R_DRIVE        0x89 //137 vel+rad
#define R_DRIVE_DIRECT 0x91 //145 Rvel+Rvel
#define R_DRIVE_PWM    0x92 //146 RPWM+LPWM

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


void twist_CB(const geometry_msgs::Twist& twistmsg) {

  //Serial.println(twistmsg.linear.x);
  roombaDrive(R_DRIVE, twistmsg.linear.x, twistmsg.angular.z);
}

void hexSplitForRoomba(int num, int& a1, int& a2) {
  num = num & 0xFFFF;
  a1 = (num >> 8) & 0xFF;
  a2 = (num & 0xFF);
}

//Not needed. Computer already in twos complement
//void velconfig(int vel, int& a1, int& a2) {
//  if (vel < 0) {
//    vel = vel & 255;
//  }
//  hexSplitForRoomba(vel, a1, a2);
//}

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

  int velright = vel;
  int velleft = vel;
//  if ( rad != 0 && lin == 0) { //left CCW R>L
//    velright += map(ang, -1, 1, -20, 20);
//    velleft -= map(ang, -1, 1, -20, 20);
//  }
//  else if ( rad > 0 && lin != 0 ) {
//    velright += map(ang, -1, 1, 0, 200);
//  }
//  else if ( rad < 0 && lin != 0 ) {
//    velleft += map(ang, -1, 1, 0, 200);
//  }

  //debug----
  debugtwist.linear.x = vel;
  debugtwist.angular.z = rad;
  twistDebug.publish(&debugtwist);
  //debug----
  
  hexSplitForRoomba(vel, b1, b2);
  hexSplitForRoomba(rad, b3, b4);

  if (lin == 0 && rad != 0) {
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

  roomba.write(op_code);
  delay(20);
  roomba.write(b1);
  delay(20);
  roomba.write(b2);
  delay(20);
  roomba.write(b3);
  delay(20);
  roomba.write(b4);
  delay(20);
}

//ros::Subscriber<geometry_msgs::Twist> twistsub("/cmd_vel_mux/input/teleop", &twist_CB);
ros::Subscriber<geometry_msgs::Twist> twistsub("/turtle1/cmd_vel", &twist_CB); //for testing with turtle_teleop_key
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
  //euclid.begin(0);
  // start USB serial port
  twist_nh.initNode();
  twist_nh.subscribe(twistsub);
  twist_nh.advertise(twistDebug);
  //twist_nh.getHardware()->setBaud(115200);

  //Serial.println("Initialize complete");
  delay(500);
  roomba.write(R_RESET); // reset Roomba, to clear all current states
  //Serial.println("Resetting Roomba");
  flash();
}

void flash() {
  digitalWrite(ledPin, HIGH);
  delay(5);
  digitalWrite(ledPin, LOW);
  delay(5);
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
  }

  if (fullmode == 1) {
    //spin other nodes here
    twist_nh.spinOnce();
    delay(20);
  }
}
