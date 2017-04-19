

#define buttonPin 2

#define roomba Serial1
#define euclid Serial2

// Roomba serial commands
#define R_RESET     0x07
#define R_START     0x80
#define R_STOP      0xAD
#define R_MODE_FULL 0x84
#define R_MOTOR     0x8A

/*************************************************************
SETUP
*************************************************************/
void setup() { // will be triggered when robot is powered on
  delay(2000); // Needed to let the robot initialize
  pinMode(buttonPin, INPUT_PULLUP);

  // start HW1 serial
  roomba.begin(115200);
  // start HW2 serial
  //euclid.begin(0);
  // start USB serial port
  Serial.begin(115200);
  
  delay(500);
  roomba.write(R_RESET); // reset Roomba, to clear all current states
}
/*************************************************************
LOOP
*************************************************************/
void loop() {
  if (roomba.available() > 0) {
    Serial.write(roomba.read());
  }
  if (Serial.available() > 0) {
    roomba.write(Serial.read());
  }
  
  if (digitalRead(buttonPin) == LOW) {
    roomba.write(R_START); // This command starts the OI.
    delay(100);
    roomba.write(R_MODE_FULL); // set mode to full (see p.7 of OI manual)
    delay(100);
    roomba.write(R_MOTOR); // set motor power
    delay(100);
    roomba.write(0b00000100); // ^ brush motor full (sends power to on-board computer)
    delay(100);
  }
}
