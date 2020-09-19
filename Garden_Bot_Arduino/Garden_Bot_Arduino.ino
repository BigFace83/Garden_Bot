#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

#define enA 5
#define enB 6
#define in1R 7
#define in2R 4
#define in1L 9
#define in2L 8

#define SonarPin 10
#define STOP 0
#define DRIVEMODE 1
#define TURNMODE 2


int Mode = STOP;


#define BUFFERSIZE 64
char databuffer[BUFFERSIZE];
int serialcounter = 0;

unsigned long previousMillis = 0;

volatile int Speed = 0; //Speed reference for left and right motors
volatile int Command = 0; 
volatile int Data = 0;



MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int Rollangle;
int Pitchangle;
int Yawangle;

//Container for serial data
char data_packet[50];

//Servo variables
Servo Servo1;
Servo Servo2;

//Global variables for servo positions
int Servo1Pos = 90;
int Servo2Pos = 90;

#define Serv1Pin A7
#define Serv2Pin A6


//Define Variables we'll be connecting to
double DriveSetpoint, DriveInput, DriveOutput;
double TurnSetpoint, TurnInput, TurnOutput;

//Specify the links and initial tuning parameters
PID DrivePID(&DriveInput, &DriveOutput, &DriveSetpoint, 0.1, 0.0, 0.5, DIRECT);
PID TurnPID(&TurnInput, &TurnOutput, &TurnSetpoint, 4.0, 0.0, 1.5, DIRECT);
float RightWheel = 0.0;
float LeftWheel = 0.0;



void setup() {
  Serial.begin(115200);
  //TCCR1B &= ~7; //PWM frequency for motors
  //TCCR1B |= 2;
  pinMode(enA, OUTPUT); //Motor control pins to outputs
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);

  // Set initial motor states to braking
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, HIGH);
  digitalWrite(enA, HIGH);

  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, HIGH);
  digitalWrite(enB, HIGH);


  //PID for driving in a straight line using IMU Yaw data
  DriveSetpoint = 0;
  DrivePID.SetSampleTime(100);
  DrivePID.SetOutputLimits(-10, 10);
  DrivePID.SetMode(AUTOMATIC);

  //PID for turning on the spot using IMU Yaw data
  TurnSetpoint = 0;
  TurnPID.SetSampleTime(100);
  TurnPID.SetOutputLimits(-255, 255);
  TurnPID.SetMode(AUTOMATIC);


  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-135);
  mpu.setYGyroOffset(77);
  mpu.setZGyroOffset(-147);
  mpu.setZAccelOffset(1466); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }



  //initialize servos
  Servo1.attach(12);
  Servo2.attach(13);

  Servo1.write(Servo1Pos);
  Servo2.write(Servo2Pos);


  //check if IMU yaw data has settled. Move on only when steady readings are present
  //Needs 20 identical readings before moving on
  int counter = 0;
  GetMPUdata();
  int YawPrev = Yawangle;
  Serial.print("Allow IMU to settle...");

  while (1) {
    delay(50);
    GetMPUdata();
    int yawdiff = abs(Yawangle - YawPrev);
    if (yawdiff == 0) {
      counter++;
    }
    else {
      counter = 0;
    }
    if (counter > 20) {
      break;
    }
    YawPrev = Yawangle;

  }

  //TurnSetpoint = Yawangle;
  //DriveSetpoint = Yawangle;
  Serial.println("Setup complete");
}

void loop() {

  GetMPUdata();
  int Sonar = ReadSonar(SonarPin);

  //keep an eye out for serial data
  while (Serial.available() > 0) // if something is available from serial port
  {
    char c = Serial.read();    // get it
    databuffer[serialcounter] = c; //add it to the data buffer
    serialcounter++;
    if (c == '\n') { //newline character denotes end of message
      serialcounter = 0; //reset serialcounter ready for the next message
      sscanf (databuffer, "%d,%d", &Command, &Data);
      memset(databuffer, 0, sizeof(databuffer));//clear data buffer when command has been received

      //Serial commands
      // 0,data - Stops robot wheels. Ignores data but needs it to be sent
      // 1,data - Robot forward or reverse, data is speed reference. 0-255 forward, 0--255 reverse
      // 2,data - Turn on the spot. Data is target angle
      // 3,data - Servo 1 position, data is angle
      // 4,data - Servo 1 position, data is angle
      switch (Command) {
      case 0:
        LeftWheel = 0;
        RightWheel = 0;
        SetWheelSpeeds(int(LeftWheel), int(RightWheel));
        Mode = STOP;
        break;
      case 1:
        Speed = Data;
        DriveSetpoint = Yawangle;
        Mode = DRIVEMODE;
        LeftWheel = constrain(Speed, -255.0 , 255.0);
        RightWheel = constrain(Speed, -255.0 , 255.0);
        SetWheelSpeeds(int(LeftWheel), int(RightWheel));
        break;
      case 2:
        Data = constrain(Data, -179 , 179);
        TurnSetpoint = Data;
        LeftWheel = 0;
        RightWheel = 0;
        SetWheelSpeeds(int(LeftWheel), int(RightWheel));
        Mode = TURNMODE;
        break;
      case 3:
        Servo1Pos = Data;
        break;
      case 4:
        Servo2Pos = Data;
        break;}
      break;}
  }

  

  Servo1.write(Servo1Pos);
  Servo2.write(Servo2Pos);


  if (Mode == TURNMODE) {

    TurnInput = ResolveWrap(Yawangle, TurnSetpoint);
    if (TurnPID.Compute()) {
      LeftWheel = LeftWheel + TurnOutput;
      LeftWheel = constrain(LeftWheel, -255.0 , 255.0);
      RightWheel = RightWheel - TurnOutput;
      RightWheel = constrain(RightWheel, -255.0 , 255.0);

    }

    SetWheelSpeeds(int(LeftWheel), int(RightWheel));
  }
  else if (Mode == DRIVEMODE) {
    DriveInput = ResolveWrap(Yawangle, DriveSetpoint);
    if (DrivePID.Compute()) {
      LeftWheel = LeftWheel + DriveOutput;
      LeftWheel = constrain(LeftWheel, -255.0 , 255.0);
      RightWheel = RightWheel - DriveOutput;
      RightWheel = constrain(RightWheel, -255.0 , 255.0);
    }
    if (Sonar < 200 || Speed == 0){
      SetWheelSpeeds(0, 0);
    }
    else{
      SetWheelSpeeds(int(LeftWheel), int(RightWheel));
    }
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 200) //use this to determine frequency of sending data
  {
    previousMillis = currentMillis;
    sprintf(data_packet, "%d,%d,%d,%d", Rollangle, Pitchangle, Yawangle, Sonar);
    Serial.println(data_packet);
  }

}


void SetWheelSpeeds(int left, int right) {

  analogWrite(enB, abs(left));
  analogWrite(enA, abs(right));

  if (left > 0) {
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
  }
  else if (left < 0) {
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
  }
  else { //Brake
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, HIGH);
    digitalWrite(enB, HIGH);
  }

  if (right > 0) {
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
  }
  else if (right < 0) {
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
  }
  else { //Brake
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, HIGH);
    digitalWrite(enA, HIGH);
  }
}



void GetMPUdata() {

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else { // if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    Yawangle = (ypr[0] * 180 / M_PI);
    Pitchangle = (ypr[1] * 180 / M_PI);
    Rollangle = (ypr[2] * 180 / M_PI);
  }
}

int ReadSonar(int Pin) //Read sonar sensor
{
  
  //read head sonar sensor 
  pinMode(Pin, OUTPUT);
  digitalWrite(Pin, LOW);             // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(Pin, HIGH);            // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(Pin, LOW);             // Send pin low again before waiting for pulse back in
  pinMode(Pin, INPUT);
  int duration = pulseIn(Pin, HIGH);  // Reads echo pulse in from SRF05 in micro seconds
  int sonardist = duration/5.8;      // Dividing this by 58 gives us a distance in mm
  
  return sonardist;
  
}

int ResolveWrap(int Yawangle, int Setpoint)
{
//Some maths to deal with wrap around at 179,-179 to feed sensible values to PID controller
//Calculates consistent error value when operating near wrap around point
    int angleDiff = Yawangle - Setpoint;
    int newYaw;
    if (angleDiff < -179) {
      newYaw = Setpoint + (359 + angleDiff);
    }
    else if (angleDiff > 179) {
      newYaw = Setpoint - (359 - angleDiff);
    }
    else {
      newYaw = Yawangle;
    }

    return newYaw;
}
