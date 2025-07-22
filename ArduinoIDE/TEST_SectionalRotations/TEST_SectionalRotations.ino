#include <Dynamixel2Arduino.h>
#include <M5Atom.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <fstream>
#include <chrono>
#include <cmath>
using namespace std;
using namespace ControlTableItem;

#define DEBUG_SERIAL Serial
HardwareSerial& DXL_SERIAL = Serial1;
Dynamixel2Arduino dxl;
BluetoothSerial SERIAL_BT;
String device_name = "ESP32-BT-Slave";

#define SCALE_FACTOR 1000
#define AVERAGING_NUM 8
#define AVERAGING_BITSHIFT 3
#define DEG_TO_TICKS 11.375 //exact value, 4095/360

#define EKF_N 6
#define EKF_M 6
#include <tinyekf.h>
static const float eps = 1e-4;
static const float Q[EKF_N*EKF_N] = 
{
  eps, 0, 0, 0, 0, 0,
  0, eps, 0, 0, 0, 0,
  0, 0, eps, 0, 0, 0,
  0, 0, 0, eps, 0, 0,
  0, 0, 0, 0, eps, 0,
  0, 0, 0, 0, 0, eps
};
static const float R[EKF_M*EKF_M] = 
{
  eps, 0, 0, 0, 0, 0,
  0, eps, 0, 0, 0, 0,
  0, 0, eps, 0, 0, 0,
  0, 0, 0, eps, 0, 0,
  0, 0, 0, 0, eps, 0,
  0, 0, 0, 0, 0, eps
};
static const float F[EKF_N*EKF_N] = 
{
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0,
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1,
};
static const float H[EKF_M*EKF_N] = 
{
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0,
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1,
};
static ekf_t _ekf;

const uint8_t RX_SERVO = 26;
const uint8_t TX_SERVO = 32;

const uint8_t HEAD_DXL_ID = 1;
const uint8_t FTORSO_DXL_ID = 2;
const uint8_t RTORSO_DXL_ID = 3;
const float DXL_PROTOCOL_VER = 2.0;
volatile bool zeroed = 0;
bool ping_check[3];
bool scan_check;
bool fallArmed = 0;

volatile int lastInd = 0;
int accelX, accelY, accelZ;
int gyroX, gyroY, gyroZ;
int accelXAry[AVERAGING_NUM] = {}, accelYAry[AVERAGING_NUM] = {}, accelZAry[AVERAGING_NUM] = {};
int gyroXAry[AVERAGING_NUM] = {}, gyroYAry[AVERAGING_NUM] = {}, gyroZAry[AVERAGING_NUM] = {};
float accelXOff, accelYOff, accelZOff;
float gyroXOff, gyroYOff, gyroZOff;
float accelXCurr, accelYCurr, accelZCurr;
float gyroXCurr, gyroYCurr, gyroZCurr;
volatile int accelXAvg, accelYAvg, accelZAvg;
volatile int gyroXAvg, gyroYAvg, gyroZAvg;
volatile int accelXTot, accelYTot, accelZTot;
volatile int gyroXTot, gyroYTot, gyroZTot;
int32_t accelAry[3];
double accelR;
volatile int headPos, FTPos, RTPos;
bool overTurn[3]= {0, 0, 0};

volatile int counter = 0;
volatile int fallCounter = 0;

ofstream dataCollection;

void resetPos(int id, int speed)
{
  int pos = dxl.getPresentPosition(id);
  if (pos <= 2048)
  {
    dxl.setGoalPosition(id, 0);
    dxl.setGoalVelocity(id, speed, UNIT_PERCENT);
  }
  else
  {
    dxl.setGoalPosition(id, 4095, UNIT_RAW);
    dxl.setGoalVelocity(id, speed, UNIT_PERCENT);
  }
  int newPos = dxl.getPresentPosition(id);
  if (abs(newPos-pos) > 2048)
  {
    overTurn[id-1] = !overTurn[id-1];
  }
  DEBUG_SERIAL.printf("Reset: %d, %d\n", id, overTurn[id-1]);
}
void turnToPos(int id, int target, int speed)
{
  int currPos = dxl.getPresentPosition(id);
  if (abs(target) > 180){overTurn[id-1] = !overTurn[id-1];}
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, speed);
  dxl.setGoalPosition(id, currPos + target*DEG_TO_TICKS, UNIT_RAW);
  DEBUG_SERIAL.printf("Turn: %d, %d | %d\n", id, overTurn[id-1], currPos);
}

void fallSeq()
{
  turnToPos(FTORSO_DXL_ID, 300, 5000);
  turnToPos(RTORSO_DXL_ID, -300, 5000);
  delay(250);
  turnToPos(FTORSO_DXL_ID, -300, 5000);
  turnToPos(RTORSO_DXL_ID, 300, 5000);
  resetPos(FTORSO_DXL_ID, 5000);
  resetPos(RTORSO_DXL_ID, 5000);

  fallArmed = 0;
  M5.dis.fillpix(0xff0000);
}

void updateIMU()
{
  M5.IMU.getAccelData(&accelXCurr, &accelYCurr, &accelZCurr);
  M5.IMU.getGyroData(&gyroXCurr, &gyroYCurr, &gyroZCurr);

  accelX = int(accelXCurr * SCALE_FACTOR);
  accelXTot = accelXTot - accelXAry[lastInd] + accelX;
  accelXAry[lastInd] = accelX;
  accelXAvg = accelXTot >> AVERAGING_BITSHIFT;

  accelY = int(accelYCurr * SCALE_FACTOR);
  accelYTot = accelYTot - accelYAry[lastInd] + accelY;
  accelYAry[lastInd] = accelY;
  accelYAvg = accelYTot >> AVERAGING_BITSHIFT;

  accelZ = int(accelZCurr * SCALE_FACTOR);
  accelZTot = accelZTot - accelZAry[lastInd] + accelZ;
  accelZAry[lastInd] = accelZ;
  accelZAvg = accelZTot >> AVERAGING_BITSHIFT;

  gyroX = int(gyroXCurr * SCALE_FACTOR);
  gyroXTot = gyroXTot - gyroXAry[lastInd] + gyroX;
  gyroXAry[lastInd] = gyroX;
  gyroXAvg = gyroXTot >> AVERAGING_BITSHIFT;

  gyroY = int(gyroYCurr * SCALE_FACTOR);
  gyroYTot = gyroYTot - gyroYAry[lastInd] + gyroY;
  gyroYAry[lastInd] = gyroY;
  gyroYAvg = gyroYTot >> AVERAGING_BITSHIFT;

  gyroZ = int(gyroZCurr * SCALE_FACTOR);
  gyroZTot = gyroZTot - gyroZAry[lastInd] + gyroZ;
  gyroZAry[lastInd] = gyroZ;
  gyroZAvg = gyroZTot >> AVERAGING_BITSHIFT;

  lastInd++;
  lastInd = lastInd % AVERAGING_NUM;

  accelAry[0] = accelXAvg;
  accelAry[1] = accelYAvg;
  accelAry[2] = accelZAvg;

  uint8_t* accelAryPtr = (uint8_t*)accelAry;
  size_t me = sizeof(accelAry);
  SERIAL_BT.write(accelAryPtr, me);
}

void Kalman()
{
  M5.IMU.getAccelData(&accelXCurr, &accelYCurr, &accelZCurr);
  M5.IMU.getGyroData(&gyroXCurr, &gyroYCurr, &gyroZCurr);

  accelX = accelXCurr * SCALE_FACTOR; accelY = accelYCurr * SCALE_FACTOR; accelZ = accelZCurr * SCALE_FACTOR;
  gyroX = gyroXCurr * SCALE_FACTOR; gyroY = gyroYCurr * SCALE_FACTOR; gyroZ = gyroZCurr * SCALE_FACTOR;

  const float z[EKF_M] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};

  const float fx[EKF_N] = {_ekf.x[0], _ekf.x[1], _ekf.x[2], _ekf.x[3], _ekf.x[4], _ekf.x[5]};

  ekf_predict(&_ekf, fx, F, Q);

  const float hx[EKF_M] = {_ekf.x[0], _ekf.x[1], _ekf.x[2], _ekf.x[3], _ekf.x[4], _ekf.x[5]};

  ekf_update(&_ekf, z, hx, H, R);

  accelAry[0] = _ekf.x[0];
  accelAry[1] = _ekf.x[1];
  accelAry[2] = _ekf.x[2];

  uint8_t* accelAryPtr = (uint8_t*)accelAry;
  size_t me = sizeof(accelAry);
  SERIAL_BT.write(accelAryPtr, me);  
}

void printTime() 
{
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() % 1000;
    auto t = std::time(0);
    auto now = std::localtime(&t);
    char buffer[sizeof "9999-12-31 29:59:59.9999"];
    sprintf(
        buffer,
        "%04d-%02d-%02d %02d:%02d:%02d.%lld",
        now->tm_year + 1900,
        now->tm_mon + 1,
        now->tm_mday,
        now->tm_hour,
        now->tm_min,
        now->tm_sec,
        millis);

    std::cout << buffer << std::endl;
}

void setup() 
{
  //M5 Atom Matrix Setup
  M5.begin(true, true, true);
  M5.IMU.Init();
  M5.dis.fillpix(0xff0000);
  M5.dis.setBrightness(10);

  //tinyEKF Setup
  const float Pdiag[EKF_N] = {1,1};
  ekf_initialize(&_ekf, Pdiag);

  //DYNAMIXEL servo Setup
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  SERIAL_BT.begin(device_name);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);  //set baudrate
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER);

  dxl.torqueOff(HEAD_DXL_ID);
  dxl.setOperatingMode(HEAD_DXL_ID, OP_EXTENDED_POSITION);
  dxl.setGoalVelocity(HEAD_DXL_ID, 0);
  dxl.torqueOn(HEAD_DXL_ID);

  dxl.torqueOff(FTORSO_DXL_ID);
  dxl.setOperatingMode(FTORSO_DXL_ID, OP_EXTENDED_POSITION);
  dxl.setGoalVelocity(FTORSO_DXL_ID, 0);
  dxl.torqueOn(FTORSO_DXL_ID);

  dxl.torqueOff(RTORSO_DXL_ID);
  dxl.setOperatingMode(RTORSO_DXL_ID, OP_EXTENDED_POSITION);
  dxl.setGoalVelocity(RTORSO_DXL_ID, 0);
  dxl.torqueOn(RTORSO_DXL_ID);
}

void loop() //MAIN LOOP
{
  if (Serial.available()) {
    SERIAL_BT.write(Serial.read());
  }
  if (SERIAL_BT.available()) {
    Serial.write(SERIAL_BT.read());
  }

  if (!zeroed) //Initialize head and torso positions
  {
    resetPos(HEAD_DXL_ID, 5);
    resetPos(FTORSO_DXL_ID, 5);
    resetPos(RTORSO_DXL_ID, 5);
    zeroed = 1;
  }

  if (M5.Btn.wasPressed())
  {
    fallArmed = 1;
    M5.dis.fillpix(0x00ff00);
    //fallSeq();
  }

  updateIMU();
  //Kalman();

  accelR = sqrt(pow(accelAry[0],2) + pow(accelAry[1],2) + pow(accelAry[2],2));
  //DEBUG_SERIAL.printf("%d, %d, %d | %f\n\r", accelAry[0], accelAry[1], accelAry[2], accelR);
  if (fallArmed && static_cast <int>(accelR) < 100)
  {
    fallCounter++;
    if (fallCounter >= 3)
    {
      DEBUG_SERIAL.printf("Fall Detected\n\r");
      fallSeq();
    }  
  }

  delay(20);
  M5.update();
}