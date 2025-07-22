#include <Dynamixel2Arduino.h>
#include <M5Atom.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <iostream>
#include <bitset>
#include <fstream>
#include <string>
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

const uint8_t RX_SERVO = 26;
const uint8_t TX_SERVO = 32;

const uint8_t HEAD_DXL_ID = 1;
const uint8_t FTORSO_DXL_ID = 2;
const uint8_t RTORSO_DXL_ID = 3;
const float DXL_PROTOCOL_VER = 2.0;
volatile bool torque_on = false;

volatile int lastInd = 0;
int accelX, accelY, accelZ;
int gyroX, gyroY, gyroZ;
int accelXAry[AVERAGING_NUM] = {}, accelYAry[AVERAGING_NUM] = {}, accelZAry[AVERAGING_NUM] = {};
int gyroXAry[AVERAGING_NUM] = {}, gyroYAry[AVERAGING_NUM] = {}, gyroZAry[AVERAGING_NUM] = {};
float accelXCurr, accelYCurr, accelZCurr;
float gyroXCurr, gyroYCurr, gyroZCurr;
volatile int32_t accelXAvg, accelYAvg, accelZAvg;
volatile int32_t gyroXAvg, gyroYAvg, gyroZAvg;
volatile int32_t accelXTot, accelYTot, accelZTot;
volatile int32_t gyroXTot, gyroYTot, gyroZTot;
int32_t accelAry[3];
std::string printout = "";
char* printoutChar;

bool testFlag = 0;
int counter = 0;

ofstream dataCollection;

//function to transform
uint8_t* transformInt(int16_t inputAry[]) {
  int inputSize = 3;  //sizeof(inputAry);// / sizeof(inputAry[0]);
  int bits[inputSize * 16];
  uint8_t* output = new uint8_t[inputSize * 2];
  //DEBUG_SERIAL.printf("inputSize is %d\n", inputSize);

  for (int i = 0; i < inputSize; i++) {
    //DEBUG_SERIAL.printf("i is %d\n", i);
    int num1 = inputAry[i];
    if (num1 < 0) {
      num1 = num1 + 65536;
    }
    for (int j = 15; j >= 0; j--) {
      //DEBUG_SERIAL.printf("num1 is %d\n", num1);
      if (num1 != 0) {
        bits[i * 16 + j] = num1 % 2;
        num1 = num1 >> 1;
      } else {
        bits[i * 16 + j] = 0;
      }
    }
    //DEBUG_SERIAL.prinf("Num1.%d: %d", i, num1);
  }

  for (int i = 0; i < 48; i++) {
    DEBUG_SERIAL.printf("%d", bits[i]);
  }
  DEBUG_SERIAL.printf("\n");


  for (int i = 0; i < inputSize * 2; i++) {
    DEBUG_SERIAL.printf("i is %d\n", i);
    uint8_t num2 = 0;
    for (int j = 0; j < 8; j++) {
      int singleBit = bits[i * 8 + j];
      num2 = num2 + (singleBit * pow(2, 7 - j));
      DEBUG_SERIAL.printf("bit is %d, num2 is %d\n", singleBit, num2);
    }
    output[i] = num2;
  }
  return output;
}

void setup() {
  // put your setup code here, to run once:
  //M5 Atom Matrix Setup
  M5.begin(true, true, true);
  M5.IMU.Init();
  M5.dis.fillpix(0xff0000);

  M5.dis.setBrightness(10);

  //DYNAMIXEL servo Setup
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  SERIAL_BT.begin(device_name);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);  //set baudrate
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER);
  dxl.ping(HEAD_DXL_ID);  //returns 1 if successful, for some reason only returns 1 when checked in the main loop
  dxl.scan();

  dxl.torqueOff(HEAD_DXL_ID);
  dxl.setOperatingMode(HEAD_DXL_ID, OP_VELOCITY);
  dxl.setGoalVelocity(HEAD_DXL_ID, 0);
  dxl.torqueOn(HEAD_DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    SERIAL_BT.write(Serial.read());
  }
  if (SERIAL_BT.available()) {
    Serial.write(SERIAL_BT.read());
  }

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

  //uint8_t* accelAryMod = transformInt(accelAry);

  uint8_t* accelAryPtr = (uint8_t*)accelAry;
  //unsigned char my_buffer[20] = "Hello World";
  //uint8_t* u8_pointer = my_buffer;
  size_t me = sizeof(accelAry);
  SERIAL_BT.write(accelAryPtr, me);


  if (counter < 500) {
    DEBUG_SERIAL.printf("%d\n", counter);
  }
  counter++;
  delay(20);
}
