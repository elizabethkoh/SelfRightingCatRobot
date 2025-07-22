#include <Dynamixel2Arduino.h>
#include <M5Atom.h>
#include <HardwareSerial.h>

#define DEBUG_SERIAL Serial
HardwareSerial& DXL_SERIAL = Serial1;
Dynamixel2Arduino dxl;

//Atom Matrix Ports
const uint8_t RX_SERVO = 26;
const uint8_t TX_SERVO = 32;

const uint8_t DXL_ID = 1; //DYNAMIXEL ID is specific to each servo, must be unique, change in DYNAMIXEL Wizard
const float DXL_PROTOCOL_VER = 2.0;
volatile bool torque_on = false;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

using namespace ControlTableItem;

//variables
bool ping_check = 0;
bool scan_check = 0;
bool initial_torque = 0;
bool torque_check = 0;

void setup() {
  // put your setup code here, to run once:

  //M5 Atom Matrix Setup
  M5.begin(true, true, true);
  M5.IMU.Init();
  M5.dis.fillpix(0xff0000);
  M5.dis.setBrightness(10);

  //DYNAMIXEL servo Setup
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);  //set baudrate
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER);
  ping_check = dxl.ping(DXL_ID);  //returns 1 if successful, for some reason only returns 1 when checked in the main loop
  scan_check = dxl.scan();

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.setGoalVelocity(DXL_ID, 0); 
  torque_check = dxl.torqueOn(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:

  //toggle torque_on if Atom Matrix button is pressed
  if (M5.Btn.wasPressed())
  {
    DEBUG_SERIAL.printf("Torque Check: %d\n", torque_check);
    
    ping_check = dxl.ping(DXL_ID);  //returns 1 if successful
    scan_check = dxl.scan();
    DEBUG_SERIAL.printf("Ping: %d, Scan: %d\n", ping_check, scan_check);
    torque_on = !torque_on;
    //toggle the torque accordingly
    if (torque_on == 0)
    {
      dxl.setGoalVelocity(DXL_ID, 0);
      M5.dis.fillpix(0xff0000);
      dxl.ledOn(DXL_ID);
    }
    else
    {
      dxl.setGoalVelocity(DXL_ID, 50, UNIT_RPM); //FOR WHATEVER REASON UNIT_PERCENT DOESNT WORK 
      M5.dis.fillpix(0x00ff00);
      dxl.ledOff(DXL_ID);
    }
  }
  
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  DEBUG_SERIAL.printf("Gyro | X: %f, Y: %f, Z: %f\n", gyroX, gyroY, gyroZ);

  M5.IMU.getAccelData(&accelX, &accelY, &accelZ);
  DEBUG_SERIAL.printf("Accel | X: %f, Y: %f, Z: %f\n", accelX, accelY, accelZ);

  delay(20);
  M5.update();
}
