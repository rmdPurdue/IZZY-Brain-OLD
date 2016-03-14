#include "IZZY_Brain_I2C_02012016.h"
#include <SoftwareSerial.h>
#include <Wire.h>

//Test.

SoftwareSerial xbee(2,3);

float resolution = 93.2;  // counts per inch

long I2CSendLastTime;
long I2CSendTime;
long moveElapsedTime;
long lastTime;
int sampleTime = 150;
int DATABUFFERSIZE = 80;
int serialBuffer[81];
char startChar, endChar, delimiterChar;

PROFILE profileAStandby = {0, 0, 0, 0, 0, 0}; // d, accel, decel, vmax, setVel, moveElapsedTime
PROFILE profileA = {0, 0, 0, 0, 0, 0};
PROFILE profileBStandby = {0, 0, 0, 0, 0, 0};
PROFILE profileB = {0, 0, 0, 0, 0, 0};
//gGOBOOLEAN goBoolean = {false, false, false, false};
MOTOR motorA = {0, 0, 0};
MOTOR motorB = {0, 0, 0};
TIME timeA = {0, 0, false};
TIME timeB = {0, 0, false};
CUE thisCue = {0,0,0,0,0,0,0,false,false,0,0,false};  //standbyT, standbyAT, standbyDT, t, at, dt, cueUp, go, start, elapsed, started

COMMS outgoingI2C = {false, false, false, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // cmdRecieved, transmissionLost, Estop, type, command, motor, parameter, data1, data 2, lastTime
COMMS incomingI2C = {false, false, false, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup() {
  
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  
  Wire.begin(1);
  Serial.begin(9600);
  xbee.begin(9600);
  
//  goBoolean.goA = false;
//  goBoolean.goB = false;
  thisCue.go = false;
  
  startChar = '!';
  endChar = 255;
  delimiterChar = ',';
}

void loop() {
  parseSerial();
  receiveI2C();
  calculateProfile(&profileAStandby, &thisCue);  // find a way to only do this once, not every cycle
  calculateProfile(&profileBStandby, &thisCue);
  
  /**********************************************************************
  Calculate Velocity, A and B
  **********************************************************************/
  
  if(thisCue.cueUp) {
    profileA = profileAStandby;
    profileB = profileBStandby;
    profileAStandby = {0, 0, 0, 0, 0, 0};
    profileBStandby = {0, 0, 0, 0, 0, 0};
    thisCue.t = thisCue.standbyT * 1000;
    thisCue.at = thisCue.standbyAT * 1000;
    thisCue.dt = thisCue.standbyDT * 1000;
    profileA.vmax /= 1000;
    profileB.vmax /= 1000;
    profileA.accel /= 1000000;
    profileB.accel /= 1000000;
    profileA.decel /= 1000000;
    profileB.decel /= 1000000;
    thisCue.standbyT = 0;
    thisCue.standbyAT = 0;
    thisCue.standbyDT = 0;
    thisCue.cueUp = false;
    thisCue.go = true;
//    goBoolean.cueUpA = false;
//    goBoolean.cueUpB = false;
//    goBoolean.goA = true;
//    goBoolean.goB = true;
  Serial.print("Vmax: ");
  Serial.print(profileA.vmax);
  Serial.print(" Accel: ");
  Serial.print(profileA.accel);
  Serial.print(" Decel: ");
  Serial.println(profileA.decel);
  }

  if(thisCue.go && !thisCue.started) {
    thisCue.start = millis();
    thisCue.started = true;
  }
  if(thisCue.go && thisCue.started) {
    long now = millis();
    thisCue.elapsed = now - thisCue.start;
    long constTime = thisCue.t - thisCue.at - thisCue.dt;
    float accelPositionA = 0.5 * profileA.accel * pow(thisCue.at,2);
    float accelPositionB = 0.5 * profileB.accel * pow(thisCue.at,2);
    float constPositionA = accelPositionA + (profileA.vmax * constTime);
    float constPositionB = accelPositionB + (profileB.vmax * constTime);
    long timeBeforeDecel = thisCue.t - thisCue.dt;
    if(thisCue.elapsed <= thisCue.t) {
      if(thisCue.elapsed < thisCue.at) {
        profileA.setVel = thisCue.elapsed * profileA.accel;
        profileB.setVel = thisCue.elapsed * profileB.accel;
        profileA.deltaPos = 0.5 * profileA.accel * pow(thisCue.elapsed,2);
        profileB.deltaPos = 0.5 * profileB.accel * pow(thisCue.elapsed,2);
      }
      if(thisCue.elapsed >= thisCue.at && thisCue.elapsed < thisCue.t - thisCue.dt) {
        profileA.setVel = profileA.vmax;
        profileB.setVel = profileB.vmax;
        profileA.deltaPos = accelPositionA + (profileA.vmax * (thisCue.elapsed - thisCue.at));
        profileB.deltaPos = accelPositionB + (profileB.vmax * (thisCue.elapsed - thisCue.at));
      }
      if(thisCue.elapsed >= thisCue.t - thisCue.dt && thisCue.elapsed <= thisCue.t) {
        profileA.setVel = (thisCue.t - thisCue.elapsed) * profileA.decel;
        profileB.setVel = (thisCue.t - thisCue.elapsed) * profileB.decel;
        profileA.deltaPos = constPositionA + (profileA.vmax * (thisCue.elapsed - timeBeforeDecel)) - (0.5 * profileA.decel * pow(thisCue.elapsed - timeBeforeDecel,2));
        profileB.deltaPos = constPositionB + (profileB.vmax * (thisCue.elapsed - timeBeforeDecel)) - (0.5 * profileB.decel * pow(thisCue.elapsed - timeBeforeDecel,2));
      }
    } else {
      profileA.setVel = 0;
      profileB.setVel = 0;
      profileA.deltaPos = 0;
      profileB.deltaPos = 0;
      thisCue.start = 0;
      thisCue.elapsed = 0;
      thisCue.started = false;
      thisCue.go = false;
    }
    Serial.print(" Elapsed: ");
    Serial.print(thisCue.elapsed);
    Serial.print(" Total Time: ");
    Serial.print(thisCue.t * 1000);
    Serial.print(" Delta Position A: ");
    Serial.println(profileA.deltaPos);
    outgoingI2C.type = 1;
    outgoingI2C.command = 0;
    outgoingI2C.motor = 0;
    outgoingI2C.parameter = thisCue.dir;
    outgoingI2C.data1 = int(profileA.deltaPos) >> 8;
    outgoingI2C.data2 = int(profileA.deltaPos) & B11111111;
    sendI2C();
//    delay(100);
//    outgoingI2C.type = 1;
//    outgoingI2C.command = 0;
//    outgoingI2C.motor = 1;
//    outgoingI2C.parameter = 0;
//    outgoingI2C.data1 = int(profileB.setVel) >> 8;
//    outgoingI2C.data2 = int(profileB.setVel) & B11111111;
//    sendI2C();
  }
  
  sendSerial();
  delay(sampleTime);
}

void calculateProfile(PROFILE *profile, CUE *cue) {
  if(cue->standbyT > 0 && cue->standbyAT > 0 && cue->standbyDT > 0) {
    float constTime = cue->standbyT - cue->standbyAT - cue->standbyDT;
    profile->vmax = (profile->d * 93.2 / ((cue->standbyAT/2) + (cue->standbyDT/2) + constTime));
    profile->accel = (profile->vmax / cue->standbyAT);
    profile->decel = (profile->vmax / cue->standbyDT);
  } else {
    profile->vmax = 0;
    profile->accel = 0;
    profile->decel = 0;
  }
}

void sendSerial() {
  byte outgoingBytes[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
  outgoingBytes[0] = incomingI2C.transmissionLost << 7;
  outgoingBytes[0] |= (incomingI2C.Estop << 6);
  outgoingBytes[0] |= (motorA.overCurrent << 5);
  outgoingBytes[0] |= (motorB.overCurrent << 4);
  outgoingBytes[0] |= (motorA.fault << 2);
  outgoingBytes[0] |= (motorB.fault << 0);
  outgoingBytes[1] = (int)motorA.vel >> 8;
  outgoingBytes[2] = (int)motorA.vel;
  outgoingBytes[3] = (int)motorB.vel >> 8;
  outgoingBytes[4] = (int)motorB.vel;
  outgoingBytes[5] = profileAStandby.d;
  outgoingBytes[6] = thisCue.standbyT;
  outgoingBytes[7] = thisCue.standbyAT;
  outgoingBytes[8] = thisCue.standbyDT;
  outgoingBytes[9] = thisCue.dir;
//  outgoingBytes[6] = profileAStandby.t;
//  outgoingBytes[7] = profileAStandby.at;
//  outgoingBytes[8] = profileAStandby.dt;
//  outgoingBytes[9] = profileBStandby.d;
//  outgoingBytes[10] = profileBStandby.t;
//  outgoingBytes[11] = profileBStandby.at;
//  outgoingBytes[12] = profileBStandby.dt;
  xbee.print("!");
  for(int i = 0; i < 13; i++) {
    xbee.write(outgoingBytes[i]);
    xbee.print(",");
  }
  xbee.write(255);
}

void parseSerial() {
  boolean gotData = false;
  gotData = getSerialData();
  if(gotData) {
    if(serialBuffer[0] == 2) {
      thisCue.cueUp = true;
    } else if(serialBuffer[0] == 1) {
      profileAStandby.d = serialBuffer[1];
      profileBStandby.d = serialBuffer[1];
      thisCue.standbyT = serialBuffer[2];
      thisCue.standbyAT = serialBuffer[3];
      thisCue.standbyDT = serialBuffer[4];
      thisCue.dir = serialBuffer[5];
    }
  }
/*  byte val[9] = {0,0,0,0,0,0,0,0,0};
  if(xbee.available() > 0) {
    for(int i=0; i<9; i++) {
      val[i] = xbee.read();
    }
  }
  if(val[0] == 2) {
    thisCue.cueUp = true;
    goBoolean.cueUpA = true;
    goBoolean.cueUpB = true;
  } else if(val[0] == 1) {
    profileAStandby.d = val[1];
    thisCue.t = val[2];
    thisCue.at = val[3];
    thisCue.dt = val[4];
    profileAStandby.t = val[2];
    profileAStandby.at = val[3];
    profileAStandby.dt = val[4];
    profileBStandby.d = val[5];
    profileBStandby.t = val[6];
    profileBStandby.at = val[7];
    profileBStandby.dt = val[8];
  }
  */
}

void receiveI2C() {
  int numBytes =  Wire.requestFrom(2,5);
  byte incomingBytes[5] = {0,0,0,0,0};
  int test;
  int velAinSec;
  int velBinSec;
  while(Wire.available()) {
    for(int a = 0; a < 5; a++) {
      incomingBytes[a] = Wire.read();
    }
  }
  incomingI2C.transmissionLost = incomingBytes[0] >> 7 & B00000001;
  incomingI2C.Estop = incomingBytes[0] >> 6 & B00000001;
  motorA.overCurrent = incomingBytes[0] >> 5 & B00000001;
  motorB.overCurrent = incomingBytes[0] >> 4 & B00000001;
  motorA.fault = incomingBytes[0] >> 3 & B00000001;
  motorB.fault = incomingBytes[0] >> 1 & B00000001;
  
  motorA.vel = incomingBytes[1] << 8 | incomingBytes[2];
  motorB.vel = incomingBytes[3] << 8 | incomingBytes[4];
}

void sendI2C() {
  byte bytes[] = {0, 0, 0};
  bytes[0] = outgoingI2C.type << 5;
  bytes[0] |= outgoingI2C.command << 3;
  bytes[0] |= outgoingI2C.motor << 2;
  bytes[0] |= outgoingI2C.parameter;
  bytes[1] = outgoingI2C.data1;
  bytes[2] = outgoingI2C.data2;
  Wire.beginTransmission(2);
  Wire.write(bytes,3);
  Wire.endTransmission();
}

boolean getSerialData() {
  int serialBufferIndex = 0;
  boolean storeData = false;
  while(xbee.available() > 0) {
    int incomingByte = xbee.read();
    if(incomingByte == startChar) {
      serialBufferIndex = 0;
      storeData = true;
    }
    if(storeData) {
      if(serialBufferIndex == DATABUFFERSIZE) {
        serialBufferIndex = 0;
        break;
      }
      if(incomingByte == 255) {
        serialBuffer[serialBufferIndex] = 0;
        return true;
      } else {
        if(incomingByte != '!' && incomingByte != delimiterChar) {
          serialBuffer[serialBufferIndex++] = incomingByte;
          serialBuffer[serialBufferIndex] = 0;
        }
      }
    }
  }
  return false;
}
        
