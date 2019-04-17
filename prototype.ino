#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include "Adafruit_LSM303.h"
#include "Adafruit_LSM303_U.h"
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

#define LIR 13
#define RIR 12

#define LM1 11
#define LM2 10

#define RM1 9
#define RM2 5

#define baseline 10
#define radius 2.5

char ssid[] = SECRET_SSID;       // your network SSID (name)
char pass[] = SECRET_PASS;       // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

float circumference = radius * TWO_PI;

float tickDistance = radius * deg2rad( 90 / 75.8 );

float thetaZ = 0, _thetaZ = tickDistance / baseline;

float x = 0, _x = ( baseline / 2 ) * sin ( _thetaZ ), _x0;
float y = 0, _y = ( baseline / 2 ) * ( 1 - cos ( _thetaZ ) ), _y0;

int cntR = 0; cntL = 0;

int KpSpeed = 4; KpDirection = 4;

Matrix<4,4> rotationR    = {  cos( _thetaZ ), -sin( _thetaZ ),  0,  0,
                              sin( _thetaZ ),  cos( _thetaZ ),  0,  baseline / 2,
                              0,  0,  z,  0,
                              0,  0,  0,  1};

Matrix<4,4> rotationR    = {  cos( -_thetaZ ), -sin( -_thetaZ ),  0,  0,
                              sin( -_thetaZ ),  cos( -_thetaZ ),  0,  -baseline / 2,
                              0,  0,  z,  0,
                              0,  0,  0,  1};
                                  
Matrix<4,4> translationR = {  1,  0,  0,  0,
                              0,  1,  0,  -baseline,
                              0,  0,  1,  0,
                              0,  0,  0,  1};

Matrix<4,4> translationL = {  1,  0,  0,  0,
                              0,  1,  0,  baseline,
                              0,  0,  1,  0,
                              0,  0,  0,  1};

Matrix<4,4> transformR = rotationR * translationR;
Matrix<4,4> transformL = rotationL * translationL;

Matrix<4,4> globalMatrix = {  1,  0,  0,  0,
                              0,  1,  0,  0,
                              0,  0,  1,  0,
                              0,  0,  0,  1};

void rightTickInterrupt() {
  cntR += 1;
  thetaZ += _thetaZ;

  globalMatrix = globalMatrix * transformR;
}

void lefttTickInterrupt() {
  cntL += 1;
  thetaZ -= _thetaZ;

  globalMatrix = globalMatrix * transformL;
}

void setupIR() {
  pinMode(LIR, INPUT_PULLUP);
  pinMode(RIR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIR), leftTickInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIR), rightTickInterrupt, RISING);
}

void setupMotors() {
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  analogWrite(LM1, 0);
  analogWrite(LM2, 0);
  analogWrite(RM1, 0);
  analogWrite(RM2, 0);
}

void setupSerial() {
  Serial. begin(9600);
}

void setupWifi() {
  WiFi.config(ip);
  
  Serial.begin(9600);
  while (!Serial) {
  }

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    delay(10000);
  }

  Serial.print("You're connected to the network");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void setup() {
  // put your setup code here, to run once:

  setupWifi();

  setupSerial(); // udp and debug

  //setupMatrices();

  setupStruct();

  setupIR();
  setupMotors();
}

void checkUDP() {

  //TODO: figure out how this works
  
}

int getSpeed(double velocity) {
  int countTicks = (cntR + cntL) / 2;

  float delta = (count * (1 / 75.8)) / 2;

  return (velocity - delta) * KpSpeed;
}

int getDirection(double theta) {
  float delta = theta - thetaGlobal;
  return (delta) * KpDirection;
}

void loop() {
  // put your main code here, to run repeatedly:

  //checkUDP
  //parseUDP
  //setSpeed
  //setDirection
  //setMotorLeft setMotorRight
  //ifPickedUp

}
