#include <PID_v1.h>

//*************pin assignement**************

const int motorPWM = 3;
const int motorClockwise = 4;
const int motorAntiClockwise = 5;

const int currentSensor = A1;
const int springLeft = A2;
const int springRight = A3;
const int rotoryPot = A4;


//---------- variable declaration ------------

double Setpoint, Input, Output;

double Kt;   // spring constant
double Kx;   // length of linear pot
double Kti; // motor torque-current constant

double Kp;
double Ki;
double Kd;

int nrReadings;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//------------ initialization --------------

void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(motorClockwise, OUTPUT);
  pinMode(motorAntiClockwise, OUTPUT);
  
  pinMode(currentSensor, INPUT);
  pinMode(springLeft, INPUT);
  pinMode(springRight, INPUT);
  pinMode(rotoryPot, INPUT);

  nrReadings = 5;

  Serial.begin(9600);
  SerialSend();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 150);
  myPID.SetSampleTime(10);
  myPID.SetControllerDirection(DIRECT);
   
  digitalWrite(motorClockwise, LOW);
  digitalWrite(motorAntiClockwise, HIGH);
}

//----------------- process ------------------
void loop() {
  SerialReceive();
  
  Motormove();
  
  SerialSend();
}

//----------------- movement ------------------

void Motormove() {
  Input = getInput();
  myPID.Compute();
  analogWrite(motorPWM, Output);
  delay(20);
}



double getInput() {
  double sum;
  double avg;
  double avgInput;
  
  for (int i = 0; i <= nrReadings; i++) {
    sum += analogRead(currentSensor);       
    delayMicroseconds(10);
  }
  avgInput = sum/nrReadings;
  sum = 0;
  return avgInput;
}



float getTorque(){
  float torque;
  int SLeft, SRight, diff;
  
  SLeft = analogRead(springLeft);
  SRight = analogRead(springRight);
  
  diff = SLeft - SRight;
  
  torque = (Kt * Kx * diff)/1024;
  return torque;
}



float getAngle(){
  int reading;
  float angle;
  
  reading = analogRead(rotoryPot);

  angle = 180.0 * (reading/1024);
  
  return angle;
}

//----------------- communication ----------------------

//  the bytes coming from the arduino follow the following format:
//  0-3: float joint 1

union {                // This Data structure lets
  byte asBytes[4];    // us take the byte array
  float asFloat[1];    // sent from processing and
}                      // easily convert it to a
foo;  



void SerialReceive()
{
  double temp, temp2;
  int index=0;
  
  while(Serial.available()&&index<1)
  {
    foo.asBytes[index] = Serial.read();
    index++;
  }

  if(index==1)
  {
    temp = float(foo.asFloat[0]);
    temp2 = (temp*1024)/(Kti*20);
  }

  if (temp2>=0){
    myPID.SetControllerDirection(DIRECT);
    digitalWrite(motorClockwise, LOW);
    digitalWrite(motorAntiClockwise, HIGH);
    Setpoint = temp2;
  } else {
    myPID.SetControllerDirection(REVERSE);
    digitalWrite(motorClockwise, HIGH);
    digitalWrite(motorAntiClockwise, LOW);
    Setpoint = abs(temp2);
  }
  Serial.flush();                         
}



void SerialSend()
{
  Serial.print("Torque ");
  Serial.print(getTorque());   
  Serial.print(" ");  
  Serial.print("Angle ");
  Serial.print(getAngle());   
  Serial.print(" ");
}
