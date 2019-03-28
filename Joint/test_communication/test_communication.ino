void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

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
