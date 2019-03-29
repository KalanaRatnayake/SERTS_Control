float Setpoint;

void setup() {
  Serial.begin(9600);
}

void loop() {
  SerialReceive();
  
  SerialSend();
}


union {
  byte asBytes[4];
  float asFloat[1];
}
foo;  

void SerialReceive()
{
  int index=0;
  
  while(Serial.available()&&index<4)
  {
    foo.asBytes[index] = Serial.read();
    index++;
  }

  if(index==4)
  {
    Setpoint = float(foo.asFloat[0]);
  }
  Serial.flush();                         
}



void SerialSend()
{
  Serial.print("\nValue ");
  Serial.print( Setpoint);  
}
