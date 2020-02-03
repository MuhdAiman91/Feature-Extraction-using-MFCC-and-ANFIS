int ledPin=A0;
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (Serial.available()>0)
  {
    int baca = Serial.read();
    if (baca == 'a')
    {
      digitalWrite(ledPin,LOW);
      Serial.println("NYALA");
    }
    else if (baca == 'b')
    {
      digitalWrite(ledPin,HIGH);
      Serial.println("MATI");
    }
    else if (baca == 'c')
    {
      digitalWrite(ledPin,HIGH);
      Serial.println("TAK DIKENAL");
    }
  }
}
