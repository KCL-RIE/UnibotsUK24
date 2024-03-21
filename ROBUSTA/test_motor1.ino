int In1   = 50;//Pin no 7 of the arduino connects with motor driver's In1 pin//
int In2   = 52;//Pin no 8 of the arduino connects with motor driver's In2 pin//
int ENA   = 2;//Pin no 3 of the arduino connects with motor driver's ENA pin//
int SPEED = 50;//initialize the speed of the motor//

void setup() {
  // put your setup code here, to run once:
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(ENA,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveFront();
}

// Common function to Move the car forward
void moveFront()
{
  Serial.println(F("Move Front."));
  digitalWrite(In1,HIGH);// To change the direction of the motor by type low instead of High"//
  digitalWrite(In2,LOW);
  analogWrite(ENA,SPEED);//ENA of the motor controller connects with pwm pin 3 of arduino,its change the speed of the motor//
}
