#include <QTRSensors.h>
#include <Servo.h>

Servo Fan;

#define Esc 9
int FanSpeed = 0; //0-180

//  Motor Left
#define motorL1 4
#define motorL2 3
#define motorL_PWM 5

// Motor Right
#define motorR1 7
#define motorR2 8
#define motorR_PWM 6




QTRSensors qtr;
byte Ain[] = {7, 8, 6};
byte Bin[] = {4, 3, 5};

int MaxSpeed = 110;
int BaseSpeed = 80;
int minSpeed = -75;

double motorSpeedA = 0;
double motorSpeedB = 0;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int P;
int I;
int D;
int a = 1000;

float Kp = 0.1;
float Ki = 0.0001;
float Kd = 0.65;

float aKi=a*Ki;

int lastError = 0;

int cnt = 0;
int zai_medregch = 2;
int buttonState = 0;
int lastButtonState = 0; 
int state = 0;
int incomingByte = 0;
bool Enable = false;
bool serialEnable = false;
void setup()
{  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2, A3, A4, A5, A6,A7}, SensorCount);
  qtr.setEmitterPin(13);

  for (int i = 0; i < 3; i++){
    pinMode(Ain[i],OUTPUT);
    pinMode(Bin[i],OUTPUT);
  }
  pinMode (zai_medregch, INPUT);
  delay(500); 

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);

Fan.attach(Esc);

}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error =  positionLine - 3500;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = BaseSpeed + motorSpeedChange;
  int motorSpeedB = BaseSpeed - motorSpeedChange;

  if (motorSpeedA > MaxSpeed) {
    motorSpeedA = MaxSpeed;
  }
  if (motorSpeedB > MaxSpeed) {
    motorSpeedB = MaxSpeed;
  }
  if (motorSpeedA < -75) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    motorSpeedA = minSpeed;
  }
  if (motorSpeedB < -75) {
    motorSpeedB = minSpeed;
  }
  if (Enable == true){

  forward_movement(motorSpeedA, motorSpeedB);
  }
}

void forward_movement(int speedA, int speedB) {
  // LEFT motor (FORWARD INVERTED)
  if (speedA < 0) {
    speedA = -speedA;
    digitalWrite(motorL1, HIGH);
    digitalWrite(motorL2, LOW);
  } else {
    digitalWrite(motorL1, LOW);
    digitalWrite(motorL2, HIGH);
  }

  // RIGHT motor (FORWARD INVERTED)
  if (speedB < 0) {
    speedB = -speedB;
    digitalWrite(motorR1, LOW);
    digitalWrite(motorR2, HIGH);
  } else {
    digitalWrite(motorR1, HIGH);
    digitalWrite(motorR2, LOW);
  }

  analogWrite(motorL_PWM, speedA);
  analogWrite(motorR_PWM, speedB);
}



void stop (){
  digitalWrite(motorR2, HIGH);
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, HIGH);
  analogWrite(motorL_PWM, 0);
  analogWrite(motorR_PWM, 0);
}

void motorturn(){
  // RIGHT motor forward
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);

  // LEFT motor backward
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);

  analogWrite(motorL_PWM, 100);
  analogWrite(motorR_PWM, 100);
}


void ready (){
   buttonState = digitalRead(zai_medregch);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      cnt++;
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  if(cnt==1){
    stop();
  }
  else if(cnt<=2){
    PID_control();
  }
  else stop();
}

void loop()
{
  //ready();
  PID_control();
  Fan.write(FanSpeed);
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    if (state == 1)
    {
      if (incomingByte == 'u')
      {
        Kp += 0.01;
        Serial.println(Kp);
      }
      else if (incomingByte == 'd')
      {
        Kp -= 0.01;
        Serial.println(Kp);
      }
    }
    else if (state == 2)
    {
      if (incomingByte == 'u')
      {
        Ki += 0.0001;
        Serial.println(Ki);
      }
      else if (incomingByte == 'd')
      {
        Ki -= 0.0001;
        Serial.println(Ki);
      }
    }
    else if (state == 3)
    {
      if (incomingByte == 'u')
      {
        Kd += 0.10;
        Serial.println(Kd);
      }
      else if (incomingByte == 'd')
      {
        Kd -= 0.10;
        Serial.println(Kd);
      }
    }
    else if (state == 4)
    {
      if (incomingByte == 'u')
      {
        BaseSpeed += 10;
        Serial.println(BaseSpeed);
      }
      else if (incomingByte == 'd')
      {
        BaseSpeed -= 10;
        Serial.println(BaseSpeed);
      }
    }
    else if (state == 5)
    {
      if (incomingByte == 'u')
      {
        MaxSpeed += 10;
        Serial.println(MaxSpeed);
      }
      else if (incomingByte == 'd')
      {
        MaxSpeed -= 10;
        Serial.println(MaxSpeed);
      }
    }
    else if (state == 6)
    {
      if (incomingByte == 'u')
      {
        FanSpeed += 5;
        Serial.println(FanSpeed);
      }
      else if (incomingByte == 'd')
      {
        FanSpeed -= 5;
        Serial.println(FanSpeed);
      }
    }
    
    switch (incomingByte)
    {     
    case 'P':
      state = 1;
      Serial.println(Kp);
      break;
    case 'I':
      state = 2;
      Serial.println(aKi);
      break;
    case 'D':
      state = 3;
      Serial.println(Kd);
      break;
    case 'B':
      state = 4;
      Serial.println(BaseSpeed);
      break;
    case 'M':
      state = 5;
      Serial.println(MaxSpeed);
      break;
    case 'E':

      state = 6;
      Serial.println(FanSpeed);
      
      break;
    case 'Z':
      
      Kp = 0.05;
      Ki = 0.0008;
      Kd = 0.3;
      MaxSpeed = 110;
      BaseSpeed = 80;
      minSpeed = -75;
      FanSpeed = 40;

      break;
  
    case 's': //start
      Enable = true;
      break;
    case 'O':
      Enable = false;
      stop();
      
      break;
    case 'x':
      serialEnable = true;
      break;
    case 'k':
      serialEnable = false;
      break;
    default:
      break;
    }
    incomingByte = 0;
  }
}

  


