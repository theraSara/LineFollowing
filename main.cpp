#include <Wire.h>
#define RightMotor 10
#define LeftMotor 5
#define MaxOutput 255
#define ObstacleDistance 200
#define Distance 100

enum {LineFollow, FollowMe};
int current = LineFollow; 

uint8_t barRaw; 
int position; 
CircularBuffer positionHistory(100);

double Setpoint, Input, Output;

double Kp1 = 0.1, Ki1 = 0.01, Kd1 = 0.001;
double Kp2 = 0.1, Ki2 = 0.01, Kd2 = 0.001;

PID myPID1(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT);
PID myPID2(&Input, &Output, &Setpoint, Kp2, Ki2, Kd2, DIRECT);

void AutomaticDrive()
{
if(Output > 0)
{
if(Output > MaxOutput) Output = MaxOutput;
analogWrite(LeftMotor, Output);
analogWrite(LeftMotor+1, 0);  
analogWrite(RightMotor, Output);
analogWrite(RightMotor+1, 0);      
} 
else if (Output < 0)
{
if(-Output > MaxOutput) Output = MaxOutput;
analogWrite(LeftMotor, 0);
analogWrite(LeftMotor+1, -Output);  
analogWrite(RightMotor, 0);
analogWrite(RightMotor+1, -Output); 
}
else if (Output == 0)
{
analogWrite(LeftMotor, 0);
analogWrite(LeftMotor+1, 0);  
analogWrite(RightMotor, 0);
analogWrite(RightMotor+1, 0); 
}
}

long readUltrasonicDistance(int triggerPin, int echoPin)
{
pinMode(triggerPin, OUTPUT); 
digitalWrite(triggerPin, LOW);
delayMicroseconds(2);
digitalWrite(triggerPin, HIGH);
delayMicroseconds(10);
digitalWrite(triggerPin, LOW);
pinMode(echoPin, INPUT);
return pulseIn(echoPin, HIGH);
}
//============= POSITIONING =============
int8_t getPosition(uint8_t barRaw)
{
int16_t accumulator = 0;
uint8_t bitsCounted = 0;
int16_t i;
uint8_t lastBarPositionValue; 
uint8_t lastBarRawValue = ~barRaw;
for ( i = 0; i < 8; i++ )
if ( ((lastBarRawValue >> i) & 0x01) == 1 )
bitsCounted++;
for ( i = 7; i > 3; i-- )
if ( ((lastBarRawValue >> i) & 0x01) == 1 )
accumulator += ((-32 * (i - 3)) + 1);
for ( i = 0; i < 4; i++ )
if ( ((lastBarRawValue >> i) & 0x01) == 1 )
accumulator += ((32 * (4 - i)) - 1);
if ( bitsCounted > 0 )
lastBarPositionValue = accumulator / bitsCounted;
else
lastBarPositionValue = 0;
return lastBarPositionValue;
}
//============= Read RAW Bar =============
void requestSensorRawReading()
{
Wire.beginTransmission(0x3E);
Wire.write(240);
Wire.endTransmission();  
}
void receiveEvent(int howMany)
{
while(Wire.available()>0)
{
barRaw = Wire.read(); 
current = LineFollow; 
}
}
//============= PRINT METER =============
void displayMeter(int avePos)
{
Serial.print("Meter: ");
for(int i = -130; i <= 130; i = i + 5 )
{
if( i < 0 )
{
if((avePos > (i - 3)) && (avePos <= (i + 2)))
  Serial.print("*");
else
  Serial.print("-");
}
else if( i == 0 )
{
if((avePos > (i - 3)) && (avePos <= (i + 2)))
   Serial.print("*");
else
  Serial.print("+");
}
else if( i > 0 )
{
if((avePos > (i - 3)) && (avePos <= (i + 2)))
    Serial.print("*");
else
  Serial.print("-");
}
}
Serial.print(" avePos = ");
Serial.println(avePos);
}
//============= Display RAW Sensor =============
void displayRawSensor(uint8_t barRaw)
{
for( int i = 7; i >= 0; i-- )
Serial.print((~barRaw >> i) & 0x01);
Serial.println("b"); 
}
int forwardTimer = 0; 
//============= SETUP and CONFIGURATION =============
void setup()
{
Serial.begin(9600); 
Wire.begin(81);
Wire.onReceive(receiveEvent);
pinMode(LeftMotor, OUTPUT);
pinMode(LeftMotor+1, OUTPUT); 
pinMode(RightMotor, OUTPUT);
pinMode(RightMotor+1, OUTPUT); 

// PID Controller 1:
myPID1.SetTunings(Kp1, Ki1, Kd1, P_ON_E);
myPID1.SetMode(AUTOMATIC);
myPID1.SetOutputLimits(-MaxOutput,MaxOutput);

// PID Controller 2:
myPID2.SetTunings(Kp2, Ki2, Kd2, P_ON_E);
myPID2.SetMode(AUTOMATIC);
myPID2.SetOutputLimits(-MaxOutput,MaxOutput);
Serial.println("Robot is Up");
}
//============= LOOP =============
void loop()
{
switch(current)
{
case LineFollow:
  Serial.println("Line Follow Mode");
  delay(1000);
  requestSensorRawReading();
  Input = getPosition(barRaw);
  positionHistory.pushElement(Input);
  if (positionHistory.recordLength() > 10)
      Input = positionHistory.averageLast(10);
  Setpoint = ObstacleDistance;
  myPID1.Compute();
  AutomaticDrive();
  displayMeter(Input);
  if ((0.01723 * readUltrasonicDistance(4, 4)) < ObstacleDistance)
    current = FollowMe;
break;

case FollowMe:
  Serial.println("Follow Me Mode");
  requestSensorRawReading();
  Input = getPosition(barRaw);
  positionHistory.pushElement(Input);
  if (positionHistory.recordLength() > 10)
      Input = positionHistory.averageLast(10);
  Setpoint = Distance;
  myPID2.Compute();
  AutomaticDrive();
  if ((0.01723 * readUltrasonicDistance(4, 4)) > Distance)
  {
    current = LineFollow;
    myPID1.Initialize();
  }
break;   
}
}
