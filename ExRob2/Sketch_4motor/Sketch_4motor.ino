// Definitions Arduino pins connected to input H Bridge

// Rear motors
#define RearRightMotorDirPin1  22   //Front Right Motor direction pin 1 to Front MODEL-X IN1    (K1 = out1)
#define RearRightMotorDirPin2  24   //Front Right Motor direction pin 2 to Front MODEL-X IN2    (K1 = out2)                                 
#define RearLeftMotorDirPin3   26   //Front left Motor direction pin 1 to Front MODEL-X IN3     (K3 = out3)
#define RearLeftMotorDirPin4   28   //Front Left Motor direction pin 2 to Front MODEL-X IN4     (K3 = out4)
/*int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;*/

// Front motors
#define FrontRightMotorDirPin1  5    //Rear Right Motor direction pin 1 to Back MODEL-X IN1    (K1 = out1)
#define FrontRightMotorDirPin2  6    //Rear Right Motor direction pin 2 to Back MODEL-X IN2    (K1 = out2) 
#define FrontLeftMotorDirPin3   7    //Rear left Motor direction pin 1 to Back MODEL-X IN3     (K3 = out3)
#define FrontLeftMotorDirPin4   8    //Rear left Motor direction pin 2 to Back MODEL-X IN4     (K3 = out4)

void setup()
{
  Serial.begin(115200); // set up Serial library at 115200 bps
// Set the output pins 
// Rear motors
// MOTOR A
pinMode(RearRightMotorDirPin1, OUTPUT);
pinMode(RearRightMotorDirPin2, OUTPUT);
// MOTOR B
pinMode(RearLeftMotorDirPin3, OUTPUT);
pinMode(RearLeftMotorDirPin4, OUTPUT);
// Front motors
// MOTOR C
pinMode(FrontRightMotorDirPin1, OUTPUT);
pinMode(FrontRightMotorDirPin2, OUTPUT);
// MOTOR D
pinMode(FrontLeftMotorDirPin3, OUTPUT);
pinMode(FrontLeftMotorDirPin4, OUTPUT);
}

void loop()
{
// Rotate the Motor A clockwise
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, LOW);
delay(2000);
// Motor A
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, HIGH);
delay(500);
// Rotate the Motor B clockwise
digitalWrite(RearLeftMotorDirPin3, HIGH);
digitalWrite(RearLeftMotorDirPin4, LOW);
delay(2000);
// Motor B
digitalWrite(RearLeftMotorDirPin3, HIGH);
digitalWrite(RearLeftMotorDirPin4, HIGH);
delay(500);
// Rotates the Motor C counter-clockwise
digitalWrite(FrontRightMotorDirPin1, LOW);
digitalWrite(FrontRightMotorDirPin2, HIGH);
delay(2000);
// Motor C
digitalWrite(FrontRightMotorDirPin1, HIGH);
digitalWrite(FrontRightMotorDirPin2, HIGH);
delay(500);
// Rotates the Motor D counter-clockwise
digitalWrite(FrontLeftMotorDirPin3, LOW);
digitalWrite(FrontLeftMotorDirPin4, HIGH);
delay(2000);
// Motor B
digitalWrite(FrontLeftMotorDirPin3, HIGH);
digitalWrite(FrontLeftMotorDirPin4, HIGH);
delay(500);
}
