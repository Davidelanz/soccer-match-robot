// Definitions Arduino pins connected to input H Bridge

// Rear motors
#define RearRightMotorDirPin1  24   //Rear Right Motor direction pin 1 to Rear MODEL-X IN1    (K1 = out1)
#define RearRightMotorDirPin2  25   //Rear Right Motor direction pin 2 to Rear MODEL-X IN2    (K1 = out2)                                 
#define RearLeftMotorDirPin3   30   //Rear left Motor direction pin 1 to Rear MODEL-X IN3     (K3 = out3)
#define RearLeftMotorDirPin4   31   //Rear Left Motor direction pin 2 to Rear MODEL-X IN4     (K3 = out4)

#define vel_RearRight 7 // Pin for velocity control for rear right motor
#define vel_RearLeft  6 // Pin for velocity control for rear left motor


// Front motors
#define FrontRightMotorDirPin1  36    //Front Right Motor direction pin 1 to Front MODEL-X IN1    (K1 = out1)
#define FrontRightMotorDirPin2  37    //Front Right Motor direction pin 2 to Front MODEL-X IN2    (K1 = out2) 
#define FrontLeftMotorDirPin3   40    //Front left Motor direction pin 1 to Front MODEL-X IN3     (K3 = out3)
#define FrontLeftMotorDirPin4   41    //Front left Motor direction pin 2 to Front MODEL-X IN4     (K3 = out4)

#define vel_FrontRight 5 // Pin for velocity control for front right motor
#define vel_FrontLeft  4 // Pin for velocity control for front left motor

void setup()
{
  Serial.begin(115200); // set up Serial library at 115200 bps
// Set the output pins 
// Rear motors
// Right MOTOR (A)
pinMode(RearRightMotorDirPin1, OUTPUT);
pinMode(RearRightMotorDirPin2, OUTPUT);
// Left MOTOR (B)
pinMode(RearLeftMotorDirPin3, OUTPUT);
pinMode(RearLeftMotorDirPin4, OUTPUT);
// Front motors
// Right MOTOR (C)
pinMode(FrontRightMotorDirPin1, OUTPUT);
pinMode(FrontRightMotorDirPin2, OUTPUT);
// Left MOTOR (D)
pinMode(FrontLeftMotorDirPin3, OUTPUT);
pinMode(FrontLeftMotorDirPin4, OUTPUT);
// Velocity pins 
pinMode(vel_RearRight, OUTPUT);
pinMode(vel_RearLeft, OUTPUT);
pinMode(vel_FrontRight, OUTPUT);
pinMode(vel_FrontLeft, OUTPUT);
  

  Serial.println("<Arduino is ready>");
  Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.

}

void loop()
{
// Rotate the Motor A clockwise
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, LOW);
analogWrite(vel_RearRight, 200);
delay(2000);
// Motor A
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, HIGH);
analogWrite(vel_RearRight, 200);
delay(500);
// Rotate the Motor B clockwise
digitalWrite(RearLeftMotorDirPin3, HIGH);
digitalWrite(RearLeftMotorDirPin4, LOW);
analogWrite(vel_RearLeft, 200);
delay(2000);
// Motor B
digitalWrite(RearLeftMotorDirPin3, HIGH);
digitalWrite(RearLeftMotorDirPin4, HIGH);
analogWrite(vel_RearLeft, 200);
delay(500);
// Rotates the Motor C counter-clockwise
digitalWrite(FrontRightMotorDirPin1, LOW);
digitalWrite(FrontRightMotorDirPin2, HIGH);
analogWrite(vel_FrontRight, 200);
delay(2000);
// Motor C
digitalWrite(FrontRightMotorDirPin1, HIGH);
digitalWrite(FrontRightMotorDirPin2, HIGH);
analogWrite(vel_FrontRight, 200);
delay(500);
// Rotates the Motor D counter-clockwise
digitalWrite(FrontLeftMotorDirPin3, LOW);
digitalWrite(FrontLeftMotorDirPin4, HIGH);
analogWrite(vel_FrontLeft, 200);
delay(2000);
// Motor B
digitalWrite(FrontLeftMotorDirPin3, HIGH);
digitalWrite(FrontLeftMotorDirPin4, HIGH);
analogWrite(vel_FrontLeft, 200);
delay(500);
}
