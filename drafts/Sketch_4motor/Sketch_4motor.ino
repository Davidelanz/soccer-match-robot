// Definitions Arduino pins connected to input H Bridge
// IN1-IN3:blue; IN2-IN4:gray
// Rear motors
#define RearLeftMotorDirPin1    24   //Rear Right Motor direction pin 1 to Rear MODEL-X IN1    (K1 = out1)
#define RearLeftMotorDirPin2    25   //Rear Right Motor direction pin 2 to Rear MODEL-X IN2    (K1 = out2)                                 
#define RearRightMotorDirPin1   30   //Rear left Motor direction pin 1 to Rear MODEL-X IN3     (K3 = out3)
#define RearRightMotorDirPin2   31   //Rear Left Motor direction pin 2 to Rear MODEL-X IN4     (K3 = out4)

#define vel_RearRight 7 // Pin for velocity control for rear right motor
#define vel_RearLeft  6 // Pin for velocity control for rear left motor


// Front motors
#define FrontLeftMotorDirPin1    36    //Front Right Motor direction pin 1 to Front MODEL-X IN1    (K1 = out1)
#define FrontLeftMotorDirPin2    37    //Front Right Motor direction pin 2 to Front MODEL-X IN2    (K1 = out2) 
#define FrontRightMotorDirPin1   42    //Front left Motor direction pin 1 to Front MODEL-X IN3     (K3 = out3)
#define FrontRightMotorDirPin2   43    //Front left Motor direction pin 2 to Front MODEL-X IN4     (K3 = out4)

#define vel_FrontRight 4 // Pin for velocity control for front right motor
#define vel_FrontLeft  5 // Pin for velocity control for front left motor

void setup()
{
  Serial.begin(115200); // set up Serial library at 115200 bps
// Set the output pins 
// Rear motors
// Right MOTOR (A)
pinMode(RearLeftMotorDirPin1, OUTPUT);
pinMode(RearLeftMotorDirPin2, OUTPUT);
// Left MOTOR (B)
pinMode(RearRightMotorDirPin1, OUTPUT);
pinMode(RearRightMotorDirPin2, OUTPUT);
// Front motors
// Right MOTOR (C)
pinMode(FrontLeftMotorDirPin1, OUTPUT);
pinMode(FrontLeftMotorDirPin2, OUTPUT);
// Left MOTOR (D)
pinMode(FrontRightMotorDirPin1, OUTPUT);
pinMode(FrontRightMotorDirPin2, OUTPUT);
// Velocity pins 
pinMode(vel_RearLeft, OUTPUT);
pinMode(vel_RearRight, OUTPUT);
pinMode(vel_FrontLeft, OUTPUT);
pinMode(vel_FrontRight, OUTPUT);

  

  Serial.println("<Arduino is ready>");
  //Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.

}

void loop()
{
// Rotate the Motor A clockwise
Serial.println("< Rotating rear left motor clockwise >");
digitalWrite(RearLeftMotorDirPin1, HIGH);
digitalWrite(RearLeftMotorDirPin2, LOW);
analogWrite(vel_RearLeft, 100);
delay(2000);
// Motor A
digitalWrite(RearLeftMotorDirPin1, HIGH);
digitalWrite(RearLeftMotorDirPin2, HIGH);
analogWrite(vel_RearLeft, 100);
delay(500);
// Serial.println("< Rotating rear left motor counter-clockwise >");
// digitalWrite(RearLeftMotorDirPin1, LOW);
// digitalWrite(RearLeftMotorDirPin2, HIGH);
// analogWrite(vel_RearLeft, 100);
// delay(2000);
// digitalWrite(RearLeftMotorDirPin1, HIGH);
// digitalWrite(RearLeftMotorDirPin2, HIGH);
// analogWrite(vel_RearLeft, 100);
// delay(500);
// Rotate the Motor B clockwise
Serial.println("< Rotating rear right motor >");
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, LOW);
analogWrite(vel_RearRight, 100);
delay(2000);
// Motor B
digitalWrite(RearRightMotorDirPin1, HIGH);
digitalWrite(RearRightMotorDirPin2, HIGH);
analogWrite(vel_RearRight, 100);
delay(500);
// Rotates the Motor C counter-clockwise
Serial.println("< Rotating front left motor >");
digitalWrite(FrontLeftMotorDirPin1, LOW);
digitalWrite(FrontLeftMotorDirPin2, HIGH);
analogWrite(vel_FrontLeft, 100);
delay(2000);
// Motor C
digitalWrite(FrontLeftMotorDirPin1, HIGH);
digitalWrite(FrontLeftMotorDirPin2, HIGH);
analogWrite(vel_FrontLeft, 100);
delay(500);
// Rotates the Motor D counter-clockwise
Serial.println("< Rotating front right motor >");
digitalWrite(FrontRightMotorDirPin1, LOW);
digitalWrite(FrontRightMotorDirPin2, HIGH);
analogWrite(vel_FrontRight, 100);
delay(2000);
// Motor D
digitalWrite(FrontRightMotorDirPin1, HIGH);
digitalWrite(FrontRightMotorDirPin2, HIGH);
analogWrite(vel_FrontRight, 100);
delay(500);
}
