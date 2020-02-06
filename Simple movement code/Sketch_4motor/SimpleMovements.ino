// Definitions Arduino pins connected to input H Bridge
// IN1-IN3:blue; IN2-IN4:gray
// Rear motors
#define RearLeftMotorDirPin1 24  //Rear Right Motor direction pin 1 to Rear MODEL-X IN1    (K1 = out1)
#define RearLeftMotorDirPin2 25  //Rear Right Motor direction pin 2 to Rear MODEL-X IN2    (K1 = out2)
#define RearRightMotorDirPin1 30 //Rear left Motor direction pin 1 to Rear MODEL-X IN3     (K3 = out3)
#define RearRightMotorDirPin2 31 //Rear Left Motor direction pin 2 to Rear MODEL-X IN4     (K3 = out4)
// Rear motors' encoders
#define encA_RL 22 // Encoder A for rear left motor
#define encB_RL 23 // Encoder B for rear left motor
#define encA_RR 28 // Encoder A for rear right motor
#define encB_RR 29 // Encoder B for rear right motor

#define vel_RearRight 7 // Pin for velocity control for rear right motor
#define vel_RearLeft 6  // Pin for velocity control for rear left motor

// Front motors
#define FrontLeftMotorDirPin1 36  //Front Right Motor direction pin 1 to Front MODEL-X IN1    (K1 = out1)
#define FrontLeftMotorDirPin2 37  //Front Right Motor direction pin 2 to Front MODEL-X IN2    (K1 = out2)
#define FrontRightMotorDirPin1 42 //Front left Motor direction pin 1 to Front MODEL-X IN3     (K3 = out3)
#define FrontRightMotorDirPin2 43 //Front left Motor direction pin 2 to Front MODEL-X IN4     (K3 = out4)

// Front motors' encoders
#define encA_FL 34 // Encoder A for front left motor
#define encB_FL 35 // Encoder B for front left motor
#define encA_FR 40 // Encoder A for front right motor
#define encB_FR 41 // Encoder B for front right motor

#define vel_FrontRight 4 // Pin for velocity control for front right motor
#define vel_FrontLeft 5  // Pin for velocity control for front left motor

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

  // Encoders pins
  pinMode(encA_RL, OUTPUT);
  pinMode(encB_RL, OUTPUT);
  pinMode(encA_RR, OUTPUT);
  pinMode(encB_RR, OUTPUT);
  pinMode(encA_FL, OUTPUT);
  pinMode(encB_FL, OUTPUT);
  pinMode(encA_FR, OUTPUT);
  pinMode(encB_FR, OUTPUT);

  Serial.println("<Arduino is ready>");
  //Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.
}

void loop()
{
  goAhead();
  goAhead();
  goAhead();
  goAhead();
  delay(1000);
  goBack();
  delay(1000);
  goLeft();
  delay(1000);
  goRight();
  delay(1000);
  turnAround();
  turnAround();
  delay(1000);
}


void turnAround()
{
  Serial.println("< Turning around >");
  // rear left motor clockwise
  digitalWrite(RearLeftMotorDirPin1, HIGH);
  digitalWrite(RearLeftMotorDirPin2, LOW);
  analogWrite(vel_RearLeft, 200);
  // rear right motor counter-clockwise
  digitalWrite(RearRightMotorDirPin1, LOW);
  digitalWrite(RearRightMotorDirPin2, HIGH);
  analogWrite(vel_RearRight, 200);
  // front left motor clockwise
  digitalWrite(FrontLeftMotorDirPin1, HIGH);
  digitalWrite(FrontLeftMotorDirPin2, LOW);
  analogWrite(vel_FrontLeft, 200);
  // front right motor counter-clockwise
  digitalWrite(FrontRightMotorDirPin1, LOW);
  digitalWrite(FrontRightMotorDirPin2, HIGH);
  analogWrite(vel_FrontRight, 200);
}

void goLeft()
{
  Serial.println("< Going left >");
  // rear left motor counter-clockwise
  digitalWrite(RearLeftMotorDirPin1, LOW);
  digitalWrite(RearLeftMotorDirPin2, HIGH);
  analogWrite(vel_RearLeft, 100);
  // rear right motor clockwise
  digitalWrite(RearRightMotorDirPin1, HIGH);
  digitalWrite(RearRightMotorDirPin2, LOW);
  analogWrite(vel_RearRight, 100);
  // front left motor clockwise
  digitalWrite(FrontLeftMotorDirPin1, HIGH);
  digitalWrite(FrontLeftMotorDirPin2, LOW);
  analogWrite(vel_FrontLeft, 100);
  // front right motor counter-clockwise
  digitalWrite(FrontRightMotorDirPin1, LOW);
  digitalWrite(FrontRightMotorDirPin2, HIGH);
  analogWrite(vel_FrontRight, 100);
}

void goRight()
{
  Serial.println("< Going right >");
  // rear left motor clockwise
  digitalWrite(RearLeftMotorDirPin1, HIGH);
  digitalWrite(RearLeftMotorDirPin2, LOW);
  analogWrite(vel_RearLeft, 100);
  // rear right motor counter-clockwise
  digitalWrite(RearRightMotorDirPin1, LOW);
  digitalWrite(RearRightMotorDirPin2, HIGH);
  analogWrite(vel_RearRight, 100);
  // front left motor counter-clockwise
  digitalWrite(FrontLeftMotorDirPin1, LOW);
  digitalWrite(FrontLeftMotorDirPin2, HIGH);
  analogWrite(vel_FrontLeft, 100);
  // front right motor clockwise
  digitalWrite(FrontRightMotorDirPin1, HIGH);
  digitalWrite(FrontRightMotorDirPin2, LOW);
  analogWrite(vel_FrontRight, 100);
}

void goAhead()
{
  Serial.println("< Going ahead >");
  // rear left motor clockwise
  digitalWrite(RearLeftMotorDirPin1, HIGH);
  digitalWrite(RearLeftMotorDirPin2, LOW);
  analogWrite(vel_RearLeft, 200);
  // rear right motor clockwise
  digitalWrite(RearRightMotorDirPin1, HIGH);
  digitalWrite(RearRightMotorDirPin2, LOW);
  analogWrite(vel_RearRight, 200);
  // front left motor clockwise
  digitalWrite(FrontLeftMotorDirPin1, HIGH);
  digitalWrite(FrontLeftMotorDirPin2, LOW);
  analogWrite(vel_FrontLeft, 200);
  // front right motor clockwise
  digitalWrite(FrontRightMotorDirPin1, HIGH);
  digitalWrite(FrontRightMotorDirPin2, LOW);
  analogWrite(vel_FrontRight, 200);
}

void goBack()
{
  Serial.println("< Going backwards >");
  // rear left motor counter-clockwise
  digitalWrite(RearLeftMotorDirPin1, LOW);
  digitalWrite(RearLeftMotorDirPin2, HIGH);
  analogWrite(vel_RearLeft, 100);
  // rear right motor counter-clockwise
  digitalWrite(RearRightMotorDirPin1, LOW);
  digitalWrite(RearRightMotorDirPin2, HIGH);
  analogWrite(vel_RearRight, 100);
  // front left motor counter-clockwise
  digitalWrite(FrontLeftMotorDirPin1, LOW);
  digitalWrite(FrontLeftMotorDirPin2, HIGH);
  analogWrite(vel_FrontLeft, 100);
  // front right motor counter-clockwise
  digitalWrite(FrontRightMotorDirPin1, LOW);
  digitalWrite(FrontRightMotorDirPin2, HIGH);
  analogWrite(vel_FrontRight, 100);
}