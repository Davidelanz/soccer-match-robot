#include <string.h>

// Definitions Arduino pins connected to input H Bridge
// IN1-IN3:blue; IN2-IN4:gray
// Rear motors
#define RearLeftMotorDirPin1 24  //Rear Left Motor direction pin 1 to Rear MODEL-X IN1    (K1 = out1)
#define RearLeftMotorDirPin2 25  //Rear Left Motor direction pin 2 to Rear MODEL-X IN2    (K1 = out2)
#define RearRightMotorDirPin1 30 //Rear Right Motor direction pin 1 to Rear MODEL-X IN3     (K3 = out3)
#define RearRightMotorDirPin2 31 //Rear Right Motor direction pin 2 to Rear MODEL-X IN4     (K3 = out4)
// Rear motors' encoders
#define encA_RR 22 // Encoder A for rear right motor
#define encB_RR 23 // Encoder B for rear right motor
#define encA_RL 28 // Encoder A for rear left motor
#define encB_RL 29 // Encoder B for rear left motor

#define vel_RearRight 7 // Pin for velocity control for rear right motor
#define vel_RearLeft 6  // Pin for velocity control for rear left motor

// Front motors
#define FrontLeftMotorDirPin1 36  //Front Left Motor direction pin 1 to Front MODEL-X IN1    (K1 = out1)
#define FrontLeftMotorDirPin2 37  //Front Left Motor direction pin 2 to Front MODEL-X IN2    (K1 = out2)
#define FrontRightMotorDirPin1 42 //Front Right Motor direction pin 1 to Front MODEL-X IN3     (K3 = out3)
#define FrontRightMotorDirPin2 43 //Front Right Motor direction pin 2 to Front MODEL-X IN4     (K3 = out4)

// Front motors' encoders
#define encA_FR 34 // Encoder A for front right motor
#define encB_FR 35 // Encoder B for front right motor
#define encA_FL 40 // Encoder A for front left motor
#define encB_FL 41 // Encoder B for front left motor


#define vel_FrontRight 4 // Pin for velocity control for front right motor
#define vel_FrontLeft 5  // Pin for velocity control for front left motor

void setup() {
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
  /*pinMode(encA_RL, OUTPUT);
  pinMode(encB_RL, OUTPUT);
  pinMode(encA_RR, OUTPUT);
  pinMode(encB_RR, OUTPUT);
  pinMode(encA_FL, OUTPUT);
  pinMode(encB_FL, OUTPUT);
  pinMode(encA_FR, OUTPUT);
  pinMode(encB_FR, OUTPUT);*/

  Serial.println("<Arduino is ready>");
  //Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.
}


// function to control motor
// speed is how fast the motor rotates
// Please set pwmPin, InaPin and INbPin for the motor you want to drive
void control_motor(int speed, int pwmPin, int INaPin, int INbPin){
    if(speed > 0){
        digitalWrite(INaPin, HIGH);
        digitalWrite(INbPin, LOW);
        analogWrite(pwmPin, speed);
    }
    else if(speed < 0){
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, HIGH);
        analogWrite(pwmPin, -speed);
    }
    else{
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, LOW);
    }
}

// In time loop, receive from serial and control 6 motors
void loop() {
    static int speed[4];
    static char buff[30];
    int counter = 0;

    // read command from raspberry pi
    while(Serial.available()){
        buff[counter] = Serial.read();
        if (counter > 30 || buff[counter] == '*') {
            buff[counter] = '\0';
            speed[0]=atoi(strtok(buff,","));
            speed[1]=atoi(strtok(NULL,","));
            speed[2]=atoi(strtok(NULL,","));
            speed[3]=atoi(strtok(NULL,","));
            //speed[4]=atoi(strtok(NULL,","));
            //speed[5]=atoi(strtok(NULL,","));
        }
        else{
            counter++;
        }
    }

    //Serial.println("<Speed is", + speed[0]);
    // control motors
    control_motor(speed[0], vel_FrontLeft, FrontLeftMotorDirPin1, FrontLeftMotorDirPin2);
    control_motor(speed[1], vel_FrontRight, FrontRightMotorDirPin1, FrontRightMotorDirPin2);
    control_motor(speed[2], vel_RearLeft, RearLeftMotorDirPin1, RearLeftMotorDirPin2);
    control_motor(speed[3], vel_RearRight, RearRightMotorDirPin1, RearRightMotorDirPin2);
    //control_motor(speed[4], pwmPin4, INaPin4, INbPin4);
    //control_motor(speed[5], pwmPin5, INaPin5, INbPin5);

    // send messages to raspberry pi
    Serial.print(speed[0]); Serial.print(",");
    Serial.print(speed[1]); Serial.print(",");
    Serial.print(speed[2]); Serial.print(",");
    Serial.println(speed[3]);// Serial.print(",");
    //Serial.print(speed[4]); Serial.print(",");
    //Serial.println(speed[5]);

    delay(100);
    
}
