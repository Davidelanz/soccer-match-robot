
#define encoderA 22
#define encoderB 23
#define motorPin 2
#define en1 24
#define en2 25
#define LOOPTIME 100  

int stateA = 0;
int stateB = 0;
int counter = 0;
unsigned long t_start;
double vel=0;
String sw="";
double actvel=0;
int countAnt=0;
int PWM_val = 0; 
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
double speed_req = 0.0;                            // speed (Set Point)
double speed_act = 0.0;                              // speed (actual value)
double Kp =   16.0;                                // PID proportional control Gain
double Kd =    4.0;                                // PID Derivitave control gain
double last_error=0.0;



void setup() {
  Serial.begin(115200);           // set up Serial library at 115200 bps
  pinMode(motorPin, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode (encoderA, INPUT);
  pinMode (encoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), checkA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), checkB, CHANGE);
  digitalWrite(en1, LOW);
  digitalWrite(en2, HIGH);
  t_start = millis();
  //analogWrite(motorPin, 200);
  Serial.println("<Arduino is ready>");
  Serial.flush();
}

void loop() {
  
  String serialResponse ="";

  if (Serial.available()) {
    serialResponse  = Serial.readString();
    vel = serialResponse.substring(2,5).toDouble();
    sw = serialResponse.substring(1,2);
    if(sw=="+"){
    digitalWrite(en1, LOW);
    digitalWrite(en2, HIGH);
    }
    else if (sw=="-"){
    digitalWrite(en1, HIGH);
    digitalWrite(en2, LOW);
    }
    speed_req=vel;
  }
   if((millis()-lastMilli) >= LOOPTIME)   { 
      
       lastMilli = millis();
       speed_act = (double)(((double)(counter - countAnt)*(1000.0/(double)LOOPTIME))/(double)(320.0)); 
       countAnt=counter;
       PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
       analogWrite(motorPin, PWM_val);  
     }
     printMotorInfo();  
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                     
   lastMilliPrint = millis();
   Serial.print("< RPM:");          Serial.print(speed_act);         Serial.print("  PWM:");  Serial.print(PWM_val);   Serial.print(" > \n");          
 }
}



int updatePid(int command, double targetValue, double currentValue)   {             // compute PWM value
double pidTerm = 0.0;                                                            // PID correction
double error=0.0;                                                               
 error = (double) (fabs(targetValue) - fabs(currentValue)); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                          
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void checkA() {
  stateB = digitalRead(encoderB);
  stateA = digitalRead(encoderA);
  if (stateA != stateB) {
    counter++;
  }
  else {
    counter--;
  }
}

void checkB() {
  stateB = digitalRead(encoderB);
  stateA = digitalRead(encoderA);
  if (stateA == stateB) {
    counter++;
  }
  else {
    counter--;
  }
}
