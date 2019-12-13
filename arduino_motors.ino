// Arduino board pins
// - Right motor
#define vel_pin_r  2   // Pin for velocity control for motor r
#define enc1_r    22   // Ecoder A for motor r
#define enc2_r    23   // Ecoder B for motor r
#define in1_r     24   // Rotation orientation for motor r: component 1
#define in2_r     25   // Rotation orientation for motor r: component 2


// Variables initialization
int state_enc1_r = 0;  // Econder A state for motor r
int state_enc2_r = 0;  // Econder B state for motor r
int counter = -1;      // ???
unsigned long t_start; // Time @ process started
int vel=0;             // Velocity variable
int sw=0;              // ???
double actvel=0;       // ???


void setup() {
  
  // Set up Serial library at 115200 bps
  Serial.begin(115200);
  /*   communication from Arduino board and PC.
    All Arduinos have at least one serial port: Serial.
    It communicates on digital pins 0 (RX) and 1 (TX) as well as with the computer via USB. 
    Thus, if you use these functions, you cannot also use pins 0 and 1 for digital input or output.
    You can use the Arduino environment's built-in serial monitor to communicate with an Arduino board. 
    Click the serial monitor button in the toolbar and select the same 
    baud rate used in the call to begin().
  */

  // Pin mapping 
  pinMode(vel_pin_r, OUTPUT);    
  pinMode(in1_r,     OUTPUT);
  pinMode(in2_r,     OUTPUT);
  pinMode(enc1_r,    INPUT);
  pinMode(enc2_r,    INPUT);

  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc1_r), check_enc1, CHANGE);
  // (when enc1_r changes, launch check_enc1 as interrupt function)
  attachInterrupt(digitalPinToInterrupt(enc2_r), check_enc2, CHANGE);

  // Initialize digital (HIGH/LOW) pin values
  digitalWrite(in1_r, LOW);
  digitalWrite(in2_r, HIGH);
  // (rotate counterclockwise)
  
  // Initialize analog (e.g. 200) pin values
  analogWrite(vel_pin_r, 200);
    
  // Initialize variables
  t_start = millis(); 
  //   millis() returns the number of milliseconds since the Arduino
  //   board began running the current program.

  Serial.println("<Arduino is ready>");
  
  Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.
  //   (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
}

void loop() {
  
  String serialResponse ="";

  while (Serial.available()) {
    serialResponse  = Serial.readString();
    vel = serialResponse.substring(1,4).toInt();
    sw = serialResponse.substring(5,6).toInt();
    if(sw==1){
    digitalWrite(in1_r, !digitalRead(in1_r));
    digitalWrite(in2_r, !digitalRead(in2_r));
    
    }
    analogWrite(vel_pin_r, vel);
    Serial.print("< Counter is ");
    Serial.print(counter);
    Serial.print(" and velocity is ");
    Serial.print(actvel);
    Serial.println(" >");
  }
}

void check_enc1() {
int count;
  state_enc2_r = digitalRead(enc2_r);
  state_enc1_r = digitalRead(enc1_r);
  if (state_enc1_r != state_enc2_r) {
    counter++;
    count=100;
  }
  else {
    counter--;
    count=-100;
  }
  if (counter%100==0){
  unsigned long t_fine = millis();
  unsigned long delta = t_fine - t_start;
  actvel = count / double(delta);
  t_start = millis();
  }
}

void check_enc2() {
  int count;
  state_enc2_r = digitalRead(enc2_r);
  state_enc1_r = digitalRead(enc1_r);
  if (state_enc1_r == state_enc2_r) {
    counter++;
    count=100;
  }
  else {
    counter--;
    count=-100;
  }
  if (counter%100==0){
  unsigned long t_fine = millis();
  unsigned long delta = t_fine - t_start;
  actvel = count / double(delta);
  t_start = millis();
  }
}
