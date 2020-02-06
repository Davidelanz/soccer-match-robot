/* Motor specs: 
  - VM: M+ (black)
  - GM: M- (red)
  - Vcc for Encoder (Hall sensor) (white)
  - GND for Encoder (yellow)
  - S1: Signal Encoder 1 (orange).
  - S2: Signal Encoder 2 (green)
  - Nominal Voltage: DC 9,0 V.
  - Transmission Ratio: 1/75.
  - Max speed: 11500 ± 10% rp 
*/

/* Here I'm trying to use the output of the encoder??*/


// ctrl + k + c/ctrl + k + u

// Arduino board pins
// Right motor
#define vel_pin_r 2 // Pin for vel_rocity control for motor r
#define enc1_r 22   // Ecoder A for motor r
#define enc2_r 23   // Ecoder B for motor r
#define in1_r 24    // Rotation orientation for motor r: component 1
#define in2_r 25    // Rotation orientation for motor r: component 2

// #define vel_pin_r 2 // Pin for vel_rocity control for motor r
// #define enc1_r 22   // Ecoder A for motor r
// #define enc2_r 23   // Ecoder B for motor r
// #define in1_r 24    // Rotation orientation for motor r: component 1
// #define in2_r 25    // Rotation orientation for motor r: component 2

// Left motor
#define vel_pin_l 3 // Pin for vel_rocity control for motor l
#define enc1_l 28   // Ecoder A for motor l
#define enc2_l 29   // Ecoder B for motor l
#define in1_l 30    // Rotation orientation for motor l: component 1
#define in2_l 31    // Rotation orientation for motor l: component 2

// #define vel_pin_l 3 // Pin for vel_rocity control for motor l
// #define enc1_l 28   // Ecoder A for motor l
// #define enc2_l 29   // Ecoder B for motor l
// #define in1_l 30    // Rotation orientation for motor l: component 1
// #define in2_l 31    // Rotation orientation for motor l: component 2

// Variables initialization
// Right motor
int state_enc1_r = 0;    // Econder A state for motor r
int state_enc2_r = 0;    // Econder B state for motor r
int counter_r = -1;      // ???
unsigned long t_start_r; // Time @ process started
int vel_r = 0;           // velocity variable
int sw_r = 0;            // ???
double actvel_r = 0;     // ???

// Left motor
int state_enc1_l = 0;    // Econder A state for motor r
int state_enc2_l = 0;    // Econder B state for motor r
int counter_l = -1;      // ???
unsigned long t_start_l; // Time @ process started
int vel_l = 0;           // velocity variable
int sw_l = 0;            // ???
double actvel_l = 0;     // ???

int ka = 0;

void setup()
{
  
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
  // Right motor
  pinMode(vel_pin_r, OUTPUT);
  pinMode(in1_r, OUTPUT);
  pinMode(in2_r, OUTPUT);
  pinMode(enc1_r, INPUT);
  pinMode(enc2_r, INPUT);
  // Left motor
  pinMode(vel_pin_l, OUTPUT);
  pinMode(in1_l, OUTPUT);
  pinMode(in2_l, OUTPUT);
  pinMode(enc1_l, INPUT);
  pinMode(enc2_l, INPUT);

  // Right motor
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc1_r), check_enc1_r, CHANGE);
  // (when enc1_r changes, launch check_enc1 as interrupt function)
  attachInterrupt(digitalPinToInterrupt(enc2_r), check_enc2_r, CHANGE);

  // Left motor
  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc1_r), check_enc1_l, CHANGE);
  // (when enc1_r changes, launch check_enc1 as interrupt function)
  attachInterrupt(digitalPinToInterrupt(enc2_r), check_enc2_l, CHANGE);

  // Initialize digital (HIGH/LOW) pin values for right motor
  digitalWrite(in1_r, LOW);
  digitalWrite(in2_r, HIGH);
  // (rotate counterclockwise)

  // Initialize digital (HIGH/LOW) pin values for left motor
  digitalWrite(in1_l, HIGH);
  digitalWrite(in2_l, LOW);
  // (rotate clockwise)

  // Initialize analog (e.g. 200) pin values
  analogWrite(vel_pin_r, 200);
  analogWrite(vel_pin_l, 200);

  // Initialize variables
  t_start_r = millis();

  t_start_l = millis();

  //   millis() returns the number of milliseconds since the Arduino
  //   board began running the current program.

  Serial.println("<Arduino is ready>");

  Serial.flush();
  //   flush() waits for the transmission of outgoing serial data to complete.
  //   (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
}

void loop()
{

  String serialResponse = "";

  while (ka > 10000) //Serial.available()
  {
    // serialResponse = Serial.readString();
    // vel_r = serialResponse.substring(1, 4).toInt();
    // vel_l = serialResponse.substring(1, 4).toInt();
    // sw_r = serialResponse.substring(5, 6).toInt();
    // sw_l = serialResponse.substring(5, 6).toInt();
    vel_r = 50;
    vel_r = 50;
    sw_r = 1;
    sw_l = 1;
    if (sw_r == 1)
    {
      digitalWrite(in1_r, !digitalRead(in1_r));
      digitalWrite(in2_r, !digitalRead(in2_r));
    }
    if (sw_l == 1)
    {
      digitalWrite(in1_l, !digitalRead(in1_l));
      digitalWrite(in2_l, !digitalRead(in2_l));
    }

    analogWrite(vel_pin_r, vel_r);
    analogWrite(vel_pin_l, vel_l);

    Serial.print("< Right motor counter is ");
    Serial.print(counter_r);
    Serial.print(" and its velocity is ");
    Serial.print(actvel_r);
    Serial.println(" >");

    Serial.print("< Left motor counter is ");
    Serial.print(counter_l);
    Serial.print(" and its velocity is ");
    Serial.print(actvel_l);
    Serial.println(" >");
  }

  Serial.println("Ka =");
  Serial.print(ka);
  ka++;
}

// Right encoders
void check_enc1_r()
{
  int count;
  state_enc2_r = digitalRead(enc2_r);
  state_enc1_r = digitalRead(enc1_r);
  if (state_enc1_r != state_enc2_r)
  {
    counter_r++;
    count = 100;
  }
  else
  {
    counter_r--;
    count = -100;
  }
  if (counter_r % 100 == 0)
  {
    unsigned long t_fine = millis();
    unsigned long delta = t_fine - t_start_r;
    actvel_r = count / double(delta);
    t_start_r = millis();
  }
}
/*
void check_enc2_r()
{
  int count;
  state_enc2_r = digitalRead(enc2_r);
  state_enc1_r = digitalRead(enc1_r);
  if (state_enc1_r == state_enc2_r)
  {
    counter_r++;
    count = 100;
  }
  else
  {
    counter_r--;
    count = -100;
  }
  if (counter_r % 100 == 0)
  {
    unsigned long t_fine = millis();
    unsigned long delta = t_fine - t_start_r;
    actvel_r = count / double(delta);
    t_start_r = millis();
  }
}
*/
// Left encoders
void check_enc1_l()
{
  int count;
  state_enc2_l = digitalRead(enc2_l);
  state_enc1_l = digitalRead(enc1_l);
  if (state_enc1_l != state_enc2_l)
  {
    counter_l++;
    count = 100;
  }
  else
  {
    counter_l--;
    count = -100;
  }
  if (counter_l % 100 == 0)
  {
    unsigned long t_fine = millis();
    unsigned long delta = t_fine - t_start_l;
    actvel_r = count / double(delta);
    t_start_l = millis();
  }
}
/*
void check_enc2_l()
{
  int count;
  state_enc2_l = digitalRead(enc2_l);
  state_enc1_l = digitalRead(enc1_l);
  if (state_enc1_l != state_enc2_l)
  {
    counter_l++;
    count = 100;
  }
  else
  {
    counter_l--;
    count = -100;
  }
  if (counter_l % 100 == 0)
  {
    unsigned long t_fine = millis();
    unsigned long delta = t_fine - t_start_l;
    actvel_r = count / double(delta);
    t_start_l = millis();
  }
}
*/