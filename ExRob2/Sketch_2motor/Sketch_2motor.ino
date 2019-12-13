// Definitions Arduino pins connected to input H Bridge

#define R_arduino_in1  22
#define R_arduino_in2  24                    
#define L_arduino_in3  26
#define L_arduino_in4  28

void setup()
{
    // set up Serial library at 115200 bps
    Serial.begin(115200); 
    // Set the output pins 
    // MOTOR R
    pinMode(R_arduino_in1, OUTPUT);
    pinMode(R_arduino_in2, OUTPUT);
    // MOTOR L
    pinMode(L_arduino_in3, OUTPUT);
    pinMode(L_arduino_in4, OUTPUT);
}    
    
void loop()
{
    // Rotate the Motor R clockwise
    digitalWrite(R_arduino_in1, HIGH);
    digitalWrite(R_arduino_in2, LOW);
    delay(2000);
    // Stop Motor R
    digitalWrite(R_arduino_in1, HIGH);
    digitalWrite(R_arduino_in2, HIGH);
    delay(500);
    // Rotate the Motor L clockwise
    digitalWrite(L_arduino_in3, HIGH);
    digitalWrite(L_arduino_in4, LOW);
    delay(2000);
    // Stop Motor L
    digitalWrite(L_arduino_in3, HIGH);
    digitalWrite(L_arduino_in4, HIGH);
    delay(500);
}
