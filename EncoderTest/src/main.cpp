#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// MAX 1200RPM, if RPM > 1200, arduino slows down

#define IRC0AM 2 // A-Channel ENCODER //2, 3
#define IRC0BM 3 // B-Channel ENCODER //3, 4

#define IRC1AM 20 // A-Channel ENCODER //18, 19
#define IRC1BM 21 // B-Channel ENCODER //19, 20

#define IRC1IP 6 // Interrupt Pin for IRC1

#define T0OUT 8 // Timer 0 Output Pin
#define T1OUT 9 // Timer 1 Output Pin

#define CIN A0 // Control voltage for TOUT pins

Encoder encoder0(IRC0AM, IRC0BM); //
Encoder encoder1(IRC1AM, IRC1BM); // 616, -838

float speed2rpm(const float &speed);
void readIRC(Encoder &encoder, const unsigned long &current_time, const unsigned long &last_time, long &position, float &speed);

void setup()
{
  pinMode(IRC0AM, INPUT_PULLUP); // Set A-Channel ENCODER pin as input with pull-up resistor
  pinMode(IRC0BM, INPUT_PULLUP); // Set B-Channel ENCODER pin as input with pull-up resistor
  pinMode(IRC1AM, INPUT_PULLUP); // Set A-Channel ENCODER pin as input with pull-up resistor
  pinMode(IRC1BM, INPUT_PULLUP); // Set B-Channel ENCODER pin as input with pull-up resistor
  pinMode(IRC1IP, INPUT);        // Set interrupt pin for IRC1

  pinMode(T0OUT, OUTPUT); // Set Timer 0 Output Pin as output
  pinMode(T1OUT, OUTPUT); // Set Timer 1 Output Pin as output

  pinMode(CIN, INPUT); // Set Control voltage pin as input

  Serial.begin(115200);
  Serial.println("--- MCU started ---");
}

unsigned long current_time = 0;

void loop()
{
  static long position0 = 0, position1 = 0;
  static float speed0 = 0.0f, speed1 = 0.0f;
  static unsigned long last_time = 0; // Initialize timer

  static int ircip = 0;
  static int cout = 0;

  ircip = analogRead(IRC1IP); // Read the interrupt pin state

  cout = analogRead(CIN);            // Read the control voltage pin state
  cout = map(cout, 0, 1023, 0, 255); // Map the value to 8 bits
  cout = constrain(cout, 0, 255);    // Constrain the value to 0-255

  analogWrite(T0OUT, cout); // Write the control voltage to Timer 0 Output Pin
  analogWrite(T1OUT, cout); // Write the control voltage to Timer 1 Output

  current_time = micros();

  // if (last_time + 10e3 < current_time) // Print every second
  // {

  readIRC(encoder0, current_time, last_time, position0, speed0);
  readIRC(encoder1, current_time, last_time, position1, speed1);

  // Serial.print(current_time);
  // Serial.print(" ");
  Serial.print(position0);
  // Serial.print(" ");
  // Serial.print(speed2rpm(speed0)); // Print the RPM

  Serial.print(" ");
  Serial.print(position1);
  Serial.print(" ");
  // Serial.print(speed2rpm(speed1)); // Print the RPM
  // Serial.print(" ");
  // Serial.print(ircip); // Print the interrupt pin state

  // Serial.print(" ");
  Serial.print(cout); // Print the control voltage pin state
  Serial.print(" ");
  Serial.println((current_time - last_time) / 1000.0f, 5); // Print the time difference in milliseconds
  last_time = current_time;
  // }
}

void readIRC(Encoder &encoder, const unsigned long &current_time, const unsigned long &last_time, long &position, float &speed)
{
  long last_position = position;
  position = encoder.read();
  speed = (float)(position - last_position) / (current_time - last_time); // steps per minute
}

float speed2rpm(const float &speed)
{
  return speed / 6e7f; // Convert to RPM
}
