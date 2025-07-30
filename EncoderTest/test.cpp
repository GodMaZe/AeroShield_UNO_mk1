#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// MAX 1200RPM, if RPM > 1200, arduino slows down

#define A 2 // A-Channel ENCODER //2, 3
#define B 3 // B-Channel ENCODER //3, 4

Encoder encoder(A, B);

void setup()
{
  Serial.begin(115200);
  Serial.println("--- MCU started ---");
}

unsigned long current_time = 0;

void loop()
{
  static long position = 0;
  static float ot_pos = 0.0f, ot_last_pos = 0.0f;
  static float speed = 0.0f;
  static unsigned long encodertimer = 999, dt = 0, lastprinttimer = micros(); // Initialize timer

  current_time = micros();

  // if (lastprinttimer + 10e3 < current_time) // Print every second
  // {
  ot_last_pos = ot_pos; // Convert to rotations
  position = encoder.read();
  ot_pos = position / 4096.0f; // Convert to rotations
  dt = current_time - lastprinttimer;
  encodertimer = current_time;
  speed = (float)(ot_pos - ot_last_pos) / (dt / 6e7f); // steps per minute
  lastprinttimer = current_time;
  Serial.print(current_time);
  Serial.print(" ");
  Serial.print(speed); // Print the RPM
  Serial.print(" ");
  Serial.print(ot_pos);
  Serial.print(" ");
  Serial.println(dt / 1000.0f, 5); // Print the time difference in milliseconds
  // }
}
