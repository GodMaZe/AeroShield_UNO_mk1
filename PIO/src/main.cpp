#include <AeroShield.h>

// ---------------------------------------------------------

const static int BUILT_IN_LED_PIN = 13; // PIN pre zabudovanu LED
// const static int CURRENT_SENSOR_PIN = A0; // PIN pre citanie prudu z motora

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 1000;
const unsigned long LED_onTime = 100;

// --------------------------------------------------------- CALIBRATION DATA
static float PHI_INIT = 0.0f; // uhol natocenia ramena pri inicializacii

// ---------------------------------------------------------
float REFERENCE_SIGNAL = 0.0f; // potenciometer
float OUTPUT_SIGNAL = 0.0f;    // uhol natocenia ramena (I2C bus)
float CONTROL_SIGNAL = 0.0f;   // PWM pre motor (matlab -> SPI bus)

unsigned long time_curr;
unsigned long time_tick = 0;
unsigned long time_delta;

unsigned long time_curr_data;

bool LED_on = false;

void setup()
{
    pinMode(BUILT_IN_LED_PIN, OUTPUT); // for LED
    // pinMode(CURRENT_SENSOR_PIN, INPUT_PULLUP); // for current sensor

    AeroShield.begin();
    AeroShield.calibrate();

    // Initialize Serial communication
    // Set the baud rate to 115200 for communication with the Serial Monitor
    Serial.begin(115200);
    Serial.println("--- MCU starting ---");
    time_curr_data = millis();
    Serial.print(time_curr_data);
    Serial.print(" ");
    Serial.print(REFERENCE_SIGNAL);
    Serial.print(" ");
    Serial.print(PHI_INIT);
    Serial.print(" ");
    Serial.print(CONTROL_SIGNAL);
    Serial.print(" ");
    Serial.print("\n");
}

// ---------------------------------------------------------

int recvByte(float &output)
{
    static char buf[4];
    memset(buf, 0, 4 * sizeof(char)); // Initialize buffer to zero
    if (Serial.available() >= 4)
    {
        Serial.readBytes(buf, 4);             // Read 4 bytes from Serial
        memcpy(&output, &buf, sizeof(float)); // Copy the float value to CONTROL_SIGNAL
        return 0;                             // Success
    }
    return 1; // No new data received
}

void processNewData()
{

    if (recvByte(CONTROL_SIGNAL))
    {
        // No new data received
        return;
    }

    time_curr_data = millis();

    // Convert the received character to an integer value (potentiometer value)
    REFERENCE_SIGNAL = AeroShield.referenceRead(); // Read the potentiometer value
    OUTPUT_SIGNAL = AeroShield.sensorReadDegree();

    AeroShield.actuatorWrite(CONTROL_SIGNAL); // Write the control signal to the actuator

    // Print the current time and the signals to the Serial Monitor
    Serial.print(time_curr_data);
    // Serial.print(analogRead(CURRENT_SENSOR_PIN) / 1023.0 * 5.0 * 0.185);
    Serial.print(" ");
    Serial.print(REFERENCE_SIGNAL); // Read the current sensor value
    Serial.print(" ");
    Serial.print(OUTPUT_SIGNAL);
    Serial.print(" ");
    Serial.print(CONTROL_SIGNAL);
    Serial.print(" ");
    Serial.print("\n");
}

// ---------------------------------------------------------

void buildInLedBlink()
{
    time_delta = time_curr - time_tick;

    if (LED_on == true)
    {
        if (time_delta >= LED_onTime)
        {
            digitalWrite(BUILT_IN_LED_PIN, LOW);
            LED_on = false;
        }
    }

    if (time_delta >= T_sample)
    {
        time_tick = time_curr;

        digitalWrite(BUILT_IN_LED_PIN, HIGH);
        LED_on = true;
    }
}

// ---------------------------------------------------------

void loop()
{
    time_curr = millis();

    processNewData();

    buildInLedBlink();

    delay(2);
}