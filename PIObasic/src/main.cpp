#include <AeroShield.h>

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 1000;
const unsigned long LED_onTime = 100;

// --------------------------------------------------------- CALIBRATION DATA
static float PHI_INIT = 0.0f; // uhol natocenia ramena pri inicializacii

// ---------------------------------------------------------
float REFERENCE_SIGNAL = 0.0f; // potenciometer
float OUTPUT_SIGNAL = 0.0f;    // uhol natocenia ramena (I2C bus)
float CONTROL_SIGNAL = 0.0f;   // PWM pre motor (matlab -> SPI bus)

unsigned long time_curr, time_tick = 0, time_delta, time_curr_data, time_dt = 0, time_last_data = 0;

bool LED_on = false;

static char buf[4];

void sendData()
{
    time_dt = time_curr_data - time_last_data;
    time_last_data = time_curr_data;

    Serial.print(time_curr_data);
    Serial.print(" ");
    Serial.print(REFERENCE_SIGNAL);
    Serial.print(" ");
    Serial.print(OUTPUT_SIGNAL);
    Serial.print(" ");
    Serial.print(CONTROL_SIGNAL);
    Serial.print(" ");
    Serial.print(time_dt);
    Serial.print("\n");
}

void readData()
{
    // Convert the received character to an integer value (potentiometer value)
    REFERENCE_SIGNAL = AeroShield.referenceRead(); // Read the potentiometer value
    OUTPUT_SIGNAL = AeroShield.sensorReadDegree();
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // for LED

    AeroShield.begin();
    AeroShield.calibrate();

    memset(buf, 0, 4 * sizeof(char)); // Initialize buffer to zero

    // Initialize Serial communication
    // Set the baud rate to 115200 for communication with the Serial Monitor
    Serial.begin(115200);
    Serial.println("--- MCU starting ---");
    time_curr_data = micros();

    readData(); // Read the potentiometer and sensor values

    sendData();
}

// ---------------------------------------------------------

int recvByte(float &output)
{

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

    time_curr_data = micros();

    readData(); // Read the potentiometer and sensor values

    if (recvByte(CONTROL_SIGNAL))
    {
        // No new data received
        return;
    }
    sendData();

    AeroShield.actuatorWrite(CONTROL_SIGNAL); // Write the control signal to the actuator
}

// ---------------------------------------------------------

void buildInLedBlink()
{
    time_delta = time_curr - time_tick;

    if (LED_on == true)
    {
        if (time_delta >= LED_onTime)
        {
            digitalWrite(LED_BUILTIN, LOW);
            LED_on = false;
        }
    }

    if (time_delta >= T_sample)
    {
        time_tick = time_curr;

        digitalWrite(LED_BUILTIN, HIGH);
        LED_on = true;
    }
}

// ---------------------------------------------------------

void loop()
{
    // time_curr = micros();

    processNewData();

    buildInLedBlink();

    // delay(2);
}