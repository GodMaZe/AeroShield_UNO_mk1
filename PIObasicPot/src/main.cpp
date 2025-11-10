#include <AeroShield.h>

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 1000;
const unsigned long LED_onTime = 100;

// ---------------------------------------------------------
float REFERENCE_SIGNAL = 0.0f; // potenciometer
float OUTPUT_SIGNAL = 0.0f;    // uhol natocenia ramena (I2C bus)
float CONTROL_SIGNAL = 0.0f;   // PWM pre motor (matlab -> SPI bus)

unsigned long time_curr, time_tick = 0, time_delta, time_curr_data, time_dt = 0, time_last_data = 0;

bool LED_on = false;

void sendData()
{
    time_dt = time_curr_data - time_last_data;
    time_last_data = time_curr_data;

    Serial.print(static_cast<double>(time_curr_data) / 1e6); // Convert microseconds to seconds
    Serial.print(" ");
    Serial.print(REFERENCE_SIGNAL);
    Serial.print(" ");
    Serial.print(OUTPUT_SIGNAL);
    Serial.print(" ");
    Serial.print(CONTROL_SIGNAL);
    Serial.print(" ");
    Serial.print(static_cast<double>(time_dt) / 1e6); // Convert microseconds to seconds
    Serial.print("\n");
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // for LED

    AeroShield.begin();
    AeroShield.calibrate();

    // Initialize Serial communication
    // Set the baud rate to 9600 for communication with the Serial Monitor
    Serial.begin(9600);
    Serial.println("--- MCU starting ---");
    time_curr_data = micros();

    REFERENCE_SIGNAL = AeroShield.referenceRead(); // Read the potentiometer value
    CONTROL_SIGNAL = REFERENCE_SIGNAL;
    OUTPUT_SIGNAL = AeroShield.sensorReadDegree(); // Read the potentiometer and sensor values

    sendData();
}

bool recvFloat(float &output)
{
    if (Serial.available() >= 4)
    {
        // Discard the incoming bytes, used solely for synchronization
        while(Serial.available()) Serial.read(); // Clear the buffer

        // Read the 4 bytes from Serial and reconstruct the float
        // char buf[4];
        // for (int i = 0; i < 4; i++)
        // {
        //     buf[i] = Serial.read();
        // }
        // memcpy(&output, buf, sizeof(float));
        return true;
    }
    return false;
}


// ---------------------------------------------------------

void buildInLedBlink()
{
    time_delta = (time_curr_data - time_tick)/1000;

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
        time_tick = time_curr_data;

        digitalWrite(LED_BUILTIN, HIGH);
        LED_on = true;
    }
}

// ---------------------------------------------------------

void loop()
{
    time_curr_data = micros();

    REFERENCE_SIGNAL = AeroShield.referenceRead(); // Read the potentiometer value
    CONTROL_SIGNAL = REFERENCE_SIGNAL;
    AeroShield.actuatorWrite(CONTROL_SIGNAL); // Write the control signal to the actuator
    OUTPUT_SIGNAL = AeroShield.sensorReadDegree();

    if (recvFloat(CONTROL_SIGNAL))
    {
        sendData();
    }

    buildInLedBlink();

    // delay(2);
}