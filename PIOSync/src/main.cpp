#include <AeroShield.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

// ---------------------------------------------------------

const static int BUILT_IN_LED_PIN = 13; // PIN pre zabudovanu LED

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 50; // perioda vzorkovania v ms

// ---------------------------------------------------------
float OUTPUT_SIGNAL = 0.0f;    // uhol natocenia ramena (I2C bus)
float CONTROL_SIGNAL = 0.0f;   // Potentiometer value (0.0 - 100.0 %)
bool CONTROL_OVERRIDE = false; // flag pre manualne riadenie

unsigned long time_curr, time_last, control_time;

// Initialize the serial comms, calibrate the aeroshield system, send the initial state of the system
bool IS_INITIALIZED = false, IS_CALIBRATED = false, IS_SETUP = false;

static char buf[4];
QueueHandle_t AeroDataQueue;
SemaphoreHandle_t mut_serial;
SemaphoreHandle_t mut_waitMatlab;

struct AeroData
{
    double time;
    float output;
    float control;
    float potentiometer;
    double control_time;
    double dt;
};

void writeSerialData(const AeroData &data)
{
    Serial.print(data.time);
    Serial.print(" ");
    Serial.print(data.output);
    Serial.print(" ");
    Serial.print(data.control);
    Serial.print(" ");
    Serial.print(data.potentiometer);
    Serial.print(" ");
    Serial.print(data.dt);
    Serial.print(" ");
    Serial.print(data.control_time);
    Serial.print("\n");
}

int recvByte(float &output)
{

    if (Serial.available() >= 4)
    {
        Serial.readBytes(buf, 4);             // Read 4 bytes from Serial
        memcpy(&output, &buf, sizeof(float)); // Copy the float value to CONTROL_SIGNAL
        return 1;                             // Success
    }
    return 0; // No new data received
}

void TaskBlinkLED(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskReadData(void *pvParameters);

void setup()
{
    Serial.begin(115200);

    memset(buf, 0, 4 * sizeof(char)); // Initialize buffer to zero

    AeroDataQueue = xQueueCreate(5, sizeof(struct AeroData));

    mut_waitMatlab = xSemaphoreCreateBinary();
    xSemaphoreTake(mut_waitMatlab, 0);

    if (AeroDataQueue != NULL)
    {
        // Serial.println("AeroDataQueue created successfully.");
        // Create a task to handle Serial writing
        xTaskCreate(TaskSerial, "SerialHandle", 128, NULL, 3, NULL);

        // Create a task to read the potentiometer
        xTaskCreate(TaskReadData, "ReadData", 128, NULL, 2, NULL);
    }
    // Create a task to blink the built-in LED (indicating MCU is running)
    xTaskCreate(TaskBlinkLED, "BlinkLED", 128, NULL, 0, NULL);
}

void loop()
{
}

// ---------------------------------------------------------

void TaskBlinkLED(void *pvParameters)
{
    (void)pvParameters;

    pinMode(BUILT_IN_LED_PIN, OUTPUT); // for LED

    for (;;)
    {
        if (AeroDataQueue == NULL)
        {
            // Blink rapidly to indicate error
            digitalWrite(BUILT_IN_LED_PIN, HIGH);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            digitalWrite(BUILT_IN_LED_PIN, LOW);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else
        {
            digitalWrite(BUILT_IN_LED_PIN, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            digitalWrite(BUILT_IN_LED_PIN, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void TaskSerial(void *pvParameters)
{
    (void)pvParameters;

    while (!Serial)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    float receivedValue = 0.0f;
    AeroData data;

    while (!IS_CALIBRATED)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while(Serial.available())
    {
        Serial.read(); // Clear any initial junk data
    }

    Serial.println("--- MCU starting ---");

    // while(Serial.available() < 4);

    IS_INITIALIZED = true;

    while(!IS_SETUP){
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while(xQueueReceive(AeroDataQueue, &data, portMAX_DELAY) != pdPASS){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    writeSerialData(data);

    for (;;)
    {
        while(Serial.available() < 4);
        if (recvByte(receivedValue))
        {
            if (receivedValue < 0)
            {
                CONTROL_OVERRIDE = false;
            }
            else
            {
                CONTROL_SIGNAL = receivedValue;
                CONTROL_OVERRIDE = true;
            }
            control_time = micros();
        }
        // Unlock waiting for matlab message
        xSemaphoreGive(mut_waitMatlab);

        if (xQueueReceive(AeroDataQueue, &data, portMAX_DELAY) == pdPASS && Serial)
        {
            writeSerialData(data);
        }
    }
}

void TaskReadData(void *pvParameters)
{
    (void)pvParameters;

    AeroShield.begin();

    while (!AeroShield.calibrate())
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    IS_CALIBRATED = true;

    while (!IS_INITIALIZED)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    float controlSignal = 0.0f, outputSignal = 0.0f;
    time_curr = micros();
    time_last = time_curr;
    vTaskDelay(T_sample / portTICK_PERIOD_MS);

    AeroData data;

    data.potentiometer = AeroShield.referenceRead(); // Initial read to ground the floating pin value

    outputSignal = AeroShield.sensorReadDegree();
    data.potentiometer = AeroShield.referenceRead();
    controlSignal = CONTROL_OVERRIDE ? CONTROL_SIGNAL : data.potentiometer;
    data.time = static_cast<double>(time_curr) / 1e6;
    data.output = outputSignal;
    data.control = controlSignal;
    data.dt = static_cast<double>(time_curr - time_last) / 1e6;
    data.control_time = static_cast<double>(control_time) / 1e6;
    time_last = time_curr;
    xQueueSend(AeroDataQueue, &data, 0);

    IS_SETUP = true;

    for (;;)
    {
        // Wait for communication from matlab
        xSemaphoreTake(mut_waitMatlab, portMAX_DELAY);

        AeroShield.actuatorWrite(controlSignal);
        outputSignal = AeroShield.sensorReadDegree();

        time_curr = micros();

        data.potentiometer = AeroShield.referenceRead();
        controlSignal = CONTROL_OVERRIDE ? CONTROL_SIGNAL : data.potentiometer;
        data.time = static_cast<double>(time_curr) / 1e6;
        data.output = outputSignal;
        data.control = controlSignal;
        data.dt = static_cast<double>(time_curr - time_last) / 1e6;
        data.control_time = static_cast<double>(control_time) / 1e6;
        time_last = time_curr;

        xQueueSend(AeroDataQueue, &data, 0);
    }
}