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
SemaphoreHandle_t mut_comms, mut_calibration, mut_init;

static char buf[4];
QueueHandle_t AeroDataQueue;
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
        return 0;                             // Success
    }
    return 1; // No new data received
}

void TaskBlinkLED(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskReadData(void *pvParameters);

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB
    }

    Serial.flush();

    memset(buf, 0, 4 * sizeof(char)); // Initialize buffer to zero

    AeroDataQueue = xQueueCreate(5, sizeof(struct AeroData));

    mut_waitMatlab = xSemaphoreCreateBinary();
    mut_comms = xSemaphoreCreateBinary();
    mut_calibration = xSemaphoreCreateBinary();
    mut_init = xSemaphoreCreateBinary();

    // xSemaphoreTake(mut_waitMatlab, 0);

    if (AeroDataQueue != NULL)
    {
        // Create a task to handle Serial writing
        xTaskCreate(TaskSerial, "SerialHandle", 100, NULL, 3, NULL);

        // Create a task to read the potentiometer
        xTaskCreate(TaskReadData, "ReadData", 100, NULL, 1, NULL);
    }
    // Create a task to blink the built-in LED (indicating MCU is running)
    xTaskCreate(TaskBlinkLED, "BlinkLED", 20, NULL, 0, NULL);
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

// ---------------------------------------------------------

void TaskSerial(void *pvParameters)
{
    (void)pvParameters;

    while(Serial.available())
    {
        Serial.read(); // Clear any initial junk data
    }

    Serial.println("--- MCU starting ---");

    // while(Serial.available() < 4);

    xSemaphoreGive(mut_comms); // Allow data reading task to start
    xSemaphoreTake(mut_calibration, portMAX_DELAY); // Wait for calibration to complete
    xSemaphoreTake(mut_init, portMAX_DELAY); // Wait for initial data to be ready

    float receivedValue = 0.0f;
    AeroData data;

    for (;;)
    {
        xQueueReceive(AeroDataQueue, &data, portMAX_DELAY);

        writeSerialData(data);

        while (recvByte(receivedValue))
        {
            vTaskDelay(1/ portTICK_PERIOD_MS);
        }
        CONTROL_SIGNAL = receivedValue;
        CONTROL_OVERRIDE = true;
        control_time = micros();
        // Unlock waiting for matlab message
        xSemaphoreGive(mut_waitMatlab);        
    }
}

// ---------------------------------------------------------

void TaskReadData(void *pvParameters)
{
    xSemaphoreTake(mut_comms, portMAX_DELAY);
    (void)pvParameters;

    AeroShield.begin();

    AeroShield.calibrate();

    xSemaphoreGive(mut_calibration);

    float controlSignal = 0.0f, outputSignal = 0.0f;
    time_curr = micros();
    time_last = time_curr;

    AeroData data;

    data.potentiometer = AeroShield.referenceRead(); // Initial read to ground the floating pin value

    outputSignal = AeroShield.sensorReadDegree();
    controlSignal = 0.0f;
    data.time = static_cast<double>(time_curr) / 1e6;
    data.output = outputSignal;
    data.control = controlSignal;
    data.dt = static_cast<double>(time_curr - time_last) / 1e6;
    data.control_time = static_cast<double>(control_time) / 1e6;
    xQueueSend(AeroDataQueue, &data, 0);

    xSemaphoreGive(mut_init);

    time_curr = micros();
    time_last = time_curr;

    for (;;)
    {
        // Wait for communication from matlab
        xSemaphoreTake(mut_waitMatlab, portMAX_DELAY);
        time_curr = micros();
        
        data.potentiometer = AeroShield.referenceRead();
        controlSignal = CONTROL_OVERRIDE ? CONTROL_SIGNAL : data.potentiometer;

        AeroShield.actuatorWrite(controlSignal);
        outputSignal = AeroShield.sensorReadDegree();
        
        
        
        data.time = static_cast<double>(time_curr) / 1e6;
        data.output = outputSignal;
        data.control = controlSignal;
        data.dt = static_cast<double>(time_curr - time_last) / 1e6;
        data.control_time = static_cast<double>(control_time) / 1e6;
        
        time_last = time_curr;

        xQueueSend(AeroDataQueue, &data, 0);
        
    }
}