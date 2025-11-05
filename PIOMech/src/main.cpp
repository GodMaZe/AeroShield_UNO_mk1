#include <AeroShield.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

// ---------------------------------------------------------

const static int BUILT_IN_LED_PIN = 13; // PIN pre zabudovanu LED

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 50; // perioda vzorkovania v ms

// ---------------------------------------------------------

unsigned long time_curr, time_last, control_time;

// Initialize the serial comms, calibrate the aeroshield system, send the initial state of the system
SemaphoreHandle_t mut_comms, mut_calibration;

static bool ARDUINO_RESET = false;

QueueHandle_t AeroDataQueue;

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
    if(ARDUINO_RESET) return;
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
    Serial.println(data.control_time);
}

void TaskBlinkLED(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskReadData(void *pvParameters);

void setup()
{
    ARDUINO_RESET = true;

    Serial.begin(115200);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB
    }

    AeroDataQueue = xQueueCreate(1, sizeof(AeroData));

    mut_comms = xSemaphoreCreateBinary();
    mut_calibration = xSemaphoreCreateBinary();

    // xSemaphoreTake(mut_waitMatlab, 0);

    if (AeroDataQueue != NULL)
    {
        // Create a task to handle Serial writing
        xTaskCreate(TaskSerial, "SerialHandle", 100, NULL, 3, NULL);

        // Create a task to read the potentiometer
        xTaskCreate(TaskReadData, "ReadData", 100, NULL, 1, NULL);
    }
    
    // Create a task to blink the built-in LED (indicating MCU is running)
    xTaskCreate(TaskBlinkLED, "BlinkLED", 100, NULL, 0, NULL);
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
        else if(!Serial)
        {
            // Blink to indicate waiting for Serial connection
            digitalWrite(BUILT_IN_LED_PIN, HIGH);
            vTaskDelay(500 / portTICK_PERIOD_MS);
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

    AeroData data;

    xSemaphoreGive(mut_comms); // Allow data reading task to start
    xSemaphoreTake(mut_calibration, portMAX_DELAY); // Wait for calibration to complete

    ARDUINO_RESET = false;

    for (;;)
    {
        if(xQueueReceive(AeroDataQueue, &data, portMAX_DELAY) == pdTRUE && Serial)
        {
            writeSerialData(data);
        }
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

    AeroData data;

    data.potentiometer = AeroShield.referenceRead();
    data.potentiometer = 0.0;

    time_curr = micros();
    time_last = time_curr;
    long time_start = 0;

    vTaskDelay(T_sample / portTICK_PERIOD_MS);

    for (;;)
    {
        time_curr = micros();

        if(time_start == 0)
        {
            time_start = time_curr;
            time_last = time_curr;
        }
        
        data.potentiometer = AeroShield.referenceRead();

        AeroShield.actuatorWrite(data.potentiometer);
        data.output = AeroShield.sensorReadDegree();
        data.control = data.potentiometer;
        data.time = static_cast<double>(time_curr - time_start) / 1e6;
        data.dt = static_cast<double>(time_curr - time_last) / 1e6;
        data.control_time = static_cast<double>(control_time) / 1e6;
        
        time_last = time_curr;

        xQueueSend(AeroDataQueue, &data, 0);
        vTaskDelay(T_sample / portTICK_PERIOD_MS);
    }
}