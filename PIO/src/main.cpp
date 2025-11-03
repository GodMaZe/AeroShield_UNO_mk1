#include <AeroShield.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

// ---------------------------------------------------------

const static int BUILT_IN_LED_PIN = 13;   // PIN pre zabudovanu LED

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 50; // perioda vzorkovania v ms

// ---------------------------------------------------------
float OUTPUT_SIGNAL = 0.0f;    // uhol natocenia ramena (I2C bus)
float CONTROL_SIGNAL = 0.0f;   // Potentiometer value (0.0 - 100.0 %)
bool CONTROL_OVERRIDE = false; // flag pre manualne riadenie

unsigned long time_curr, time_last;

bool IS_INITIALIZED = false;

static char buf[4];
QueueHandle_t AeroDataQueue;

struct AeroData
{
    unsigned long time;
    float output;
    float control;
    float dt;
};

void writeSerialData(const AeroData &data)
{
    Serial.print(data.time);
    Serial.print(" ");
    Serial.print(data.output);
    Serial.print(" ");
    Serial.print(data.control);
    Serial.print(" ");
    Serial.print(data.dt);
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
void TaskSerialWrite(void *pvParameters);
void TaskSerialRead(void *pvParameters);
void TaskReadData(void *pvParameters);


void setup()
{
    Serial.begin(115200);

    memset(buf, 0, 4 * sizeof(char)); // Initialize buffer to zero

    AeroDataQueue = xQueueCreate(5, sizeof(struct AeroData));

    if(AeroDataQueue != NULL)
    {
        Serial.println("AeroDataQueue created successfully.");
        // Create a task to handle Serial writing
        xTaskCreate(TaskSerialWrite, "SerialWrite", 128, NULL, 2, NULL);

        Serial.println("Serial Write Task created.");
        // Create a task to handle Serial reading
        xTaskCreate(TaskSerialRead, "SerialRead", 128, NULL, 1, NULL);

        Serial.println("Serial Read Task created.");
        // Create a task to read the potentiometer
        xTaskCreate(TaskReadData, "ReadData", 128, NULL, 1, NULL);

        Serial.println("Read Data Task created.");
    }
    else{
        Serial.println("Failed to create AeroDataQueue.");
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
        else{
            digitalWrite(BUILT_IN_LED_PIN, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            digitalWrite(BUILT_IN_LED_PIN, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void TaskSerialWrite(void *pvParameters)
{
    (void)pvParameters;

    Serial.println("Waiting for Serial connection...");

    // while(!Serial)
    // {
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    IS_INITIALIZED = true;

    Serial.println("--- MCU starting ---");

    AeroData data;

    for (;;)
    {
        if (xQueueReceive(AeroDataQueue, &data, portMAX_DELAY) == pdPASS)
        {
            writeSerialData(data);
        }
    }
}

void TaskSerialRead(void *pvParameters)
{
    (void)pvParameters;

    float receivedValue = 0.0f;

    Serial.println("Serial Read Task started.");

    for (;;)
    {
        if (!recvByte(receivedValue) && IS_INITIALIZED)
        {
            if(receivedValue < 0)
            {
                CONTROL_OVERRIDE = false;
            }
            else
            {
                CONTROL_SIGNAL = receivedValue;
                CONTROL_OVERRIDE = true;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TaskReadData(void *pvParameters)
{
    (void)pvParameters;

    Serial.println("Task Read Data started.");

    AeroShield.begin();
    AeroShield.calibrate();

    time_curr = micros();
    time_last = time_curr;
    vTaskDelay(T_sample / portTICK_PERIOD_MS);

    AeroData data;
    float controlSignal = 0.0f, outputSignal = 0.0f;

    for (;;)
    {
        if (IS_INITIALIZED)
        {
            controlSignal = CONTROL_OVERRIDE ? CONTROL_SIGNAL : AeroShield.referenceRead()/1023.0f*100.0f;
            outputSignal = AeroShield.sensorReadDegree();

            time_curr = micros();
            
            data.time = time_curr;
            data.output = outputSignal;
            data.control = controlSignal;
            data.dt = (float)(time_curr - time_last) / 1000000.0f;
            time_last = time_curr;

            xQueueSend(AeroDataQueue, &data, 0);

            AeroShield.actuatorWrite(controlSignal);
        }
        vTaskDelay(T_sample / portTICK_PERIOD_MS);
    }
}