#include <AeroShield.h>
#include <arduino-timer.h>

struct __attribute__((packed)) AeroData
{
	unsigned long time;
	float output;
	float control;
	float potentiometer;
	unsigned long control_time;
	unsigned long dt;
};

union AeroDataUnion
{
	AeroData data;
	uint8_t bytes[sizeof(AeroData)];
};

// ---------------------------------------------------------

constexpr static int BUILT_IN_LED_PIN = 13; // PIN pre zabudovanu LED

// --------------------------------------------------------- BUILD-IN LED BLINK
const unsigned long T_sample = 2000; // perioda vzorkovania v ms

unsigned long time_curr, time_last, control_time;

// Initialize the AeroData object
AeroDataUnion AeroDataInstance;

// Blink the built-in LED timer
auto ledTimer = timer_create_default();

void writeSerialData(const AeroDataUnion &data)
{
	Serial.write(data.bytes, sizeof(AeroDataUnion));
}

int recvByte(float &output)
{
	if (Serial.available() >= 4)
	{
		Serial.readBytes((uint8_t *)&output, 4); // Read 4 bytes from Serial
		return 1;								 // Success
	}
	return 0; // No new data received
}

void DoMeasurement()
{
	AeroShield.actuatorWrite(AeroDataInstance.data.control);
	AeroDataInstance.data.control_time = micros();
	AeroDataInstance.data.output = AeroShield.sensorReadDegree();
	AeroDataInstance.data.potentiometer = AeroShield.referenceRead();
	AeroDataInstance.data.dt = AeroDataInstance.data.time - time_last;
	writeSerialData(AeroDataInstance);
	time_last = AeroDataInstance.data.time;
}

bool TaskBlinkLED(void *);

void setup()
{
	pinMode(BUILT_IN_LED_PIN, OUTPUT); // for LED

	AeroShield.begin();
	AeroShield.calibrate();
	AeroShield.referenceRead();

	Serial.begin(250000);

	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB
	}
	Serial.flush();
	while (Serial.available())
	{
		Serial.read(); // Clear any initial junk data
	}

	Serial.println("--- MCU starting ---");

	AeroDataInstance.data.time = micros();
	time_last = AeroDataInstance.data.time;
	DoMeasurement();

	// Start the LED blink timer
	ledTimer.every(T_sample, TaskBlinkLED);
}

void loop()
{
	AeroDataInstance.data.time = micros();

	if (recvByte(AeroDataInstance.data.control))
	{
		DoMeasurement();
	}

	// Tick the LED blink timer
	ledTimer.tick();
}

// ---------------------------------------------------------

bool TaskBlinkLED(void *)
{
	static bool ledState = false;
	digitalWrite(BUILT_IN_LED_PIN, ledState ? HIGH : LOW);
	ledState = !ledState;
	return true;
}