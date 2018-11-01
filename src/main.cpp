#include <Arduino.h>

const uint8_t latchPin = 26;	// Pin connected to ST_CP of 74HC595
const uint8_t clockPin = 25;	// Pin connected to SH_CP of 74HC595
const uint8_t dataPin = 27;		// Pin connected to DS of 74HC595
const uint8_t reedRelay = 14;	// Magnetic relay for angular speed calculation

typedef uint32_t matrix_t;
const int ledCount = 32;		// Changing this requires editing matrix_t types accordingly
const float ledOffset = 0.35f;	// Distance from rotation point to closest led (cm)
const float ledStride = 0.3f;	// Distance between each led (cm)

int lastRelayState = LOW;
unsigned long lastRelayHighMicro = 0;
const unsigned long relayBounceSleep = 10 * 1000UL;

float angularPosition = 0;
float angularVelocity = TWO_PI / 1000000UL; // Default to 360 degree per second

unsigned long lastMicro = 0;	

matrix_t ledMatrix = 0;	// Led states

typedef struct
{
	float x, y;	// Centimeters (cm)
} Vector2;

inline Vector2 rotateVector(Vector2 v, float angle)
{
	return {
		(float)(cos(angle) * v.x - sin(angle) * v.y),
		(float)(sin(angle) * v.x + cos(angle) * v.y)
	};
}

inline void writeRegister(matrix_t data)
{
	static matrix_t registerStates = 0;
	// Don't shift if register state is unchanged
	if (registerStates != data)
	{
		digitalWrite(latchPin, LOW);
		for (int i = sizeof(data); i >= 0; i--)
			shiftOut(dataPin, clockPin, MSBFIRST, (uint8_t)(data >> (i * 8) & 0xFF));
		digitalWrite(latchPin, HIGH);
		registerStates = data;
	}
}

/// Turns leds on/off according to the current render state
inline void updateLeds()
{
	writeRegister(ledMatrix);
}

/// Converts angular coordinates (α, L) into cartesian coordinates (x, y).
/// Angles in radian, ledIndex -> 0 is closest to rotation axis.
inline Vector2 getLedCoord(float angle, int ledIndex)
{
	// HACK: Implementation is design dependent
	if (ledIndex < 16)
	{
		return {
			(float)(cos(angle) * (ledOffset + ledIndex * ledStride)),
			(float)(sin(angle) * (ledOffset + ledIndex * ledStride))
		};
	}
	else
	{
		angle += 180;
		return {
			(float)(cos(angle) * (ledOffset + ledIndex * ledStride)),
			(float)(sin(angle) * (ledOffset + ledIndex * ledStride))
		};
	}
}

/// Returns whether given point is inside the triangle using efficient Barycentric method.
bool isPointInTriangle(Vector2 p, Vector2 tri[3])
{
	float s = tri[0].y * tri[2].x - tri[0].x * tri[2].y + (tri[2].y - tri[0].y) * p.x + (tri[0].x - tri[2].x) * p.y;
	float t = tri[0].x * tri[1].y - tri[0].y * tri[1].x + (tri[0].y - tri[1].y) * p.x + (tri[1].x - tri[0].x) * p.y;
	
	if ((s < 0) != (t < 0))
		return false;
	
	float A = -tri[1].y * tri[2].x + tri[0].y * (tri[2].x - tri[1].x) + tri[0].x * (tri[1].y - tri[2].y) + tri[1].x * tri[2].y;
	
	if (A < 0)
	{
		s = -s;
		t = -t;
		A = -A;
	}
	return s > 0 && t > 0 && (s + t) <= A;
}

/// Calculates difference by accounting for integer overflows. Usefull for time calculations.
inline unsigned long wrapDiff(unsigned long big, unsigned long small)
{
	return (big >= small) ? (big - small) : ((~small) + big);
}

Vector2 triangle[3];

void setup()
{
	pinMode(reedRelay, INPUT);
	pinMode(latchPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(dataPin, OUTPUT);
	
	// Create a proper triangle
	Vector2 v = { 0.0f, ledOffset + ledStride * (ledCount - 1) - ledStride / 2 };
	triangle[0] = v;
	triangle[1] = rotateVector(v, 120 * DEG_TO_RAD);
	triangle[2] = rotateVector(v, 240 * DEG_TO_RAD);
}

void loop()
{
	unsigned long rightNow = micros();
	unsigned long microDiff = wrapDiff(rightNow, lastMicro);	// Amount of time passed in microseconds since last loop()
	unsigned long relayMicroDiff = wrapDiff(rightNow, lastRelayHighMicro);	// Amount of time passed in microseconds since last relay pin rising edge
	
	bool turn = false;	// Determines whether we are on relay rising edge (full turn)
	
	// Wait small fraction of time to avoid unstable electrical bounce
	if (relayMicroDiff > relayBounceSleep)
	{
		int relayState = digitalRead(reedRelay);
		if (lastRelayState != relayState)
		{
			if (relayState == HIGH)	// Rising edge
			{
				angularPosition = 0;
				angularVelocity = TWO_PI / relayMicroDiff;
				lastRelayHighMicro = rightNow;
				turn = true;
			}
			lastRelayState = relayState;
		}
	}
	if (!turn)
		angularPosition = relayMicroDiff * angularVelocity;
	
	// Periodic modulo for angle (360 degree)
	if (angularPosition > TWO_PI)
		angularPosition = fmodf(angularPosition, TWO_PI);

	// Calculate and write led states to ledMatrix
	ledMatrix = 0;
	matrix_t bitMask = 1;
	for (int i = 0; i < ledCount; i++)
	{
		if (isPointInTriangle(getLedCoord(angularPosition, i), triangle))
			ledMatrix |= bitMask;
		bitMask = bitMask << 1;
	}
	
	updateLeds();
	lastMicro = rightNow;
}
