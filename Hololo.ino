const byte latchPin = 3;	// Pin connected to ST_CP of 74HC595
const byte clockPin = 4;	// Pin connected to SH_CP of 74HC595
const byte dataPin = 2;		// Pin connected to DS of 74HC595
const byte reedRelay = 1;	// Magnetic relay for angular speed calculation

const byte ledCount = 16;		// Changing this requires editing uint16_t types to make sense
const float ledOffset = 1.0f;	// Distance from rotation point to closest led
const float ledStride = 0.3f;	// Distance between each leds

bool relayState = false;
bool lastRelayState = false;
unsigned long lastRelayHighMicro = 0;
unsigned long relayBounceSleep = 10 * 1000; // Microseconds

float angularVelocity = 0;
float angularPosition = 0;

uint16_t ledMatrix = 0;	// 4x4 Led states
byte rowIndex = 0;		// Index 0 is least significant 4 bit

typedef struct
{
	float x, y;	// Centimeters (cm)
} Vector2;

inline float deg2Rad(float deg)
{
	return deg * M_PI / 180;
}

inline float rad2Deg(float rad)
{
	return rad * 180 / M_PI;
}

inline Vector2 rotateVector(Vector2 v, float angle)
{
	return { cos(angle) * v.x - sin(angle) * v.y, sin(angle) * v.x + cos(angle) * v.y };
}

inline void writeRegister(byte data)
{
	digitalWrite(latchPin, LOW);
	shiftOut(dataPin, clockPin, MSBFIRST, data);
	digitalWrite(latchPin, HIGH);
}

/// Turns leds on/off according to the current render state
inline void updateLeds()
{
	byte data = (byte)(((ledMatrix >> (rowIndex * 4)) & 0x000F) << 4 | (~(B11110000 | (B0001 << rowIndex))));
	writeRegister(data);
}

/// Converts angular coordinates (α, L) into cartesian coordinates (x, y).
/// Angles in radian, ledIndex -> 0 is closest to rotation axis.
inline Vector2 getLedCoord(float angle, byte ledIndex)
{
	return { cos(angle) * (ledOffset + ledIndex * ledStride), sin(angle) * (ledOffset + ledIndex * ledStride) };
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
	Vector2 v = { 0.0f, ledOffset + ledStride * (ledCount - 0.5f) };
	triangle[0] = v;
	triangle[1] = rotateVector(v, deg2Rad(120));
	triangle[2] = rotateVector(v, deg2Rad(240));
}

unsigned long lastMicro = 0;

void loop()
{
	unsigned long rightNow = micros(),
		microDiff = wrapDiff(rightNow, lastMicro),
		relayMicroDiff = wrapDiff(rightNow, lastRelayHighMicro);
	
	if (relayMicroDiff > relayBounceSleep && lastRelayHighMicro != 0 && digitalRead(reedRelay) == HIGH)
	{
		angularPosition = 0;
		angularVelocity = (M_PI * 2) / relayMicroDiff;
		lastRelayHighMicro = now;
	}
	else
		angularPosition += microDiff * angularVelocity;
	
	uint16_t _bit = 1;
	for (byte i = 0; i < ledCount; i++)
	{
		if (isPointInTriangle(getLedCoord(angularPosition, i), triangle))
			ledMatrix |= _bit;
		
		_bit = _bit << 1;
	}
	
	rowIndex = rowIndex == 3 ? 0 : rowIndex + 1;
	updateLeds();
	
	lastMicro = rightNow;
}
