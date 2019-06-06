/*
*	SMART FlowerPot
*	pinkFarm
*	__________________
*
*	Code by Neshumov Pavel
*/

/* ----- Libaries ----- */
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

/* ----- Pins ----- */
#define VCC_LIGHT		A9
#define GND_LIGHT		A10
#define LIGHT			A8
#define LEDsPin			8

#define WiFi			Serial3

/* ----- Setup ----- */
#define LEDsAmount		23

/* ----- Devices ----- */
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LEDsAmount, LEDsPin, NEO_GRB + NEO_KHZ800);

/* ----- Sysytem vars ----- */
boolean canTransmitt = false;
boolean LEDstage = false;
unsigned long WiFitimer = 0;
unsigned int k, H = 0;
float s = 0.5, v = 0.5;
unsigned long ActionTimer = 0;
struct RGBpixel
{
	uint8_t R = 0;
	uint8_t G = 0;
	uint8_t B = 0;
};
struct RGBpixel RGBcolor;

void setup()
{
	Serial.begin(9600);
	WiFi.begin(115200);
	leds.begin();

	PinsInit();
	sayHello();
	delay(1000);
}

void loop()
{
	TWaction();
	updateFlowerPot();
}

void PinsInit() {
	pinMode(VCC_LIGHT, OUTPUT);
	pinMode(GND_LIGHT, OUTPUT);

	digitalWrite(VCC_LIGHT, 1);
	digitalWrite(GND_LIGHT, 0);
}

/* -------- Data exchange with Thingworx Server  -------- */
void TWaction() {
	if (canTransmitt && millis() - WiFitimer >= 5000) {
		WiFi.print("Light=");
		WiFi.print(map(analogRead(LIGHT), 0, 1024, 0, 100));

		Serial.println("Sended!");
		canTransmitt = false;
		WiFitimer = millis();
	}

	String message = "";
	if (WiFi.available()) {						//Waiting for '>' symbol
		message = WiFi.readStringUntil('\n');

		if (message.startsWith(">"))
			canTransmitt = true;

		else if (message.startsWith("{")) {
			Serial.print("Recieved JSON: ");
			Serial.println(message);

			StaticJsonBuffer<200> jsonBuffer;
			JsonObject& root = jsonBuffer.parseObject(message);

			if (!root.success())
				Serial.println("JSON paring failed");
			else {
				H = (float)root["H"];
				s = (float)root["s"] / 100.0;
				v = (float)root["v"] / 100.0;

				Serial.println("Parsed from JSON: ");
				Serial.print("H: ");
				Serial.print(H);
				Serial.print(" s: ");
				Serial.print(s);
				Serial.print(" v: ");
				Serial.println(v);
			}
		}
		else
			Serial.println(message);
	}
}

/* -------- Enable LEDs, Checking Light  -------- */
void updateFlowerPot() {
	if (millis() - ActionTimer >= 20) {
		int light = analogRead(LIGHT);
		if (light > 900)
			light = 900;
		setLight(H, s, v, map(analogRead(LIGHT), 0, 900, 100, 0));

		ActionTimer = millis();
	}
}

/* -------- LEDs change function  -------- */
void setLight(int H, float s, float v, uint8_t power) {
	HSVtoRGB(H, s, v);							//Calculate RGB color

	RGBcolor.R = map(power, 0, 100, 0, RGBcolor.R);	//Mapping colors by power
	RGBcolor.G = map(power, 0, 100, 0, RGBcolor.G);
	RGBcolor.B = map(power, 0, 100, 0, RGBcolor.B);

	uint8_t InvR = map(k, 255, 0, 0, RGBcolor.R);
	uint8_t InvG = map(k, 255, 0, 0, RGBcolor.G);
	uint8_t InvB = map(k, 255, 0, 0, RGBcolor.B);

	RGBcolor.R = map(k, 0, 255, 0, RGBcolor.R);
	RGBcolor.G = map(k, 0, 255, 0, RGBcolor.G);
	RGBcolor.B = map(k, 0, 255, 0, RGBcolor.B);


	for (uint8_t i = 0; i < LEDsAmount; i += 2) {		//Even LEDs
		if (LEDstage)
			leds.setPixelColor(i, RGBcolor.R, RGBcolor.G, RGBcolor.B);
		else
			leds.setPixelColor(i, InvR, InvG, InvB);

	}
	for (uint8_t i = 1; i < LEDsAmount + 1; i += 2) {		//Odd LEDs
		if (!LEDstage)
			leds.setPixelColor(i, RGBcolor.R, RGBcolor.G, RGBcolor.B);
		else
			leds.setPixelColor(i, InvR, InvG, InvB);
	}
	if (k == 255) {
		k = 0;
		LEDstage = !LEDstage;
	}
	else
		k++;
	leds.show();
}

/* -------- LEDs starting message  -------- */
void sayHello() {
	for (uint8_t i = 0; i < LEDsAmount; i++)
	{
		HSVtoRGB(map(i, 0, 23, 0, 355), 1, 1);
		leds.setPixelColor(i, RGBcolor.R, RGBcolor.G, RGBcolor.B);
		leds.show();
		delay(100);
	}
	delay(1000);
	leds.clear();
}

/* -------- HSV to RGB converter  -------- */
void HSVtoRGB(int h, float s, float v) {
	float r = 0;
	float g = 0;
	float b = 0;

	double hf = h / 60.0;

	int i = (int)floor(h / 60.0);
	float f = h / 60.0 - i;
	float pv = v * (1 - s);
	float qv = v * (1 - s*f);
	float tv = v * (1 - s * (1 - f));

	switch (i)
	{
	case 0:
		r = v;
		g = tv;
		b = pv;
		break;
	case 1:
		r = qv;
		g = v;
		b = pv;
		break;
	case 2:
		r = pv;
		g = v;
		b = tv;
		break;
	case 3:
		r = pv;
		g = qv;
		b = v;
		break;
	case 4:
		r = tv;
		g = pv;
		b = v;
		break;
	case 5:
		r = v;
		g = pv;
		b = qv;
		break;
	}
	RGBcolor.R = constrain((int)255 * r, 0, 255);
	RGBcolor.G = constrain((int)255 * g, 0, 255);
	RGBcolor.B = constrain((int)255 * b, 0, 255);
}
