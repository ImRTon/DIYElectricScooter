/*Header files*/
#include <Servo.h>
/*Header files*/

/*Pin definitions*/
const int PULSE_IN_PIN = 2;	   //馬達速度
const int MOTOR_SPEED_PIN = 4; // unused
const int MOTOR_PIN = 5;
const int BREAKPIN_FRONT_PIN = 6;
const int BREAKPIN_BACK_PIN = 8;
const int TAILLIGHT_PIN = 9;
const int TEMP_PIN = A0;
const int THROTTLE_PIN = A2;
const int PRESERVE_BLUE = A3;
const int HORN_PIN = A4;
const int PRESERVE_ORANGE = A5;
/*Pin definitions*/

/*Settings*/
#define START_DELAY 3000 /*in mS*/
#define UPDATE_INTERVAL 200
#define UPDATETimeLength 400 /*the update frequentcy*/
#define POWER_WALL 160		 /*功耗牆 (馬達滿載180)*/

#define DEBUG 1
#define NoControl 0			 // define if Control motor from outside
#define HONK_PERIOD 400		 //喇叭每段持續時間
#define HONK_FRE 2000		 //喇叭音色
#define TAILLIGHT_PERIOD 250 //尾燈每段持續時間

#define BREAK_EN 1
#define BREAK_DIS 0

#define THERMO_LIMIT 50
#define THERMO_LIMIT_HIGH 60

#define THROTTLE_LOW 210
#define THROTTLE_HIGH 830
/*Settings*/

/*Global Variables*/
float currentOutput = 0;	   //現在的油門輸出
long long lastUpdate = 0;	   //上次的轉速更新
long long lastUpdateMotor = 0; //上次馬達更新
volatile int pulseCount = 0;   //馬達 Pulse 計數
double CurrentSpeed = 0;	   /*The current speed of the scooter (km/h)*/
Servo Motor;
boolean isHonk = 0;
long long lastHonk = 0;
boolean isTailLight = 1;
long long lastTailLight = 0;
long long lastSpeedUpdate = 0; //上次更新速度的時間點，用於控制加速減速的Timing

/*Global Variables*/

void setup()
{
#ifdef DEBUG
	Serial.begin(9600);
#endif // DEBUG

	/*DigitalPins*/
	pinMode(BREAKPIN_BACK_PIN, INPUT);
	/*DigitalPins*/

	/*PWM Pins*/
	pinMode(TAILLIGHT_PIN, OUTPUT);
	/*PWM Pins*/

	/*Analog Pins*/
	// pinMode(TEMP_PIN, INPUT);
	// pinMode(THROTTLE_PIN, INPUT);
	/*Analog Pins*/

	/*Interrupt*/
	attachInterrupt(1, counter, CHANGE);
	/*Interrupt*/

	/*Motor*/
	Motor.attach(MOTOR_PIN, 1000, 2400);
	Motor.write(0);
	delay(START_DELAY); // waiting for the ECT to initialize.
	lastSpeedUpdate = millis();
	/*Motor*/
}

void loop()
{

	if (millis() > lastUpdate + UPDATE_INTERVAL)
	{ // update current speed.

		CurrentSpeed = pulseCount * 0.199956; // Speed formula

#ifdef DEBUG
		Serial.print("Pulses : ");
		Serial.println(pulseCount);
		Serial.print("Speed : ");
		Serial.println(CurrentSpeed);
		Serial.print("Temp : ");
		Serial.println(thermometer(analogRead(TEMP_PIN)));
#endif // DEBUG

		pulseCount = 0;
		lastUpdate = millis();
	}

	if (digitalRead(BREAKPIN_BACK_PIN) == 1)
	{ // Break

#ifdef DEBUG
		Serial.println("Break!");
#endif // DEBUG
		Motor.write(0);
		TailLightManager(BREAK_EN);
		// analogWrite(TAILLIGHT_PIN, 255);
		if (analogRead(THROTTLE_PIN) > 800)
			hornManager(1);
	}
	else
	{ // Not breaking

		// analogWrite(TAILLIGHT_PIN, 150);
		int temp = thermometer(analogRead(TEMP_PIN));
		int throuttle = throttleControl(analogRead(THROTTLE_PIN), temp);
#ifdef DEBUG
		Serial.print("Throuttle :");
		Serial.println(throuttle);
		Serial.print("Temp :");
		Serial.println(temp);
#endif // DEBUG
		Motor.write(throuttle);
		if (temp > THERMO_LIMIT_HIGH)
		{
			tone(HORN_PIN, 1000, 1000);
			delay(1150);
			tone(HORN_PIN, 2500, 200);
			delay(200);
			tone(HORN_PIN, 2500, 200);
		}

		TailLightManager(BREAK_DIS);
	}
}

void counter()
{
	pulseCount++;
}

int thermometer(int tempRead)
{
	double R1 = 10000.0;  // voltage divider resistor value
	double Beta = 3950.0; // Beta value
	double To = 298.15;	  // Temperature in Kelvin for 25 degree Celsius
	double Ro = 10000.0;  // Resistance of Thermistor at 25 degree Celsius

	double Vout, Rt = 0;
	double T = 0;

	Vout = tempRead * 5.0 / 1023.0;
	Rt = R1 * Vout / (5 - Vout);
	T = 1 / (1 / To + log(Rt / Ro) / Beta); // Temperature in Kelvin
	T = T - 273.15;							// Celsius
	return T;
}

int clamp(int value, int min, int max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	else
		return value;
}

int throttleControl(int throttleRead, int thermo)
{
#ifdef DEBUG
	Serial.print("Throuttle Read :");
	Serial.println(throttleRead);
#endif // DEBUG
	// Range: 205~831 -> 210~830
	if (throttleRead < 210)
	{
		currentOutput = 0;
		lastSpeedUpdate = millis();
		return int(currentOutput);
	}

	throttleRead = clamp(throttleRead, THROTTLE_LOW, THROTTLE_HIGH);

	float throttleMax = POWER_WALL;

	// 速度控制
	if (CurrentSpeed < 8) {
		throttleMax = 0;
	} else {
		//7~20
		//50~180
		throttleMax = (CurrentSpeed - 7.0) / 13.0 * 130.0 + 50;
	}

	// 溫控
	if (thermo > THERMO_LIMIT) {
		throttleMax *= ((THERMO_LIMIT_HIGH - clamp(thermo, THERMO_LIMIT, THERMO_LIMIT_HIGH)) / (float)(THERMO_LIMIT_HIGH - THERMO_LIMIT));
	}

	// 電流控制
	// TODO
	currentOutput = clamp((throttleRead - THROTTLE_LOW) / (float)(THROTTLE_HIGH - THROTTLE_LOW) * POWER_WALL, 0, throttleMax);
	return int(currentOutput);
}

void hornManager(boolean honkControl)
{
	if (honkControl == 1)
	{
		if (millis() >= lastHonk + HONK_PERIOD * 2)
		{
			tone(HORN_PIN, HONK_FRE, HONK_PERIOD);
			lastHonk = millis();
		}
	}
}

void TailLightManager(boolean BreakControl)
{
	if (BreakControl == 1)
	{
		if (millis() >= lastTailLight + TAILLIGHT_PERIOD / 1.5 && isTailLight == 1)
		{
			digitalWrite(TAILLIGHT_PIN, 0);
			lastTailLight = millis();
			isTailLight = 0;
		}
		else if (millis() >= lastTailLight + TAILLIGHT_PERIOD && isTailLight == 0)
		{
			digitalWrite(TAILLIGHT_PIN, 1);
			lastTailLight = millis();
			isTailLight = 1;
		}
	}
	else
	{
		digitalWrite(TAILLIGHT_PIN, 1);
	}
}
