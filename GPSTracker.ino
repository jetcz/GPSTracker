#include <Timezone.h>
#include <TinyGPS.h>    
#include <SoftwareSerial.h>

#define DEBUG true

//pins
#define MODULES_POWER D8
#define SIGFOX_RX D5
#define SIGFOX_TX D1
#define GPS_RX D6
#define GPS_TX D7
#define IGNITION_SENSE_PIN A0

//constants
//max message count per day by sigfox ETSI regulatory  
#define MSG_MAX_DAILY_COUNT 140
//reserved message count for ignition events
#define MSG_RESERVED_COUNT 5
//how often we send message when operating on battery
#define MSG_FREQ_RATE_BATTERY_MINUTES 180
//how often we send message when operating on engine running - starting period
#define MSG_FREQ_START_RATE_ENGINE_MINUTES 5
//how often we send message when operating on engine running - max period
#define MSG_FREQ_MAX_RATE_ENGINE_MINUTES 15
//rate at which we are throttling sending messages when engine running, 1 means add one minute every hour, 2 means add two minutes every hour... until MSG_FREQ_MAX_RATE_ENGINE_MINUTES is reached
#define MSG_FREQ_THROTTLE_RATE 2
//voltage threshold to decide if we are operating on battery or engine
#define IGNITION_THRESHOLD_VOLTAGE 14
//minimum voltage step between battery operation and engine operation
#define IGNITION_THRESHOLD_VOLTAGE_DIFF 1.5
//voltage calibration const (for 10k resistor connected to A0 of the witty board)
#define VOLTAGE_CALIBRATION_CONST 0.0206185567
//maximum allowed time for acquiring gps fix
#define GPSFIX_MAX_TIMEOUT_MINUTES 3
//maximum allowed time for which is gps fix considered valid
#define GPSFIX_MAX_AGE_MINUTES 3

/// <summary>
/// Provides byte representation of the float values int the payload
/// </summary>
typedef union {
	float val;
	byte binary[4];
} binaryFloat;

/// <summary>
/// Payload sent over Sigfox
/// </summary>
typedef struct Payload
{
	bool valid;
	binaryFloat lat;
	binaryFloat lon;
	byte spd;
	unsigned long timestamp; //not sent
} Payload;

Payload payload;
Payload payload_prev;

SoftwareSerial Sigfox = SoftwareSerial(SIGFOX_RX, SIGFOX_TX);
SoftwareSerial GPS = SoftwareSerial(GPS_RX, GPS_TX);
TinyGPS tgps;

TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };    //summer time = UTC + 2 hours
TimeChangeRule CET = { "CET", Last, Sun, Oct, 3, 60 };		 //winter time = UTC + 1 hours
Timezone myTZ(CEST, CET);
TimeChangeRule *tcr;

unsigned long fix_age;
unsigned long msg_frequency = MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000;

bool ignition_changed = false;
bool engine_running = false;
bool usb_powered = false;

int day_number = -1;
int daily_message_count = 0;

void setup()
{
	pinMode(MODULES_POWER, OUTPUT);

	Serial.begin(9600);
	Sigfox.begin(9600);
	GPS.begin(4800);

	payload_prev.timestamp = -9999999;

	//testSigfoxModule();
	//sendmockdata();

}

void testSigfoxModule()
{
	Sigfox.print("AT");
	Sigfox.print("\n");
	delay(100);
	if (Sigfox.available())
	{
		Serial.write(Sigfox.read());
	}
	else
	{
		Serial.println("sigfox module not responding");
		while (1)
		{
			if (Serial.available()) {
				Sigfox.write(Serial.read());
			}

			if (Sigfox.available()) {
				Serial.write(Sigfox.read());
			}
		}
	}
}

void loop()
{
	monitorCarVoltage();

	adjustMessageInterval();

	if (ignition_changed)
	{
		processOnIgnitionChanged();
		ESP.deepSleep(5e6);
	}
	else if (usb_powered)
	{
		processOnUSBPower();
		ESP.deepSleep(5e6);
	}
	else if (engine_running)
	{
		processOnEngineRunning();
		ESP.deepSleep(5e6);
	}
	else
	{
		processOnBattery();
		ESP.deepSleep(20e6);
	}
}

/// <summary>
/// Power on/off Sigfox module and GPS module.
/// </summary>
/// <param name="enable"></param>
void powerDevices(bool enable)
{
	digitalWrite(MODULES_POWER, enable);
}

/// <summary>
/// Process when ignition has changed.
/// This does not contain check for the count of used messages, we want to always receive the message even if we may break the regulatory limits.
/// If we turned the engine off, the modules are powered off.
/// </summary>
void processOnIgnitionChanged()
{
#ifdef DEBUG
	Serial.print("Processing ignition changed");
#endif // DEBUG

	getGPSData();
	serviceMessageCounter();

	if (payload.valid
		&& payload.timestamp - payload_prev.timestamp > 15 * 1000 //send only if last payload is more than 15 sec old
		&& sendPayload())
	{
		daily_message_count++;

		if (!engine_running)
		{
			powerDevices(false); //turn off gps and radio only on success
		}
	}
}

/// <summary>
/// Process when engine is running.
/// The message sending starts of quite fast and slows down gradualy as we are driving.
/// The modules are always powered.
/// </summary>
void processOnEngineRunning()
{
#ifdef DEBUG
	Serial.print("Processing engine running");
#endif // DEBUG

	getGPSData();
	serviceMessageCounter();

	if (payload.valid
		&& daily_message_count <= MSG_MAX_DAILY_COUNT - MSG_RESERVED_COUNT
		&& payload.timestamp - payload_prev.timestamp > msg_frequency
		&& sendPayload())
	{
		daily_message_count++;
	}
}

/// <summary>
/// Process when engine is off.
/// We send message only occasionally, but we need it to come through. 
/// The car may be standing under a tree or underground garage, so we try to send the message a few times in a row until it works.
/// Also the modules are powered off and GPS cold start may take a while.
/// </summary>
void processOnBattery()
{
#ifdef DEBUG
	Serial.print("Processing on battery");
#endif // DEBUG

	serviceMessageCounter(); //this must go before get gps data, not ideal, but otherwise we would know that we can reset the counter after midnight

	if (daily_message_count <= MSG_MAX_DAILY_COUNT - MSG_RESERVED_COUNT
		&& millis() - payload_prev.timestamp > MSG_FREQ_RATE_BATTERY_MINUTES * 60 * 1000)
	{
		for (int i = 0; i < 3; i++) //try sending few times
		{
			getGPSData();

			if (payload.valid
				&& sendPayload())
			{
				daily_message_count++;
				powerDevices(false); //turn off gps and radio only on success
				break;
			}
			else
			{
				//something failed, wait for a while
				ESP.deepSleep(60e6); //sleep 60 sec
			}
		}

		//if we failed to send the location on battery, disable gps and set timestamp of the payload to now to allow waiting and prevent battery discharge
		if (!payload.valid)
		{
			powerDevices(false);
			payload.timestamp = millis();
		}
	}
}

/// <summary>
/// Process when powered from usb (debug)
/// </summary>
void processOnUSBPower()
{
#ifdef DEBUG
	Serial.print("Processing usb powered");
#endif // DEBUG

	getGPSData();
	serviceMessageCounter();

	if (payload.valid
		&& payload.timestamp - payload_prev.timestamp > 60 * 1000 //send only if last payload is more than 60 sec old
		&& sendPayload())
	{
		daily_message_count++;
	}
}

/// <summary>
/// Resets the message counter after UTC midnight.
/// </summary>
void serviceMessageCounter()
{
	if (day() != day_number)
	{
		day_number = day();
		daily_message_count = 0;
	}
}

/// <summary>
/// Adjusts the message sending interval. It gets longer while driving.
/// It is reset on ignition change.
/// </summary>
void adjustMessageInterval()
{
	if (ignition_changed)
	{
		msg_frequency = MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000; //reset the frequency on ignition change
	}
	else
	{
		//this will gradually lower the frequency of how often we send messages with sigfox
		unsigned long f = MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000 + millis() / 60 / MSG_FREQ_THROTTLE_RATE;

		if (f < MSG_FREQ_MAX_RATE_ENGINE_MINUTES * 60 * 1000)
		{
			msg_frequency = f;
		}
	}
}

/// <summary>
/// Monitor car voltage and set variables indicating ignition changed event and engine state.
/// </summary>
void monitorCarVoltage()
{
	static float voltage_prev = 0;

	int counts = analogRead(IGNITION_SENSE_PIN);
	float voltage = counts * VOLTAGE_CALIBRATION_CONST;

#ifdef DEBUG
	Serial.println();
	Serial.print("Car voltage counts: ");
	Serial.println(counts);
	Serial.print("Car voltage: ");
	Serial.println(voltage);
#endif // DEBUG

	engine_running = voltage > IGNITION_THRESHOLD_VOLTAGE;
	usb_powered = voltage > 4 && voltage < 6;

	if (voltage_prev = 0) //after bootup
	{
		voltage_prev = voltage;
	}
	else
	{
		bool ignition_turned_off = !engine_running
			&& voltage_prev + IGNITION_THRESHOLD_VOLTAGE_DIFF > voltage;
		bool ignition_turned_on = engine_running
			&& voltage_prev < voltage + IGNITION_THRESHOLD_VOLTAGE_DIFF;

		ignition_changed = ignition_turned_off || ignition_turned_on;
		voltage_prev = voltage;
	}
}