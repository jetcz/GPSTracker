#include <Timezone.h>
#include <TinyGPS.h>    
#include <SoftwareSerial.h>

#define DEBUG true

//pins
#define SIGFOX_RX 7
#define SIGFOX_TX 8
#define GPS_RX 9
#define GPS_TX 10
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
//maximum allowed time for acquiring gps fix
#define GPSFIX_MAX_TIMEOUT_MINUTES 3
//maximum allowed time for which is gps fix considered valid
#define GPSFIX_MAX_AGE_MINUTES 3

SoftwareSerial Sigfox = SoftwareSerial(SIGFOX_RX, SIGFOX_TX);
SoftwareSerial GPS = SoftwareSerial(GPS_RX, GPS_TX);
TinyGPS tgps;

typedef union {
	float val;
	byte binary[4];
} binaryFloat;

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

TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };    //summer time = UTC + 2 hours
TimeChangeRule CET = { "CET", Last, Sun, Oct, 3, 60 };		 //winter time = UTC + 1 hours
Timezone myTZ(CEST, CET);
TimeChangeRule *tcr;

unsigned long fix_age;
unsigned long msg_frequency = MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000;

bool ignition_changed = false;
bool engine_running = false;

int day_number = -1;
int daily_message_count = 0;

void setup()
{
	Serial.begin(9600);
	Sigfox.begin(9600);
	GPS.begin(4800);

	payload_prev.timestamp = -9999999;

	//testSigfoxModule();
	sendmockdata();
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
	}
	else
	{
		if (engine_running)
		{
			processOnEngineRunning();
		}
		else
		{
			processOnBattery();
		}
	}

	ESP.deepSleep(10e6); //sleep 10 sec
}

void processOnIgnitionChanged()
{
#ifdef DEBUG
	Serial.print("Processing ignition changed");
#endif // DEBUG

	//do not check daily message count limit here, send always even if we may break the regulation limits....

	getGPSData();
	serviceMessageCounter();

	if (payload.valid
		&& payload.timestamp - payload_prev.timestamp > 60 * 1000 //send only if last payload is more than 60 sec old
		&& sendPayload())
	{
		daily_message_count++;

		if (!engine_running)
		{
			enableGPS(false); //turn off gps only on success
		}
	}
}

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

void processOnBattery()
{
#ifdef DEBUG
	Serial.print("Processing on battery");
#endif // DEBUG

	serviceMessageCounter(); //this must go before get gps data, not ideal, but otherwise we would know that we can reset the counter after midnight

	if (daily_message_count <= MSG_MAX_DAILY_COUNT - MSG_RESERVED_COUNT
		&& millis() - payload_prev.timestamp > MSG_FREQ_RATE_BATTERY_MINUTES * 60 * 1000)
	{
		for (int i = 0; i < 5; i++) //try sending few times
		{
			getGPSData();

			if (payload.valid
				&& sendPayload())
			{
				daily_message_count++;
				enableGPS(false); //turn off gps only on success
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
			enableGPS(false);
			payload.timestamp = millis();
		}
	}
}

void serviceMessageCounter()
{
	if (day() != day_number)
	{
		day_number = day();
		daily_message_count = 0;
	}
}

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

void monitorCarVoltage()
{
	//---- - A0
	//	|
	//	220K
	//	| -- - ADC
	//	100K
	//	|
	//	GND
	//Wemos D1 Mini has already build in divider R1 220k / R2 100k for pin A0.
	//That's why you can connect 3.3V to pin A0 even datasheet for ESP8266 says that max voltage for A0 is 1V.
	//You only need add resitor in series with R1 to increase max voltage range.
	//For 12V has to be R1 1100k. So 1100k minus 220k (already there) is 880k. You don't need to build another divider R1 / R2.

	static float voltage_prev = 0;

	float voltage = analogRead(IGNITION_SENSE_PIN) * (15 / 1023.0);

	engine_running = voltage > IGNITION_THRESHOLD_VOLTAGE;

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