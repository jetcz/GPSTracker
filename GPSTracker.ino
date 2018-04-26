#include <Timezone.h>
#include <TinyGPS++.h>    
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <TimeAlarms.h>

#define DEBUG true

//pins
#define MODULES_POWER D8
#define SIGFOX_RX D5 //gpio 14 module tx
#define SIGFOX_TX D1 //gpio 5 module rx
#define GPS_RX D6 //gpio 12 module tx
#define GPS_TX D7 //gpio 13 module rx
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
//rate at which we are throttling sending messages when engine running, 1 means add one minute every hour, 2 means add two minutes every hour of driving... until MSG_FREQ_MAX_RATE_ENGINE_MINUTES is reached
#define MSG_FREQ_THROTTLE_RATE 1.8
//voltage threshold to decide if we are operating on battery or engine
#define IGNITION_THRESHOLD_VOLTAGE 14
//voltage calibration const (for 10k resistor connected to A0 of the witty board)
#define VOLTAGE_CALIBRATION_CONST 0.0206185567
//maximum allowed time for acquiring gps fix
#define GPSFIX_MAX_TIMEOUT_MINUTES 10
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
TinyGPSPlus tgps;

TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };    //summer time = UTC + 2 hours
TimeChangeRule CET = { "CET", Last, Sun, Oct, 3, 60 };		 //winter time = UTC + 1 hours
Timezone myTZ(CEST, CET);
TimeChangeRule *tcr;

unsigned long msg_frequency = MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000;

bool ignition_changed = false;
bool engine_running = false;
bool usb_powered = false;
bool ignition_change_serviced = true;

int daily_message_count = 0;

unsigned long last_ap_start = 0;
unsigned long last_ignition_change = 0;

String response = "NA";

void setup()
{
	delay(10);

#if DEBUG
	Serial.begin(115200);
	Serial.setTimeout(10);
	while (!Serial) {}

	ArduinoOTA.onStart([]() {
		Serial.println("OTA Start");
	});

	ArduinoOTA.onEnd([]() {
		Serial.println("OTA End");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("OTA Progress: %u%%\r\n", (progress / (total / 100)));
	});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

#endif // DEBUG

	pinMode(MODULES_POWER, OUTPUT);

	Sigfox.begin(9600);
	Sigfox.setTimeout(10);

	GPS.begin(9600);

	payload_prev.timestamp = -9999999;

	enableAP(true);
	powerDevices(true);

	ArduinoOTA.setHostname("GPSTrackerOTA");
	ArduinoOTA.begin();

	Alarm.timerRepeat(1, monitorCarVoltage);
	Alarm.timerRepeat(1, adjustMessageInterval);
	Alarm.timerRepeat(1, process);
	Alarm.timerRepeat(5, controlAP);
	Alarm.timerRepeat(60, setTime);

#ifdef DEBUG
	Alarm.timerRepeat(5, printGPSData);
	Serial.println("Setup done");
#endif // DEBUG
}

void loop()
{
	Alarm.delay(0);
	ArduinoOTA.handle();
	feedGPS();
}

/// <summary>
/// Alarm: process processes :)
/// </summary>
void process()
{
	if (ignition_changed || !ignition_change_serviced)
	{
		processOnIgnitionChanged();
	}
	else if (usb_powered)
	{
		processOnUSBPower();

	}
	else if (engine_running)
	{
		processOnEngineRunning();
	}
	else
	{
		processOnBattery();
	}
}

/// <summary>
/// Alarm: control disabling of ap
/// </summary>
void controlAP()
{
	if (WiFi.getMode() == WIFI_AP
		&& WiFi.softAPgetStationNum() == 0
		&& millis() > last_ap_start + 1 * 60 * 1000)
	{
		enableAP(false);
	}
}


/// <summary>
/// Enable disable wifi AP
/// </summary>
/// <param name="enable"></param>
void enableAP(bool enable)
{
	if (enable)
	{
		last_ap_start = millis();
		WiFi.forceSleepWake();
		WiFi.persistent(false);
		WiFi.mode(WIFI_AP);
		WiFi.softAP("GPSTracker", "12345678");

#if DEBUG
		Serial.println("Wifi AP enabled");
#endif // DEBUG
	}
	else
	{
		WiFi.mode(WIFI_OFF);
		WiFi.forceSleepBegin();
#if DEBUG
		Serial.println("Wifi AP disabled");
#endif // DEBUG
	}

	delay(1);
}

/// <summary>
/// Power on/off Sigfox module and GPS module.
/// </summary>
/// <param name="enable"></param>
void powerDevices(bool enable)
{
	digitalWrite(MODULES_POWER, enable);
	if (enable)
	{
		delay(20); //let sigfox "boot"
	}
}

/// <summary>
/// Process when ignition has changed.
/// This does not contain check for the count of used messages, we want to always receive the message even if we may break the regulatory limits.
/// If we turned the engine off, the modules are powered off.
/// </summary>
void processOnIgnitionChanged()
{
#if DEBUG
	Serial.print("Processing ignition changed");
#endif // DEBUG

	if (ignition_changed)
	{
		enableAP(engine_running);
	}

	if (payload.valid
		&& payload.timestamp - payload_prev.timestamp > 15 * 1000 //send only if last payload is more than 15 sec old
		&& sendPayload())
	{
		daily_message_count++;
		ignition_change_serviced = true;

		if (!engine_running)
		{
			//powerDevices(false); //turn off gps and radio only on success
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
#if DEBUG
	Serial.print("Processing engine running");
#endif // DEBUG

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
#if DEBUG
	Serial.print("Processing on battery");
#endif // DEBUG

	serviceMessageCounter(); //this must go before get gps data, not ideal, but otherwise we would know that we can reset the counter after midnight

	if (daily_message_count <= MSG_MAX_DAILY_COUNT - MSG_RESERVED_COUNT
		&& millis() - payload_prev.timestamp > MSG_FREQ_RATE_BATTERY_MINUTES * 60 * 1000)
	{
		for (int i = 0; i < 10; i++) //try sending few times
		{
			if (payload.valid
				&& sendPayload())
			{
				daily_message_count++;
				//powerDevices(false); //turn off gps only on success
				break;
			}
			else
			{
				//something failed, wait for a while
				Alarm.delay(1000);
			}
		}

		//if we failed to send the location on battery, disable gps and set timestamp of the payload to now to allow waiting and prevent battery discharge
		if (!payload.valid)
		{
			//powerDevices(false);
			payload.timestamp = millis();
			payload_prev = payload;
		}
	}
}

/// <summary>
/// Process when powered from usb (debug)
/// </summary>
void processOnUSBPower()
{
	serviceMessageCounter();

	if (payload.valid
		&& payload.timestamp - payload_prev.timestamp > 10 * 60 * 1000 //send only if last payload is more than 180 sec old
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
	static int day_number = -1;

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
	else if (engine_running)
	{
		//this will gradually lower the frequency of how often we send messages with sigfox
		unsigned long f = ((1 + daily_message_count / MSG_MAX_DAILY_COUNT)* MSG_FREQ_START_RATE_ENGINE_MINUTES * 60 * 1000)
			+ ((millis() - last_ignition_change) / 60 / MSG_FREQ_THROTTLE_RATE);

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
	static bool engine_running_prev = false;

	int counts = analogRead(IGNITION_SENSE_PIN);
	float voltage = counts * VOLTAGE_CALIBRATION_CONST;

#if DEBUG
	if (!usb_powered)
	{
		Serial.println();
		Serial.print("Car voltage counts: ");
		Serial.println(counts);
		Serial.print("Car voltage: ");
		Serial.println(voltage);
	}
#endif // DEBUG

	engine_running = voltage > IGNITION_THRESHOLD_VOLTAGE;
	usb_powered = voltage > 4 && voltage < 6;

	ignition_changed = engine_running_prev != engine_running;

	if (ignition_changed)
	{
		last_ignition_change = millis();
		ignition_change_serviced = false;
	}

	engine_running_prev = engine_running;
}