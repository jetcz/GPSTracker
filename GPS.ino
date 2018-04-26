/// <summary>
/// Feed gps object with sentences from the GPS module
/// </summary>
void feedGPS()
{
	while (GPS.available() > 0)
	{
		if (tgps.encode(GPS.read()))
		{
			getGPSData();
		}
	}
}

/// /// <summary>
/// Obtain GPS data - this may take a while
/// </summary>
void getGPSData()
{
	if (tgps.location.isValid()) //&& tgps.date.age() < 1500
	{
		Serial.println("payload valid");
		payload.lat.val = (float)tgps.location.lat();
		payload.lon.val = (float)tgps.location.lng();
		payload.spd = (byte)tgps.speed.kmph();

		payload.valid = true;
		payload.timestamp = millis();
	}
	else
	{
		payload.valid = false;
	}
}

/// <summary>
/// Print gps data
/// </summary>
void printGPSData()
{
	if (payload.valid)
	{
		time_t utc = now();
		time_t local = myTZ.toLocal(utc, &tcr);

		Serial.print("GPS obtained position (");
		Serial.print(tgps.satellites.value());
		Serial.println("):");
		Serial.print("LAT: ");
		Serial.println(payload.lat.val);
		Serial.print("LON: ");
		Serial.println(payload.lon.val);
		Serial.print("SPD: ");
		Serial.println(payload.spd);
		Serial.print("Fix age: ");
		Serial.println(tgps.date.age());
		Serial.print("Current local datetime: ");
		Serial.println(parseUnixTime(local));
		Serial.print("Current utc datetime: ");
		Serial.println(parseUnixTime(utc));
	}
	else
	{
		Serial.print("No fix detected or fix expired (");
		Serial.print(tgps.satellites.value());
		Serial.print("): ");
		Serial.println(tgps.date.age());
		Serial.print(F("DIAGS      Chars="));
		Serial.print(tgps.charsProcessed());
		Serial.print(F(" Sentences-with-Fix="));
		Serial.print(tgps.sentencesWithFix());
		Serial.print(F(" Failed-checksum="));
		Serial.print(tgps.failedChecksum());
		Serial.print(F(" Passed-checksum="));
		Serial.println(tgps.passedChecksum());
	}

	Serial.println();
}

/// <summary>
/// Create readable datetime from time_t object
/// </summary>
/// <param name=""></param>
/// <returns></returns>
String parseUnixTime(time_t)
{
	return String(day())
		+ "-" + String(month())
		+ "-" + String(year())
		+ " " + String(hour())
		+ ":" + String(minute())
		+ ":" + String(second());
}

/// <summary>
/// Sets current time from data stored in GPS, call only if we have valid GPS fix
/// </summary>
void setTime()
{
	if (tgps.date.isValid()
		&& tgps.time.isValid()
		&& tgps.date.year() >= 2018) //:-)
	{
		setTime(tgps.time.hour(), tgps.time.minute(), tgps.time.second(), tgps.date.day(), tgps.date.month(), tgps.date.year());

#ifdef DEBUG
		Serial.println(parseUnixTime(now()));
#endif // DEBUG		
	}
}