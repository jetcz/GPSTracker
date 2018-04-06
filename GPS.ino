void getGPSData()
{
	enableGPS(true); //wake up in case we are in pwr saving

	unsigned long fixing_start = millis();

	while (!payload.valid || millis() - fixing_start < GPSFIX_MAX_TIMEOUT_MINUTES * 60 * 1000)
	{
		if (GPS.available())
		{
			if (tgps.encode(GPS.read()))
			{
				tgps.f_get_position(&payload.lat.val, &payload.lon.val, &fix_age);
				if (fix_age == TinyGPS::GPS_INVALID_AGE || fix_age > GPSFIX_MAX_AGE_MINUTES * 60 * 1000) //ignore fix older than 3 minutes
				{
#ifdef DEBUG
					Serial.print("No fix detected or fix expired: ");
					Serial.println(fix_age);
#endif // DEBUG

					payload.valid = false;
				}
				else
				{
					payload.valid = true;
					payload.spd = (byte)tgps.f_speed_kmph();
					payload.timestamp = millis();
					setTime();
				}
			}
			else
			{
#ifdef DEBUG
				Serial.println("GPS provided invalid sentence");
#endif // DEBUG
			}
		}
		else
		{
#ifdef DEBUG
			Serial.println("GPS not available, going to sleep for 3 sec");
#endif // DEBUG

			ESP.deepSleep(3e6); //sleep 3 sec
		}
	}

#ifdef DEBUG

	if (payload.valid)
	{
		time_t utc = now();
		time_t local = myTZ.toLocal(utc, &tcr);

		Serial.println("GPS obtained position:");
		Serial.print("LAT: ");
		Serial.println(payload.lat.val);
		Serial.print("LON: ");
		Serial.println(payload.lon.val);
		Serial.print("SPD: ");
		Serial.println(payload.spd);
		Serial.print("Fix age: ");
		Serial.println(fix_age);
		Serial.print("Current local datetime: ");
		Serial.println(parseUnixTime(local));
		Serial.print("Current utc datetime: ");
		Serial.println(parseUnixTime(utc));
	}
	else
	{
		Serial.println("GPS did not obtain position!");
	}
#endif // DEBUG

}

String parseUnixTime(time_t)
{
	return String(day())
		+ "-" + String(month())
		+ "-" + String(year())
		+ " " + String(hour())
		+ ":" + String(minute())
		+ ":" + String(second());
}

void enableGPS(bool enable)
{
	//todo
}

void setTime()
{
	int Year;
	byte Month, Day, Hour, Minute, Second;
	tgps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &fix_age);
	if (fix_age < 1000)
	{
		tmElements_t e = { Second, Minute, Hour, Day, Month, Year - 1970 };
		time_t t = makeTime(e);
		setTime(t);
	}
}


float calcDist(float flat1, float flon1, float flat2, float flon2)
{
	float dist_calc = 0;
	float dist_calc2 = 0;
	float diflat = 0;
	float diflon = 0;

	diflat = radians(flat2 - flat1);
	flat1 = radians(flat1);
	flat2 = radians(flat2);
	diflon = radians((flon2)-(flon1));

	dist_calc = (sin(diflat / 2.0)*sin(diflat / 2.0));
	dist_calc2 = cos(flat1);
	dist_calc2 *= cos(flat2);
	dist_calc2 *= sin(diflon / 2.0);
	dist_calc2 *= sin(diflon / 2.0);
	dist_calc += dist_calc2;

	dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

	dist_calc *= 6371000.0; //Converting to meters

	return dist_calc;
}
