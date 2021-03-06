//AT$SF=D0D0484289587D4117

/// <summary>
/// Wakes the module and sends payload over the Sigfox network.
/// In the end put the module to sleep.
/// </summary>
/// <returns></returns>
bool sendPayload()
{
#if DEBUG
	Serial.println("Sending data over Sigfox");
#endif // DEBUG	

	Sigfox.print("AT$SF=");
	for (int i = 0; i < sizeof(payload.lat.binary); i++)
	{
		Sigfox.print(payload.lat.binary[i], HEX);
	}
	for (int i = 0; i < sizeof(payload.lon.binary); i++)
	{
		Sigfox.print(payload.lon.binary[i], HEX);
	}
	Sigfox.print(payload.spd, HEX);
	Sigfox.print("\n");

	bool ok = getResponse(20);
	if (ok)
	{
		payload_prev = payload;
	}

	return ok;
}

bool getResponse(int wait_sec)
{
	//read response
	for (int i = 0; i < wait_sec * 100; i++) //wait maximum of 20 sec
	{		
		if (Sigfox.available())
		{
			response = Sigfox.readString();
			break;
		}	
		delay(10);	
	}

#if DEBUG
	Serial.print("Sigfox response: ");
	Serial.println(response);
#endif // DEBUG

	bool resp = response.startsWith("OK");
	response = "NA";

	return resp;
}