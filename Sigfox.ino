//AT$SF=D0D0484289587D4117
void sendmockdata()
{
	payload.lat.val = 50.203917;
	payload.lon.val = 15.834115;
	payload.spd = 23;
	payload.valid = true;

	Serial.println("sending mock data");


	Serial.print("AT$SF=");
	for (size_t i = 0; i < sizeof(payload.lat.binary); i++)
	{
		Serial.print(payload.lat.binary[i], HEX);
	}
	for (size_t i = 0; i < sizeof(payload.lon.binary); i++)
	{
		Serial.print(payload.lon.binary[i], HEX);
	}

	Serial.print(payload.spd, HEX);
	Sigfox.print("\n");

	sendPayload();

	while (1)
	{
		if (Sigfox.available())
		{
			Serial.write(Sigfox.read());
		}
	};

}

/// <summary>
/// Wakes the module and sends payload over the Sigfox network.
/// In the end put the module to sleep.
/// </summary>
/// <returns></returns>
bool sendPayload()
{
#ifdef DEBUG
	Serial.println("Sending data over Sigfox");
#endif // DEBUG

	static String response;
	response = "";

	Sigfox.print("\n"); //wakeup
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

	//read response
	while (Sigfox.available() == 0); //wait for sending done
	while (Sigfox.available())
	{
		response += Sigfox.read();
	}

	Sigfox.print("AT$P=1"); //go to sleep

#ifdef DEBUG
	Serial.print("Sigfox response: ");
	Serial.println(response);
#endif // DEBUG

	bool ok = response.substring(0, 2) = "OK";

	if (ok)
	{
		payload_prev = payload;
	}

	return ok;
}