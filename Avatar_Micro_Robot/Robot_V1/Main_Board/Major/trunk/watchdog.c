#include "system.h"



void kick_watchdogs(void)
{

	static unsigned char last_wdt_state = 0;

	ClrWdt();
	
	if(last_wdt_state)
	{
		last_wdt_state = 0;
		WDT_PIN = 0;
	}
	else
	{
		last_wdt_state = 1;
		WDT_PIN = 1;

	}






}
