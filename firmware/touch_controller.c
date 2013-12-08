#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "rako.h"

#define GET_BIT(value, bit)    ((value) & (1 << (bit)))
#define SET_BIT(value, bit)    ((value) |= 1 << (bit))
#define CLR_BIT(value, bit)    ((value) &= ~(1 << (bit)))
#define BIT(bit)               (1 << (bit))

#define PORTCAT(a,b,c)     a##b##c
#define PORT(port)         PORTCAT(PORT,port,)
#define PORTPIN(port,pin)  PORTCAT(PIN,port,pin)
#define DDR(port)          PORTCAT(DDR,port,)
#define DD(port,pin)       PORTCAT(DD,port,pin)

#define PIN_GET(x)    GET_BIT(PORT(x##_PORT), PORTPIN(x##_PORT,x##_PIN))
#define PIN_HIGH(x)   SET_BIT(PORT(x##_PORT), PORTPIN(x##_PORT,x##_PIN))
#define PIN_LOW(x)    CLR_BIT(PORT(x##_PORT), PORTPIN(x##_PORT,x##_PIN))
#define PIN_OUTPUT(x) SET_BIT(DDR(x##_PORT),  DD(x##_PORT,x##_PIN))
#define PIN_INPUT(x)  CLR_BIT(DDR(x##_PORT),  DD(x##_PORT,x##_PIN))

#define RADIO_ENABLE_PORT  A
#define RADIO_ENABLE_PIN   0

#define RADIO_DATA_PORT    A
#define RADIO_DATA_PIN     1

#define SENSOR_ENABLE_PORT A
#define SENSOR_ENABLE_PIN  5

#define DEBUG_PORT         A
#define DEBUG_PIN          4


unsigned int g_sendPos = 0;
unsigned int g_sendBitsTotal = 0;
unsigned char g_sendData[16];
unsigned int pressed = 0;

unsigned long g_time = 0;
ISR(TIMER1_OVF_vect)
{
	++g_time;
}

inline unsigned long micros()
{
	return ((g_time << 8) + TCNT1) << 10;
}

inline void delay(unsigned long microseconds)
{
	unsigned long endTime = micros() + microseconds;
	while(micros() < endTime)
		;
}

ISR(ANA_COMP_vect)
{
//	if(pressed == 0)
//		if(ACSRA & (1 << ACO))
//			pressed = 1;
}

void sendTimerStart()
{
	PIN_HIGH(DEBUG);

	PIN_HIGH(RADIO_ENABLE);

	TCNT0L = 0;
	TCNT0H = 0;
	SET_BIT(TIFR, OCF0A);
	SET_BIT(TIMSK, OCIE0A);
}

void sendTimerStop()
{
	CLR_BIT(TIMSK, OCIE0A);

	PIN_LOW(RADIO_ENABLE);
	PIN_LOW(RADIO_DATA);

	PIN_LOW(DEBUG);
}

ISR(TIMER0_COMPA_vect)
{
	TCNT0L = 0;
	TCNT0H = 0;

	if(g_sendPos < g_sendBitsTotal)
	{
		if((g_sendData[g_sendPos / 8] & (1 << (g_sendPos % 8))) != 0)
			PIN_HIGH(RADIO_DATA);
		else
			PIN_LOW(RADIO_DATA);
		++g_sendPos;
	}
	else
		sendTimerStop();
}

inline int isSendInProgress()
{
	return PIN_GET(RADIO_ENABLE);
}

void sendDataReset()
{
	memset(g_sendData, 0, sizeof(g_sendData));
	g_sendBitsTotal = 0;
	g_sendPos = 0;
}

void sendDataAppend(const char* bits)
{
	while(*bits != '\0')
	{
		if(*bits == '1')
			g_sendData[g_sendBitsTotal / 8] |= 1 << (g_sendBitsTotal % 8);
		++g_sendBitsTotal;
		++bits;
	}
}

void send(RakoMsg* msg)
{
	int msgCount;

	for(msgCount = 0; msgCount < 4; ++msgCount)
	{
		int i;
		uint32_t data = msg->raw;
		int check = 0;
		unsigned long startTime = micros();

		while(isSendInProgress())
			if(micros() - startTime > 1000000)
				break;

		sendDataReset();

		// Preamble and start mark
		sendDataAppend("10101011110");

		// Message body
		for(i = 0; i < 28; ++i)
		{
			if(data & 0x80000000UL)
			{
				++check;
				sendDataAppend("110");
			}
			else
				sendDataAppend("10");
			data <<= 1;
		}

		// Check bit
		if(check & 1)
			sendDataAppend("110");
		else
			sendDataAppend("10");

		// Stop mark
		sendDataAppend("11110");

		sendTimerStart();

		delay(100000);
	}
}

int get()
{
	int count = 0;
	int last = 0;

	for(;;)
	{
		int current = ACSRA & (1 << ACO);
		if(current != last)
		{
			count = 0;
			last = current;
		}
		else
		{
			if(++count >= 50)
				return !current;
		}
	}
}

int main()
{
	RakoMsg msg;

	// Set timer 0 to clk/64 => 1 tick every 16 uS
	SET_BIT(TCCR0B, CS00);
	SET_BIT(TCCR0B, CS01);
	// compare after 34 ticks => 544uS
	OCR0A = 34;	

	// Set timer 1 to clk/4096 => 1 tick every 1024 uS
	SET_BIT(TCCR1B, CS10);
	CLR_BIT(TCCR1B, CS11);
	SET_BIT(TCCR1B, CS12);
	SET_BIT(TCCR1B, CS13);
	OCR1C = 255;
	SET_BIT(TIMSK, TOIE1);

	PIN_LOW(RADIO_ENABLE);
	PIN_LOW(RADIO_DATA);
	PIN_LOW(SENSOR_ENABLE);
	PIN_LOW(DEBUG);

	PIN_OUTPUT(RADIO_ENABLE);
	PIN_OUTPUT(RADIO_DATA);
	PIN_OUTPUT(SENSOR_ENABLE);
	PIN_OUTPUT(DEBUG);


	cli();
	ACSRA = (1 << ACIE);
	sei();


	msg.command.type = 0;
	msg.command.house = 5;
	msg.command.room = 4;
	msg.command.channel = 2;

	PIN_HIGH(SENSOR_ENABLE);

	for(;;)
	{
		static int last = 0;
		static int current = 0;
		static int on = 0;
		static int fading = 0;
		static unsigned long pressStartTime = 0;

		current = get();
		if(current && !last)
		{
			pressStartTime = micros();
		}
		else if(current && last)
		{
			unsigned long duration = micros() - pressStartTime;

			if(!fading && duration >= 500000)
			{
				fading = 1;

				if(on == 0)
				{
					msg.command.command = RAKO_CMD_RAISE;
					send(&msg);
					on = 1;
				}
				else
				{
					msg.command.command = RAKO_CMD_LOWER;
					send(&msg);
					on = 0;
				}
			}
			
			if(duration >= 10000000)
			{
				// Pressed for 10 seconds or more, power cycle the sensor
				// to force it to recalibrate
				PIN_LOW(SENSOR_ENABLE);
				delay(500000);
				PIN_HIGH(SENSOR_ENABLE);
				last = current = 0;
			}
		}
		else if(!current && last)
		{
			if(fading)
			{
				msg.command.command = RAKO_CMD_STOP;
				send(&msg);
				fading = 0;
			}
			else
			{
				if(on == 0)
				{
					msg.command.command = RAKO_CMD_SCENE1;
					send(&msg);
					on = 1;
				}
				else
				{
					msg.command.command = RAKO_CMD_OFF;
					send(&msg);
					on = 0;
				}
			}
		}

		last = current;
	}
}
