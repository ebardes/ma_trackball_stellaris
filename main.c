/*
   Copyright 2013 Bardes Lighting, LLC

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

// read and write ps/2 protocol
// in particular, talk to a ps/2 trackball, and return it's values over a serial port.
//
// hippy 2013

#include "inc/lm4f120h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// ps/2 connection pins
//
// pull-up on each pin (10kohm to +3.3v)
// clock - PA4
// data  - PA5
//

#define PS2_BASE  GPIO_PORTA_BASE
#define CLK_PIN   GPIO_PIN_4
#define DAT_PIN   GPIO_PIN_5

#define BUTTON_1 GPIO_PIN_4
#define DEBOUNCE 50

// possible signal states
enum
{
	PS2_STATE_IDLE,
	PS2_STATE_DATA,
	PS2_STATE_PARITY,
	PS2_STATE_STOP,
	PS2_STATE_DONE
};

// PS/2 transmission mode
enum
{
	HOST_TO_SLAVE,
	SLAVE_TO_HOST
};

// slave states
enum
{
	SLAVE_DEAD,
	SLAVE_INITIALIZED,
	SLAVE_ACK_EXPECTED,
	SLAVE_DATA_EXPECTED
};



// uptime in ticks
volatile unsigned long sysTickUptime = 0;

// ps/2 byte buffers
volatile unsigned long g_ulRXCode = 0;  // RX
volatile unsigned char g_ulTXCode = 0;  // TX

// state machines
volatile unsigned char g_ucPS2RXState;  // current RX state
volatile unsigned char g_ucPS2TXState = PS2_STATE_IDLE;  // current TX state
volatile unsigned char g_ucPS2Mode = SLAVE_TO_HOST;  // current transmission mode
volatile unsigned char g_ucPS2SlaveState = SLAVE_DEAD; // current state of the slave
volatile unsigned char g_ucPS2ExpectAck = 0;  // are we expecting an ACK?

// extraction of the three byte movement message
volatile unsigned char g_ucMovSeq = 0;
volatile unsigned char g_ucMovHeader = 0;
volatile unsigned char g_ucDeltaX = 0;
volatile unsigned char g_ucDeltaY = 0;

unsigned char g_ucParity;
unsigned char g_ucTXParity;

unsigned char g_ucRXDataBitCount;
unsigned char g_ucTXDataBitCount;




//*****************************************************************************
//
// Inc the tick timer.
//
//*****************************************************************************
void SysTick_Handler(void)
{
	sysTickUptime++;
}

//*****************************************************************************
//
// Init the tick timer.
//
//*****************************************************************************
void timerInit()
{
	SysTickPeriodSet(SysCtlClockGet()/1000); //set to 1kHZ
	SysTickIntRegister(SysTick_Handler);
	SysTickIntEnable();
	SysTickEnable();
}


//*****************************************************************************
//
// time since time in microseconds
//
//*****************************************************************************
unsigned long micros()
{
	register long ms, cycle_cnt;
	do {
		ms = sysTickUptime;
		cycle_cnt = SysTickValueGet();
	} while (ms != sysTickUptime);
	return (ms * 1000) + (80000 - cycle_cnt) / 80;
}


//*****************************************************************************
//
// return uptime.
//
//*****************************************************************************
unsigned long millis()
{
	return sysTickUptime;
}

//*****************************************************************************
//
// Delay in Microseconds.
//
//*****************************************************************************
void delayMicroseconds(unsigned int us)
{
	long now = micros();
	while (micros() - now < us);
}


//*****************************************************************************
//
// Delay in Milliseconds.
//
//*****************************************************************************
void delay(long milliseconds)
{
	unsigned long i;
	for(i=0; i<milliseconds; i++){
		delayMicroseconds(1000);
	}
}

// read ps/2 data pin.
char PS2ReadDat(void)
{
	if(GPIOPinRead(PS2_BASE, DAT_PIN) == DAT_PIN)
	{
		return(1);
	}
	else
	{
		return(0);
	}
}


// check the parity.
char getParity(unsigned n)
{
	char parity = 0;
	while (n)
	{
		parity = !parity;
		n = n & (n - 1);
	}
	return parity;
}

// send a byte from the host to the slave, in this case a PS/2 device.
void Ps2Send(unsigned char txb)
{

	// setup to send a byte to the slave PS/2 device
	g_ucPS2Mode = HOST_TO_SLAVE;
	g_ulTXCode = txb;
	g_ucTXDataBitCount = 0;
	g_ucPS2TXState = PS2_STATE_DATA;

	// disable interrupts on port a
	IntDisable(INT_GPIOA);
	// flip pins to outputs
	GPIOPinTypeGPIOOutput(PS2_BASE, CLK_PIN | DAT_PIN);
	// CLK low for 60us
	GPIOPinWrite(PS2_BASE, CLK_PIN, 0);
	delayMicroseconds(60);
	// data low
	GPIOPinWrite(PS2_BASE, DAT_PIN, 0);
	// clk high
	GPIOPinWrite(PS2_BASE, CLK_PIN, CLK_PIN);
	// clock input mode
	GPIOPinTypeGPIOInput(PS2_BASE, CLK_PIN);
	// re-enable interrupts
	IntEnable(INT_GPIOA);

	// now the interrupt routine will tx the data with the incoming clock, then flips data pin back to input.
}



void IntGPIOa(void)
{
	// clear interrupt
	GPIOPinIntClear(PS2_BASE, 0x10);
	// check which mode we are in, slave to host -or- host to slave
	switch(g_ucPS2Mode){

	//----------------------------------------------------------------------
	// slave is sending to the host (we are the host) (default)
	case SLAVE_TO_HOST:
	{
		// turn led on
		GPIO_PORTF_DATA_R |= GPIO_PIN_2;
		GPIO_PORTF_DATA_R &= ~(GPIO_PIN_1);

		switch(g_ucPS2RXState) {

		case PS2_STATE_IDLE: {
			// we should have a start bit here
			if(PS2ReadDat() == 0) {
				g_ucPS2RXState = PS2_STATE_DATA;
				g_ulRXCode = 0;
				g_ucParity = 0;
				g_ucRXDataBitCount = 0;
			} else {
				g_ucPS2RXState = PS2_STATE_IDLE;
			}
			break;
		}

		case PS2_STATE_DATA: {
			g_ulRXCode >>= 1;  // shift
			if(PS2ReadDat())  {  // read a bit
				g_ulRXCode |= 0x80; // set the bit
				g_ucParity++; // count highs
			}
			// last data bit
			if(++g_ucRXDataBitCount > 7) {
				g_ucPS2RXState = PS2_STATE_PARITY;
			}
			break;
		}

		case PS2_STATE_PARITY: {
			if((g_ucParity & 0x01) == PS2ReadDat()) {
				g_ucPS2RXState = PS2_STATE_IDLE;
			} else {
				g_ucPS2RXState = PS2_STATE_STOP;
			}
			break;
		} // parity

		case PS2_STATE_STOP: {
			// check for stop bit (inverted)
			if(PS2ReadDat()==0)
			{
				g_ucPS2RXState = PS2_STATE_IDLE;
			} else {
				g_ucPS2RXState = PS2_STATE_DONE;
			}
			break;
		}
		} // select state
	}

	//------------------------------------------------------------------
	// host is sending a command to the slave
	case HOST_TO_SLAVE:
	{
		// turn led on
		GPIO_PORTF_DATA_R |= GPIO_PIN_1;
		GPIO_PORTF_DATA_R &= ~(GPIO_PIN_2);

		switch(g_ucPS2TXState)  {
		case PS2_STATE_DATA: {
			// write data out, LSB first
			//g_ulTXCode >>= 1;
			if ( g_ucTXDataBitCount > 0) {
				if CHECK_BIT(g_ulTXCode, g_ucTXDataBitCount-1) {
					GPIOPinWrite(PS2_BASE, DAT_PIN, DAT_PIN);
				} else {
					GPIOPinWrite(PS2_BASE, DAT_PIN, 0);
				}
			} else {
				// not the start bit
				GPIOPinWrite(PS2_BASE, DAT_PIN, 0);  // start bit 0
			}
			if(++g_ucTXDataBitCount == 9) {
				g_ucPS2TXState = PS2_STATE_PARITY;
			}
			break;
		}

		case PS2_STATE_PARITY: {
			if (!getParity(g_ulTXCode)) {
				GPIOPinWrite(PS2_BASE, DAT_PIN, DAT_PIN);
			} else {
				GPIOPinWrite(PS2_BASE, DAT_PIN, 0);
			}
			g_ucPS2TXState = PS2_STATE_STOP;
			break;
		} // parity

		case PS2_STATE_STOP: {
			GPIOPinTypeGPIOInput(PS2_BASE, DAT_PIN);
			g_ucPS2TXState = PS2_STATE_DONE;
		}  // stop bit

		case PS2_STATE_DONE: {
			g_ucPS2TXState = PS2_STATE_IDLE;
			g_ucPS2ExpectAck = 1;
			g_ucPS2Mode = SLAVE_TO_HOST;
		}
		} // ps/2 tx state

	} // host to slave

	} // select transmission mode

}// interrupt handler



// initialize the ps/2 pins
void PS2Init(void) {
	IntPrioritySet(INT_GPIOA, 0x00);
	GPIOIntTypeSet(PS2_BASE, CLK_PIN, GPIO_FALLING_EDGE);
	GPIOPortIntRegister(PS2_BASE, &IntGPIOa);
	GPIOPinIntEnable(PS2_BASE, CLK_PIN);
	IntEnable(INT_GPIOA);
}

inline static void setup()
{
	// use raw 16mhz clock  (plenty more available...)
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |	SYSCTL_XTAL_16MHZ);

	// enable port for the leds
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
	volatile unsigned long ulLoop = SYSCTL_RCGC2_R;
	GPIO_PORTF_DIR_R = GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1;
	GPIO_PORTF_DEN_R = GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1;

	// turn led on red led
	GPIO_PORTF_DATA_R |= GPIO_PIN_1;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// setup the uptimer
	timerInit();

	// setup ps/2 pins
	GPIOPinTypeGPIOInput(PS2_BASE, CLK_PIN | DAT_PIN);

	// setup uart
	GPIOPinTypeUART(PS2_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	UARTCharPut(UART0_BASE, 0xFE);

	// Setup Buttons
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, BUTTON_1);
	GPIOPadConfigSet(GPIO_PORTF_BASE, BUTTON_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// interrupts on
	IntMasterEnable();
	// set interrupt on the ps/2 clock line
	PS2Init();
}

inline static void processPS2()
{
	// have we recieved a byte from the slave?
	if (g_ucPS2RXState == PS2_STATE_DONE)
	{

		switch (g_ucPS2SlaveState) {
		case SLAVE_DATA_EXPECTED:
		{
			switch (g_ucMovSeq){

			case 0: {
				g_ucMovHeader = g_ulRXCode;
				// check if this could be the first packet, whose 3rd bit is always 1
				if ( CHECK_BIT(g_ulRXCode,3) != 0) { g_ucMovSeq++; }
				break;
			}
			case 1: {
				g_ucDeltaX = g_ulRXCode;
				g_ucMovSeq++;
				break;
			}
			case 2: {
				g_ucDeltaY = g_ulRXCode;
				g_ucMovSeq = 0;
				UARTCharPut(UART0_BASE, 'm' );
				UARTCharPut(UART0_BASE, g_ucMovHeader );
				UARTCharPut(UART0_BASE, g_ucDeltaX );
				UARTCharPut(UART0_BASE, g_ucDeltaY );
				break;
			}
			}
			break;
		}// slave data expected

		case SLAVE_ACK_EXPECTED:
		{
			g_ucMovSeq = 0;
			if (g_ulRXCode == 0xAA) {
				UARTCharPut(UART0_BASE, 0xAA);

				// turn off led
				GPIO_PORTF_DATA_R &= ~(GPIO_PIN_1);
				Ps2Send(0xF2);
				g_ucPS2SlaveState = SLAVE_INITIALIZED;
			} else {
				UARTCharPut(UART0_BASE, 0xFE);  // reset has failed
			}
			break;
		}// slave ack expected

		case SLAVE_INITIALIZED:
		{
			g_ucMovSeq = 0;
			if (g_ulRXCode == 0x00) {
				Ps2Send(0xF4);
				g_ucPS2SlaveState = SLAVE_DATA_EXPECTED;
			} else {
				UARTCharPut(UART0_BASE, 0xF4 );
			}
			break;
		}// slave ack expected

		} // slave state switch
		g_ucPS2RXState = PS2_STATE_IDLE; // get ready for the next char


		// turn off the LEDs
		GPIO_PORTF_DATA_R &= ~(GPIO_PIN_2);
		GPIO_PORTF_DATA_R &= ~(GPIO_PIN_1);
	}
}

// initialize and start looping
int main(void)
{
	setup();

	int iCmd;
	unsigned long g_goMillis = 0;

	// superloop
	for (;;)
	{

		if (GPIOPinRead(GPIO_PORTF_BASE, BUTTON_1) == 0)
		{
			if (g_goMillis == 0)
				UARTCharPut(UART0_BASE, 'g');
			g_goMillis = millis();
		}
		else
		{
			if ((millis() - g_goMillis) > DEBOUNCE)
				g_goMillis = 0;
		}

		// command from the serial port
		if(UARTCharsAvail(UART0_BASE))
		{
			iCmd = UARTCharGet(UART0_BASE);

			if(iCmd == 'r') {
				Ps2Send(0xFF); // reset!
				g_ucPS2SlaveState = SLAVE_ACK_EXPECTED;
			}

			if(iCmd == 'e') {  // enable!
				Ps2Send(0xF4);
				g_ucPS2SlaveState = SLAVE_DATA_EXPECTED;
			}
		}

		processPS2();
	}
	return 0;
}
