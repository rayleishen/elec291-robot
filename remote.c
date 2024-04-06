#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define SYSCLK 72000000
#define BAUDRATE 115200L
#define DEFAULT_F 15500L

//unique to ADC
#define SARCLK 18000000L

//unique to LCD
#define LCD_RS P1_7
// #define LCD_RW Px_x // Not used in this code.  Connect to GND
#define LCD_E  P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

//buzzer
#define OUT4 P2_4

idata char buff[20];

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	//configure pin 2.4 used for square output (speaker)
	P2MDOUT |= 0b_0001_0000;
	
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	P2MDOUT |= 0x01; // P2.0 in push-pull mode
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x41; // Enable crossbar and uart 1

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
	// Initialize timer 2 for periodic interrupts
	TMR2CN0=0x00;   // Stop Timer2; Clear TF2;
	CKCON0|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*DEFAULT_F))); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2
	EA=1; // Global interrupt enable
  	
	return 0;
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

//ISR for speaker
void Timer2_ISR (void) interrupt INTERRUPT_TIMER2
{
	TF2H = 0; // Clear Timer2 interrupt flag
	OUT4=!OUT4;
	//OUT1=!OUT0;
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}


//LCD functions

void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the C8051Fxxx is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	Timer3us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	// LCD_RW=0; // We are only writing to the LCD in this program
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

int getsn (char * buff, int len)
{
	int j;
	char c;
	
	for(j=0; j<(len-1); j++)
	{
		c=getchar();
		if ( (c=='\n') || (c=='\r') )
		{
			buff[j]=0;
			return j;
		}
		else
		{
			buff[j]=c;
		}
	}
	buff[j]=0;
	return len;
}


//radio functions

void UART1_Init (unsigned long baudrate)
{
    SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 =0x00;   // disable baud rate generator
	SBRL1 = 0x10000L-((SYSCLK/baudrate)/(12L*2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1 (char c) 
{
    SFRPAGE = 0x20;
	while (!TI1);
	TI1=0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1 (char * s)
{
	while(*s)
	{
		putchar1(*s);
		s++;	
	}
}

char getchar1 (void)
{
	char c;
    SFRPAGE = 0x20;
	while (!RI1);
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

char getchar1_with_timeout (void)
{
	char c;
	unsigned int timeout;
    SFRPAGE = 0x20;
    timeout=0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if(timeout==25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1=0;
	// Clear Overrun and Parity error flags 
	SCON1&=0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1 (char * s)
{
	char c;
	
	while(1)
	{
		c=getchar1_with_timeout();
		if(c=='\n')
		{
			*s=0;
			return;
		}
		*s=c;
		s++;
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1 (void)
{
	bit mybit;
    	SFRPAGE = 0x20;
	mybit=RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++)
		{
			if(RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand (char * s)
{
	printf("Command: %s", s);
	P2_1=0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff);
	waitms(10);
	P2_1=1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

//ADC functions

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

//max voltage of EMF8 ADC
#define VDD 3.3035 // The measured value of VDD in volts
//#define VDD 5.0000 

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111); //binary is 8191, needed for VDD 
}


//misc custom functions
void data_sent(){
	putchar('(');
	putchar('^');
	putchar('_');
	putchar('^');
	putchar(')');
	putchar('/');
	putchar('\n');
}


void data_recieved(){
	putchar('\\');
	putchar('(');
	putchar('^');
	putchar('_');
	putchar('^');
	putchar(')');
	putchar('\n');
}


void main (void)
{
	char buff[17];
	
	//CANT DECLARE TOO MANY STRINGS
	//char remote_data[17]; //sends this
	//char robot_data[17]; //recieves this	
	//char buff[17];
	//char buff_lcd[17];
	
	unsigned int cnt;
	
	float joy_data[2];
	
	//f is frequency of speaker
	unsigned long int x, f;
	
	int j; //inductor freq
	float j2; //for lcd
	
	int j_no_metal;
	
	// Configure the LCD
	LCD_4BIT();
	// Configure JDY40
	UART1_Init(9600);
	// Configure ADC
	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitPinADC(2, 3); // Configure P2.3 as analog input
    	InitADC();
	
	//wait for serial port
	waitms(500);
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	
   	// Display something in the LCD
   	//canshow:0123456789ABCDEF
	LCDprint("Remote Controlle", 1, 1);
	LCDprint("Hello, World!   ", 2, 1);
	
	printf ("Remote Controller, Group B1\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDB1B1\r\n");  

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	printf("\r\Reset to transmit test message.\r\n");
	
	cnt=0;
	j=10000; //no noise 
	j_no_metal=13550; //
	
	TR2=0; // Stop timer 2
	OUT4=0;
	
	while(1)
	{
		// Read 14-bit value from the pins configured as analog inputs
		joy_data[0] = Volts_at_Pin(QFP32_MUX_P2_2);
		joy_data[1] = Volts_at_Pin(QFP32_MUX_P2_3);
		
		//printf ("V@P2.2=%7.5fV, V@P2.3=%7.5fV\n", joy_data[0], joy_data[1]);
			
		sprintf(buff, "%03d,%03d\n", (int)(joy_data[0] * 100), (int)(joy_data[1] * 100));
		
		printf("data: %s", buff);
		
		sendstr1(buff);
		data_sent();
		waitms_or_RI1(200);
		//waitms(50);

		
		if(RXU1())
		{
			getstr1(buff);
			data_recieved();

			//canshow:0123456789ABCDEF
			LCDprint("recieved data:  ", 1, 1);
			// LCDprint(buff, 2, 1);
			
			printf("recieved data: %s\n", buff);
			printf("strlen(buff): %d\n", strlen(buff));
				
			if (strlen(buff) == 5){
				//LCDprint(buff, 2, 1);
				
				j = atoi(buff);
				j2 = (float)j*10;
				sprintf(buff, "%f", j2);
				printf("j2 value(int): %f\n", j2);
				LCDprint(buff, 2, 1);
				
				printf("j value(int): %d\n", j);
			
				if(j<j_no_metal) //softer (no metal)
				{
					TR2=0; // Stop timer 2
					OUT4=0;
				}	
				else if(j>=j_no_metal) //loader (with metal)
				{
					f=1000;
					x=(SYSCLK/(2L*f));
					TR2=0; // Stop timer 2
					TMR2RL=0x10000L-x; // Change reload value for new frequency
					TR2=1; // Start timer 2
					f=SYSCLK/(2L*(0x10000L-TMR2RL));
					printf("speaker frequency: %lu\n", f);
				}
				else{
					printf("freq out of bounds\n");
				}
			}
			
			else{
				j = atoi(buff);
				printf("j value(int): %d\n", j);
				printf("incorrect freq length\n");
			}
			
		}
		
		
		//BUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONSBUTTONS
		
		//button joy
		if(P3_2==0){
			putchar('0');
			putchar('\n');
			
			//canshow:0123456789ABCDEF
			LCDprint("button joy      ", 1, 1);
			LCDprint("Hello, World!   ", 2, 1);
		}
		
		//button 0
		if(P2_5==0){
			putchar('1');
			putchar('\n');
			
			//canshow:0123456789ABCDEF
			LCDprint("button 1:       ", 1, 1);
			sprintf(buff, "new jnm: %d", j);
			LCDprint(buff, 2, 1);
			
			j_no_metal=j+500;
			
			
		}
		
		//button 2
		if(P2_6==0){
			putchar('2');
			putchar('\n');
			
			//canshow:0123456789ABCDEF
			LCDprint("button 2        ", 1, 1);
			LCDprint("Hello, World!   ", 2, 1);
		}
		
		//button 3
		if(P3_0==0){
			putchar('3');
			putchar('\n');
			
			//canshow:0123456789ABCDEF
			LCDprint("button 3        ", 1, 1);
			LCDprint("Hello, World!   ", 2, 1);
		}
		
		//button 4
		if(P3_1==0){
			putchar('4');
			putchar('\n');
			
			//canshow:0123456789ABCDEF
			LCDprint("button 4        ", 1, 1);
			LCDprint("Hello, World!   ", 2, 1);
		}
		
		//debug button
		if(P3_7==0)
		{
			//canshow:0123456789ABCDEF
			LCDprint("debug button hit", 1, 1);
			LCDprint("printin joy data", 2, 1);
			
			sprintf(buff, "Hi robot %d\r\n", cnt++);
			sendstr1(buff);
			
		    	printf ("V@P2.2=%7.5fV, V@P2.3=%7.5fV\n", joy_data[0], joy_data[1]);
		    	printf("current j (freq): %d\n", j);	
		}
	}
}
