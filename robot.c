#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 16000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define PWM_FREQ    100L
#define DUTY_CYCLE  50

void delayus(int len);


/////////////////////////////////////////////////
//              Code from James                //
/////////////////////////////////////////////////

void Init_pwm(void)
{
    // Configure output pins for PWM operation
    RPA0Rbits.RPA0R = 0x0005; // OC1 to pin RPA0 (pin 2 of DIP 28)
    RPA1Rbits.RPA1R = 0x0005; // OC2 to pin RPA1 (pin 3 of DIP 28)

    // Configure Output Compare modules for PWM operation
    OC1CON = 0x0006; // Configure OC1 in PWM mode
    OC2CON = 0x0006; // Configure OC2 in PWM mode


    // Configure Timer2 for PWM frequency
    T2CONbits.TCKPS = 0; // Set prescaler to 1:1
    PR2 = (SYSCLK / (PWM_FREQ * 1)) - 1; // Calculate period register value

    // Initial duty cycle for all PWM outputs
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);
    OC2RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);


    // Enable Timer2 and Output Compare modules
    T2CONbits.ON = 1; // Enable Timer2
    OC1CONbits.ON = 1; // Enable OC1
    OC2CONbits.ON = 1; // Enable OC2

}

void Set_pwm1(float val) // right motor control
{
    OC1RS = (PR2 + 1) * (val / 100);
}

void Set_pwm2(float val) // leff motor control
{
    OC2RS = (PR2 + 1) * (val / 100);
}


float mapJoystickToSpeed(float input) {
    // Map input range (0 - 5) to output range (-50 to 50)
    return (input - 1.65)/3.3*100;
}

void forward(float x_speed, float y_speed){
    float leftM = y_speed;
    float rightM = y_speed;
    LATBbits.LATB5 = 1;     
    LATAbits.LATA4 = 1;
    if (x_speed > 0){
        leftM = leftM + x_speed;
        printf("\rforward left: %f\n", leftM);
        printf("\rforward right: %f\n", rightM);
        Set_pwm1(100-rightM);
        Set_pwm2(100-leftM);
    }
    else {
    	x_speed = 0 - x_speed;
        rightM = rightM + x_speed;
        Set_pwm1(100-rightM);
        Set_pwm2(100-leftM);
        printf("\rforward left: %f\n", leftM);
        printf("\rfoward right: %f\n", rightM); 
    }
}

void reverse(float x_speed, float y_speed){
	y_speed = 0-y_speed;
    float leftM = y_speed;
    float rightM = y_speed;
    LATBbits.LATB5 = 0;      
    LATAbits.LATA4 = 0;    
    if (x_speed > 0){
        leftM = leftM + x_speed;
        Set_pwm1(rightM);
        Set_pwm2(leftM);
        printf("\rreverse left: %f\n", leftM);
        printf("\rreverse right: %f\n", rightM);        
    }
    else {
    	x_speed = 0-  x_speed;
        rightM = rightM + x_speed; 
        Set_pwm1(rightM);
        Set_pwm2(leftM);
        printf("\rreverse left: %f\n", leftM);
        printf("\rreverse right: %f\n", rightM);
    }
    
}


/////////////////////////////////////////////////
//              Code Originally                //
/////////////////////////////////////////////////

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

int SerialTransmit(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U2STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}


/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////

// TXD1 is in pin 26
// RXD1 is in pin 24

int UART1Configure(int desired_baud)
{
	int actual_baud;

    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
    // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
    // TRISx register must also be configured for input (set to �1�)."
    
    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13

    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));

    U1MODESET = 0x8000;     // enable UART1

    return actual_baud;
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout;
 
    while(num_char < max_size)
    {
        timeout = 0;
        while( !U1STAbits.URXDA)   // wait until data available in RX buffer
        {
        	delayus(100);

            timeout++;
            if(timeout > 100)      // if it takes too long to receive data, exit the function
            {
                *buffer = '\0';
                return num_char;
            } 
        }
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

// waitms in period.c
void delayms(int len)
{
	while(len--) wait_1ms();
}

// Use the core timer to wait for 1 us.
void wait_1us(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000000)) );
}

void delayus(int len)
{
	while(len--) wait_1us();
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}



////////////////////////////////////////////////////////// 
//             Functions from Period.c                  //
//////////////////////////////////////////////////////////

#define PIN_PERIOD (PORTB&(1<<6))
// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}



////////////////////////////////////////////////////////// 
//             Functions for reading buff               //
//////////////////////////////////////////////////////////

void egetstr1(char *str, char *buffer, int len) {
    int i;
    // for(i = len; i < 3; i++) {
    //     str[i] = buffer[i]; // Copy data
    //     printf("str[%d]: %c\r\n", i, str[i]);
    // }
    str[4] = '\0'; // Null-terminate the string

    for(i = len + 3; i < 6; i++) {
        str[i-3] = buffer[i]; // Copy data
        //printf("str2[%d]: %c\r\n", i, str[i-3]);
    }
    str[i] = '\0'; // Null-terminate the string
}

void egetstr2(char *str, char *buffer, int len) {
    int i;
    for(i = len; i < len + 3; i++) {
        str[i] = buffer[i]; // Copy data
        //printf("str[%d]: %c\r\n", i, str[i]);
    }
    str[i] = '\0'; // Null-terminate the string
}


void main(void)
{
	char buff[80];
    int cnt=0;
    
    // from Period.c
    long int count;
	float T, f;


	DDPCON = 0;
	CFGCON = 0;
    int freq = 20000;
    int xvalue, yvalue;
    xvalue = 0;
    yvalue = 0;
    char xbuff[4], ybuff[4];

    int flagCount = 0;
    int flag = 0;

    
  
    Init_pwm();
    Set_pwm2(0);
    Set_pwm1(0);
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600
 
    TRISBbits.TRISB5 = 0;
	TRISAbits.TRISA4 = 0; // Initialize RA4 as output
	LATBbits.LATB5 = 0;
	
	LATAbits.LATA4 = 0;
	float x_axis = 1.65;
	float y_axis = 1.65;
    float x_speed, y_speed; 
    
	delayms(500); // Give putty time to start before we send stuff.
    printf("JDY40 test program.\r\n");

	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB &= ~(1<<14);  // configure pin RB14 as output
	LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode

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

    ANSELB &= ~(1<<6); // Set RB6 as a digital I/O
    TRISB |= (1<<6);   // configure pin RB6 as input
    CNPUB |= (1<<6);   // Enable pull-up resistor for RB6
 	
	printf("\r\nPress and hold a push-button attached to RB6 (pin 15) to transmit.\r\n");
	
    f = 235;
	cnt=0;
	while(1)
	{
        // Receiving from Master
		if(U1STAbits.URXDA) // Something has arrived
		{
			SerialReceive1(buff, sizeof(buff)-1);
            flagCount++;
			if(strlen(buff)==7)
			{
				yvalue=atoi(&buff[0]);
				xvalue=atoi(&buff[4]);
				SerialTransmit(buff);       // this prints on slave putty
				SerialTransmit("\r\n");
				printf("(%03d, %03d)\r\n", yvalue, xvalue);

                x_axis = xvalue;
                y_axis = yvalue;
               
                x_axis = x_axis/100;
                y_axis = y_axis/100;
                
                x_speed = mapJoystickToSpeed(x_axis);
                y_speed = mapJoystickToSpeed(y_axis);
                printf("\rxspeed: %f\n", x_speed);
                printf("\ryspeed: %f\n", y_speed);
                if (y_speed > 0){
                    forward(x_speed, y_speed);
                }
                else {
                    reverse(x_speed, y_speed);
                }
			}
		}


        
        // Sending to Master
        if (flagCount >= 100){
            // obtaining frequency from robot
            count=GetPeriod(100);
            // // flag = 0;
            if(count>0)
            {
                T=(count*2.0)/(SYSCLK*100.0);
                f=1/T;
                printf("f=%.2fHz, Count=%ld        \r", f, count);
            }
            else
            {
                printf("NO SIGNAL                     \r");
            }
            fflush(stdout); // GCC peculiarities: need to flush stdout to get string out without a '\n'

            // sending frequency to master constantly
            sprintf(buff, "%f\r\n", f);
            //sprintf(buff, "count:%ld\r\n", count);
            SerialTransmit1(buff);      // this prints on master putty
            SerialTransmit(buff);       // this prints on slave putty
            printf(".");
            printf("\r\n");
            delayms(200);

            flagCount = 0;
	    }
    }
}
