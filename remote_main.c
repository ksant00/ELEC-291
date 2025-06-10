#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
// x 3.28 - 1.615 | 1.60 - 0
// y 0 - 1.675    | 1.69 - 3.28
#define ratioNW 1.674/1.675 // ratio to normalise y (x / y)
#define ratioNE 1.6/1.675
#define ratioSW 1.674/1.596
#define ratioSE 1.6/1.596

#define SYSCLK      72000000L  // SYSCLK frequency in Hz
#define BAUDRATE      115200L  // Baud rate of UART in bps
#define SARCLK 		18000000L  // SARCLK in Hz (fastest setting)
#define VDD 3.289 // The measured value of VDD in volts

#define LCD_RS P1_7
// #define LCD_RW Px_x // Not used in this code.  Connect to GND
#define LCD_E  P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

#define buzzer P1_6
#define low_buzz 4096L

#define base_change P0_6
#define lock_rem P0_2

xdata float hyp_maxNW = 2.368100716;
xdata float hyp_maxNE = 2.316381877;
xdata float hyp_maxSW = 2.318424465;
xdata float hyp_maxSE = 2.265571892;

xdata char buffstr[30];
xdata float hyp;

xdata volatile unsigned int overflow_count;

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
	
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	P2MDOUT |= 0b_0000_0010; // P2.1 in push-pull mode
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

	//Configure timer2 for buzzer
	P1MDOUT |= 0b_0100_0000;
	TMR2CN0 = 0x00;
	CKCON0 |= 0b_0001_0000;
	TMR2RL = 65536L - (SYSCLK/(low_buzz));
	TMR2 = 0x0000;
	ET2 = 1;
	TR2 = 0;

	EA = 1;

	P2MDIN |= 0b_0100_0000;
	P2MDOUT |= 0b_1011_0000;
	P2_6 = 1;

	return 0;
}

/*----------------------------------------------------------
	Default Timer3 setup fro micro-seconds delay
----------------------------------------------------------*/
// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(xdata unsigned char us)
{
	xdata unsigned char i;               // usec counter
	
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

void waitms (xdata unsigned int ms)
{
	xdata unsigned int j;
	xdata unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}


/*----------------------------------------------------------
	Timer2 for delay
----------------------------------------------------------*/
void timer2_isr(void) interrupt INTERRUPT_TIMER2
{
	TF2H = 0; // clear timer2 interrupt
	buzzer =! buzzer;
}

/*----------------------------------------------------------
	UART1 Initialization for radio
	UART1 Functions for radio
----------------------------------------------------------*/
void UART1_Init (xdata unsigned long baudrate)
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

void putchar1 (xdata char c) 
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
	xdata char c;
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
	xdata char c;
	xdata unsigned int timeout;
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

void getstr1 (xdata char * s)
{
	xdata char c;
	
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
bit RXU1(void)
{
	bit mybit;
	SFRPAGE = 0x20;
	mybit = RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1(xdata unsigned int ms)
{
	xdata unsigned int j;
	xdata unsigned char k;
	for (j = 0; j < ms; j++)
	{
		for (k = 0; k < 4; k++)
		{
			if (RXU1())
				return;
			Timer3us(250);
		}
	}
}

void SendATCommand(char *s)
{
	printf("Command: %s", s);
	P2_1 = 0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buffstr);
	waitms(10);
	P2_1 = 1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buffstr);
}

/*----------------------------------------------------------
	LCD Functions
----------------------------------------------------------*/
void LCD_pulse (void)
{
	LCD_E=1;
	Timer3us(40);
	LCD_E=0;
}

void LCD_byte (xdata unsigned char x)
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

void WriteData (xdata unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (xdata unsigned char x)
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

void LCDprint(char *string, unsigned char line, bit clear)
{
	xdata int j;

	WriteCommand(line == 2 ? 0xc0 : 0x80);
	waitms(5);
	for (j = 0; string[j] != 0; j++)
		WriteData(string[j]); // Write the message
	if (clear)
		for (; j < CHARS_PER_LINE; j++)
			WriteData(' '); // Clear the rest of the line
}

void LCDprint2(char *string, xdata unsigned char line, xdata unsigned char col)
{
	xdata int j;

	WriteCommand(line == 2 ? 0xc0 | col : 0x80 | col); // Move cursor to line and column
	for (j = 0; string[j] != 0; j++)
		WriteData(string[j]); // Write the message
}

int getsn (xdata char * buff, xdata int len)
{
	xdata int j;
	xdata char c;
	
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

/*----------------------------------------------------------
	ADC Functions
----------------------------------------------------------*/

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

void InitPinADC (xdata unsigned char portno, xdata unsigned char pin_num)
{
	xdata unsigned char mask;
	
	mask=1<<pin_num;

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

unsigned int ADC_at_Pin(xdata unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(xdata unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/16383.0);
}

unsigned int Get_ADC(void)
{
	ADINT = 0;
	ADBUSY = 1;
	while (!ADINT); // Wait for conversion to complete 
	return (ADC0);
}

/*----------------------------------------------------------
	Joystick Functions
----------------------------------------------------------*/

float Get_X_volt(void)
{
	ADC0MX = QFP32_MUX_P1_4;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return((ADC0*VDD)/16383.0);
}

float Get_Y_volt(void)
{
	ADC0MX = QFP32_MUX_P1_5;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return((ADC0*VDD)/16383.0);
}

void Motor_PWM(xdata float x_volt, xdata float y_volt, xdata int *pwm_L, xdata int *pwm_R)
{
	xdata float tempx_W = x_volt - 1.615;
	xdata float tempy_N = 1.675 - y_volt;
	xdata float tempx_E = 1.6 - x_volt;
	xdata float tempy_S = y_volt - 1.693;

	xdata float xrange_W = 3.289 - 1.615;
	xdata float xrange_E = 1.6;
	xdata float yrange_S = 3.289 - 1.693;
	xdata float yrange_N = 1.675;
	hyp = 0;
	
	// robot not moving
	// reset place for joystick
	if ((x_volt <= 1.615 && x_volt >= 1.60) && (y_volt <= 1.69 && y_volt >= 1.675))
	{
		*pwm_L = 0;
		*pwm_R = 0;
		return;
	}

	// robot moving straight forward
	if ((x_volt <= 1.615 && x_volt >= 1.60) && (y_volt < 1.675))
	{
		*pwm_R = 100 * tempy_N/yrange_N;
		*pwm_L = *pwm_R;
		return;
	}

	// robot moving straight reverse
	if ((x_volt <= 1.615 && x_volt >= 1.60) && (y_volt > 1.69))
	{
		*pwm_R = -100 * tempy_S/yrange_S;
		*pwm_L = *pwm_R;
		return;
	}

	// robot moving turning left
	if ((y_volt <= 1.69 && y_volt >= 1.675) && (x_volt > 1.615))
	{
		*pwm_R = 100 * tempx_W/xrange_W;

		if (P3_3 == 0)
			*pwm_L = -1 * *pwm_R;
		else 
			*pwm_L = 0;

		return;
	}

	// robot turning right
	if ((y_volt <= 1.69 && y_volt >= 1.675) && (x_volt < 1.60))
	{
		*pwm_L = 100 * tempx_E/xrange_E;

		if (P3_3 == 0)
			*pwm_R = -1 * *pwm_L;
		else 
			*pwm_R = 0;

		return;
	}

	// robot forward left
	if (y_volt < 1.675 && x_volt > 1.615)
	{
		hyp = sqrtf(powf(tempx_W, 2) + powf(tempy_N, 2));

		*pwm_R = 100 * hyp / hyp_maxNW;
		*pwm_L = *pwm_R * sinf(atanf((tempy_N * ratioNW) / tempx_W));
		return;
	}

	// robot forward right
	if (y_volt < 1.675 && x_volt < 1.6)
	{
		hyp = sqrtf(powf(tempx_E, 2) + powf(tempy_N, 2));

		*pwm_L = 100 * hyp / hyp_maxNW;
		*pwm_R = *pwm_L * sinf(atanf((tempy_N * ratioNE) / tempx_E));
		return;
	}

	// robot reverse left
	if (y_volt > 1.693 && x_volt > 1.615)
	{
		hyp = sqrtf(powf(tempx_W, 2) + powf(tempy_S, 2));

		*pwm_R = -100 * hyp / hyp_maxSW;
		*pwm_L = *pwm_R * sinf(atanf((tempy_S * ratioSW / tempx_W)));
		return;
	}

	// robot reverse right
	if (y_volt > 1.693 && x_volt < 1.6)
	{
		hyp = sqrtf(powf(tempx_E, 2) + powf(tempy_S, 2));

		*pwm_L = -100 * hyp / hyp_maxSE;
		*pwm_R = *pwm_L * sinf(atanf((tempy_S * ratioSW / tempx_E)));
		return;
	}
}

/*----------------------------------------------------------
	Buzzer Functions
----------------------------------------------------------*/
void change_base_ind(xdata float* T_b, xdata float* T)
{
	if (base_change == 0)
		*T_b = *T;
}


void change_pitch(xdata float* T, xdata float* T_b, xdata int* level)
{
	if (*T + 10 >= *T_b)
	{
		TR2 = 0;
		*level = 0;
		return;
	}
	else if (*T + 10  <= *T_b && *T >= *T_b - 1 * 190)
	{
		TMR2RL = 65536L - (SYSCLK/(low_buzz));
		*level = 1;
		TR2 = 1;
		return;
	}
	else if (*T + 10  <= *T_b && *T >= *T_b - 2 * 190)
	{
		TMR2RL = 65536L - (SYSCLK/(low_buzz + 2*750L));
		*level = 2;
		TR2 = 1;
		return;
	}
	else if (*T + 10  <= *T_b && *T >= *T_b - 3 * 190)
	{
		TMR2RL = 65536L - (SYSCLK/(low_buzz + 3*750L));
		*level = 3;
		TR2 = 1;
		return;
	}
	else if (*T + 10  <= *T_b && *T >= *T_b - 4 * 190)
	{
		TMR2RL = 65536L - (SYSCLK/(low_buzz + 4*750L));
		*level = 4;
		TR2 = 1;
		return;
	}
	else
	{
		TMR2RL = 65536 - (SYSCLK/(low_buzz + 5*750L));
		*level = 5;
		TR2 = 1;
		return;
	}

}


/*----------------------------------------------------------
	MAIN Function
----------------------------------------------------------*/
void main (void)
{
	// REMOTE VAR
	xdata float x_voltage, y_voltage;
	xdata int pwm_L = 0;
	xdata int pwm_R = 0;
	xdata char lcd[17];
	xdata int i = 0;

	xdata int Lock = 1;
	xdata char Password[5];
	xdata char Pass[5];

	xdata int distance = 0;

	// ROBOT VAR
	xdata int level = 0;
	xdata float ind = 0;
	xdata float T_b = 0;
	xdata float T = 0;

	sprintf(Password, "0000");

	InitPinADC(1, 4); // Configure P1.4 as analog input (High quality ADC)
	InitPinADC(1, 5); // Configure P1.5 as analog input (High quality ADC)
    InitADC();

	LCD_4BIT();

	waitms(500);
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	printf("Project 2 debug\n"
		   "File: %s\n"
		   "Compiled: %s, %s\n\n",
		   __FILE__, __DATE__, __TIME__);
	UART1_Init(9600);

	// To configure the device (shown here using default values).
	// For some changes to take effect, the JDY-40 needs to be power cycled.
	// Communication can only happen between devices with the
	// same RFID and DVID in the same channel.
	
	//SendATCommand("AT+BAUD4\r\n");
	//SendATCommand("AT+RFID8899\r\n");
	//SendATCommand("AT+DVID1122\r\n"); // Default device ID.
	//SendATCommand("AT+RFC001\r\n");
	//SendATCommand("AT+POWE9\r\n");
	//SendATCommand("AT+CLSSA0\r\n");
	
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDF439\r\n"); 
	SendATCommand("AT+RFIDF439\r\n"); 

	// To check configuration
    SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	
	while(1)
	{
		if (lock_rem == 0)
		{
			LCD_4BIT();
			Lock = 1;
		}

		while (Lock == 1)
		{
			sprintf(lcd, " *** LOCKED *** ");
			LCDprint(lcd, 1, 1);

			if (P2_2 == 0)
				sprintf(Pass, "%i%i%i%i", P3_3, P3_2, P3_1, P3_0);

			if (strcmp(Pass, Password) == 0)
			{
				Lock = 0;
				sprintf(Pass, " "); 
			}
			else
			{
				Lock = 1;
			}
		}

		while (P2_3 == 0)
		{
			sprintf(lcd, "CHANGE PASS");
			LCDprint(lcd, 1, 1);
			sprintf(lcd, "Pass: %i%i%i%i", P3_3, P3_2, P3_1, P3_0);
			LCDprint(lcd, 2, 1);
			if (P2_2 == 0)
				sprintf(Password, "%i%i%i%i", P3_3, P3_2, P3_1, P3_0);
		}

		while (P2_4 == 0)
		{
			sprintf(lcd, "ROOMBA SETTIGNS");
			LCDprint(lcd, 1, 1);
			sprintf(lcd, "Time: %is", distance);
			LCDprint(lcd, 2, 1);
			if (P3_0 == 0)
			{
				if (P3_3 == 0)
					distance++;
				else
					distance--;
				
				if (distance < 0)
					distance = 0;
			}
			waitms(250);
		}

		if (P3_3 == 0)
			P2_6 = 0;
		else
			P2_6 = 1;

		x_voltage = Get_X_volt();
		y_voltage = Get_Y_volt();
		Motor_PWM(x_voltage, y_voltage, &pwm_L, &pwm_R);
		// SXXXSXXX
		if (P2_5 == 0)
		{
			sprintf(buffstr, "%+08i+1\n", distance);
		}
		else
			sprintf(buffstr, "%+04i%+04i+0\n", pwm_L, pwm_R);

		sendstr1(buffstr);
		waitms_or_RI1(100);

		if (RXU1())
		{
			getstr1(buffstr);
			if (strlen(buffstr) == 8)
			{
				printf("%s\r\n", buffstr);
				T = atof(buffstr);
				ind = (powf((T) / (2 * 3.14159265358979), 2) / (110000000.0)) * 10.0;
			}
		}

		if (P3_0 == 1)
		{
			sprintf(lcd, "Induct: %.2fmH", ind);
			LCDprint(lcd, 1, 1);
			if (P2_5 == 1)
				sprintf(lcd, "Level: %i", level);
			else
				sprintf(lcd, "Level: %i ROOMBA", level);
		}
		else
		{
			sprintf(lcd, "Cur: %.6fmH", (powf((T) / (2 * 3.14159265358979), 2) / (110000000.0)) * 10.0);
			LCDprint(lcd, 1, 1);
			sprintf(lcd, "Ref: %.6fmH", (powf((T_b) / (2 * 3.14159265358979), 2) / (110000000.0)) * 10.0);
			LCDprint(lcd, 2, 1);
		}

		LCDprint(lcd, 2, 1);
		change_base_ind(&T_b, &T);
		change_pitch(&T, &T_b, &level);
	}
}