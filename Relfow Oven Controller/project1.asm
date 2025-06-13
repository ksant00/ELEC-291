;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;

$NOLIST
$MODN76E003
$LIST


;-------------------------------;
;          Clock Values         ;
;-------------------------------;
CLK               	EQU 16600000 ; Microcontroller system frequency in Hz
;---Timer1---;
BAUD              	EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     	EQU (0x100-(CLK/(16*BAUD)))
;---Timer3---;
TIMER3_RATE 		EQU 100      ; 100Hz or 10ms
TIMER3_RELOAD       EQU (65536-(CLK/(16*TIMER3_RATE))) ; Need to change timer 2 input divide to 16 in T2MOD
;---Timer0---;
TIMER0_RATE   		EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD 		EQU ((65536-(CLK/TIMER0_RATE)))
;---Timer2---;
TIMER2_RATE   		EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD 		EQU ((65536-(CLK/TIMER2_RATE)))

;---Oven_Power---;
PWM_OUT    			EQU P1.0 ; Logic 1=oven on
SOUND_OUT			EQU P1.6

;----------------------------------;
;          Reset vector            ;
;----------------------------------;
org 0x0000
    ljmp main

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
    ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector 
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
	
; Timer/Counter 2 overflow interrupt vector
org 0x002B
	reti

; Timer/Counter 3 overflow interrupt vector
org 0x0083
	ljmp Timer3_ISR

;--------------------;
;  Custom Registers  ;
;--------------------;
; In the 8051 we can define direct access variables starting at location 0x30 up to location 0x7F
dseg at 0x30
;---Regs for math32.inc---;
x:   				ds 4
y:   				ds 4
bcd: 				ds 5
bcd2:				ds 2
VREF_ADC: 			ds 2
compare: 			ds 1

;---FSM---;
state: 				ds 1
pwm:				ds 1
pwm_counter:		ds 1
seconds: 			ds 1
oven_temp: 			ds 2
buzzer:				ds 1

;---Parameters---;
soak_temp: 			ds 1
soak_time: 			ds 1
reflow_temp: 		ds 1
reflow_time: 		ds 1
;---Display---;
sec_display: 		ds 1
temp_junc:			ds 1

Count1ms:			ds 1


BSEG
;-------------------;
;     Bit Reg       ;
;-------------------;
mf: 				dbit 1 ; math32
reflow_process:		dbit 1 ; reflow process running or not
reflow_process_r:	dbit 1 ; reflow process ready skip
FSM_run: 			dbit 1 ; FSM running or not
s_flag: 			dbit 1 ; set to 1 every time a second has passed

; These five bit variables store the value of the pushbuttons after calling 'LCD_PB' below
PB0: 				dbit 1
PB1: 				dbit 1
PB2: 				dbit 1
PB3: 				dbit 1
PB4:			 	dbit 1

;-------------------;
;        LCD        ;
;-------------------;
cseg
; LCD SCREEN

; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

$NOLIST
$include(math32.inc)
$LIST

;-------------------------;
;   Strings and Values    ;
;-------------------------;
new_line:		db '\n', 0
dp_oven:		db 'To', 0
dp_junc:		db 'Tj', 0
dp_run:			db 'RUN', 0
dp_state:		db 'State', 0

;--------------------;
;    Timer Init      ;
;--------------------;

Init_All:
	; Set pins to bidrectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00

	; Using timer0 for speaker
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt


	; Timer3
	anl T3CON, #0b11110111 ; turn off timer 3
	mov RH3, #high(TIMER3_RELOAD) ; set high reload
	mov RL3, #low(TIMER3_RELOAD) ; set low reload
	orl T3CON, #0b00001100 ; enable timer 3
	orl EIE1, #0b00000010 ; enable timer 3 interrupt


	; Timer1 
	; Buad generator
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1


	; Using timer 2 for delay functions.  Initialize here:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	orl T2MOD, #0x80 ; Enable timer 2 autoreload
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
    setb TR2  ; Enable timer 2 


	mov EIPH1, #0x02

	setb ea

	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10000001 ; Activate AIN0, AIN5 and AIN7 analog inputs
	orl ADCCON1, #0x01 ; Enable ADC
	ret
;----------------;
;   Timer0 ISR   ;
;----------------;
Timer0_ISR:
	; Timer 0 doesn't have 16-bit auto-reload, so
	clr TR0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	setb TR0
	cpl SOUND_OUT ; Connect speaker the pin assigned to 'SOUND_OUT'!
	reti


;----------------;
;   Timer3 ISR   ;
;----------------;
Timer3_ISR:
	push psw
	push acc
	
	inc pwm_counter
	clr c
	mov a, pwm
	subb a, pwm_counter ; If pwm_counter <= pwm then c=1
	cpl c
	mov PWM_OUT, c
	
	mov a, pwm_counter
	cjne a, #100, Timer3_ISR_done
	mov pwm_counter, #0
	setb s_flag
	

	mov a, buzzer
	cjne a, #0, buzzer_out
	lcall oven_temp_update

inc_sec_check:
	mov a, state
	cjne a, #0, inc_sec ; ~state0 -> increment seconds
	sjmp Timer3_ISR_done ; state0 -> return

inc_sec:
	inc sec_display
	inc seconds

Timer3_ISR_done:
	pop acc
	pop psw
	reti

buzzer_out:
	dec buzzer
	cpl TR0
	sjmp inc_sec_check

;-------------;
; subroutines ;
;-------------;

; checks if FSM is running or not
PB0_checks:
	jb FSM_run, FSM_run_off
	setb FSM_run
	ret

FSM_run_off:
	clr FSM_run ; FSM on -> FSM off
	clr reflow_process ; clear reflow process start bit
	clr reflow_process_r ; clear reflow process ready bit
	mov state, #0
	ret ; return


;---logic for what happens when push button is pressed---;
PB1_checks:
	mov a, state
	cjne a, #0, reflow_start ; ~state0 -> exit
	mov a, soak_temp
	cjne a, #255, soak_temp_inc ; soak temp != 200 -> increment soak temp
	mov soak_temp, #0 ; soak temp == 200 -> change to 140
	ret

soak_temp_inc:
	inc soak_temp ; increment soak temp
	ret

reflow_start:
	setb reflow_process
	ret

PB2_checks:
	mov a, state
	cjne a, #0, PB2_exit ; ~state0 -> exit
	mov a, soak_time
	cjne a, #255, soak_time_inc ; soak time != 90 -> increment soak time
	mov soak_time, #0 ; soak time == 90 -> reset to 60
	ret

soak_time_inc:
	inc soak_time ; increment soak time
	ret

PB2_exit:
	ret

PB3_checks:
	mov a, state
	cjne a, #0, PB3_exit ; ~state0 -> exit
	mov a, reflow_temp
	cjne a, #255, reflow_temp_inc ; reflow temp != 249 -> increment reflow temp
	mov reflow_temp, #0 ; reflow temp == 249 -> reset to 230
	ret


reflow_temp_inc:
	inc reflow_temp ; increment reflow temp
	ret

PB3_exit:
	ret

PB4_checks:
	mov a, state
	cjne a, #0, PB4_exit ; ~state0 -> exit
	mov a, reflow_time
	cjne a, #255, reflow_time_inc ; reflow time != 60 -> increment reflow time
	mov reflow_time, #0 ; reflow time == 60 -> reset to 30
	ret

reflow_time_inc:
	inc reflow_time ; increment reflow time
	ret

PB4_exit:
	ret

Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret

wait_1ms:
	clr TR2
	clr TF2
	setb TR2
	jnb	TF2, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret

Display_format_off:
	;---Line1---;
	Set_Cursor(1,1)
	Send_Constant_String(#dp_oven)
	Display_char(#'=')
	Display_BCD(oven_temp+1)
	Display_BCD(oven_temp+0)
	Display_char(#'C')
	Display_char(#' ')
	Display_char(#' ')
	Send_Constant_String(#dp_junc)
	Display_char(#'=')
	Display_BCD(temp_junc)
	Display_char(#'C')

	
	;---Line2---;
	Set_Cursor(2,1)
	Display_char(#'S')
	mov a, soak_temp
	lcall SendToLCD
	Display_char(#',')
	mov a, soak_time
	lcall SendToLCD
	Display_char(#'R')
	mov a, reflow_temp
	lcall SendToLCD
	Display_char(#',')
	mov a, reflow_time
	lcall SendToLCD

	ret

Display_format_on:
	;---Line1---;
	Set_Cursor(1,1)
	Send_Constant_String(#dp_state)
	mov a, state
	lcall SendToLCD
	Display_char(#' ')
	Display_char(#' ')
	Send_Constant_String(#dp_oven)
	Display_BCD(oven_temp+1)
	Display_BCD(oven_temp+0)

	;---Line2---;
	Set_Cursor(2,1)
	Send_Constant_String(#dp_run)
	mov a, sec_display
	lcall SendToLCD
	Display_char(#',')
	mov a, seconds
	lcall SendToLCD
	Display_char(#' ')
	Display_char(#' ')
	Send_Constant_String(#dp_junc)
	Display_BCD(temp_junc)

	ret

; Send eight bit number via serial port, passed in ’a’.
SendToSerialPort:
	mov b, #100
	div ab
	orl a, #0x30 ; Convert hundreds to ASCII
	lcall putchar ; Send to PuTTY/Python/Matlab
	mov a, b ; Remainder is in register b
	mov b, #10
	div ab
	orl a, #0x30 ; Convert tens to ASCII
	lcall putchar ; Send to PuTTY/Python/Matlab
	mov a, b
	orl a, #0x30 ; Convert units to ASCII
	lcall putchar ; Send to PuTTY/Python/Matlab
	ret

; Eight bit number to display passed in ’a’.
; Sends result to LCD
SendToLCD:
	mov b, #100
	div ab
	orl a, #0x30 ; Convert hundreds to ASCII
	lcall ?WriteData ; Send to LCD
	mov a, b ; Remainder is in register b
	mov b, #10
	div ab
	orl a, #0x30 ; Convert tens to ASCII
	lcall ?WriteData; Send to LCD
	mov a, b
	orl a, #0x30 ; Convert units to ASCII
	lcall ?WriteData; Send to LCD
	ret

LCD_PB:
	; Set variables to 1: 'no push button pressed'
	setb PB0
	setb PB1
	setb PB2
	setb PB3
	setb PB4
	; The input pin used to check set to '1'
	setb P1.5
	
	; Check if any push button is pressed
	clr P0.0
	clr P0.1
	clr P0.2
	clr P0.3
	clr P1.3
	jb P1.5, LCD_PB_Done

	; Set the LCD data pins to logic 1
	setb P0.0
	setb P0.1
	setb P0.2
	setb P0.3
	setb P1.3
	
	; Check the push buttons one by one
	clr P1.3
	mov c, P1.5
	mov PB4, c
	setb P1.3

	clr P0.0
	mov c, P1.5
	mov PB3, c
	setb P0.0
	
	clr P0.1
	mov c, P1.5
	mov PB2, c
	setb P0.1
	
	clr P0.2
	mov c, P1.5
	mov PB1, c
	setb P0.2
	
	clr P0.3
	mov c, P1.5
	mov PB0, c
	setb P0.3

LCD_PB_Done:
	mov R2, #150
	lcall waitms	
	ret


oven_temp_update:
	; Read the 4.096V voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VREF_ADC+0, R0
	mov VREF_ADC+1, R1

	; Read the signal connected to AIN5
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x05 ; Select channel 5
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40960) ; The MEASURED LED voltage: 4.096V, with 4 decimal places
	lcall mul32
	; Retrive the ADC value
	mov y+0, VREF_ADC+0
	mov y+1, VREF_ADC+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32

	Load_y(100)
	lcall mul32
	Load_y(244)
	lcall div32
	Load_y(41)
	lcall div32

	lcall hex2bcd
	mov bcd2+1, bcd+1
	mov bcd2+0, bcd+0

	; Read the 4.096V voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VREF_ADC+0, R0
	mov VREF_ADC+1, R1

	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
  ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40960) ; The MEASURED LED voltage: 4.096V, with 4 decimal places
	lcall mul32
	; Retrive the ADC value
	mov y+0, VREF_ADC+0
	mov y+1, VREF_ADC+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32

	Load_y(27300)
	lcall sub32
	Load_y(100)
	lcall mul32
	lcall hex2bcd

	mov temp_junc, bcd+2
	mov a, bcd+2
	add a, bcd2+0
	da a
	mov oven_temp+0, a
	mov a, bcd2+1
	addc a, #0x00
	da a
	mov oven_temp+1, a
	send_bcd(oven_temp+1)
	send_bcd(oven_temp+0)
	mov DPTR, #new_line
	lcall SendString
	
	ret

SendString:
    clr A
    movc A, @A+DPTR
    jz SendStringDone
    lcall putchar
    inc DPTR
    sjmp SendString

SendStringDone:
    ret

oven_soak_convert:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a
	

	mov x+0, soak_temp
	lcall hex2bcd
	mov x+1,bcd+1
	mov x+0, bcd+0
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1
	
	pop psw
	pop acc
	ret
	
	
oven_reflow_convert:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a
	

	mov x+0, reflow_temp
	lcall hex2bcd
	mov x+1,bcd+1
	mov x+0, bcd+0
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1
	
	pop psw
	pop acc
	ret
	
oven_temp_convert:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a
	

	mov x+0, #60
	lcall hex2bcd
	mov x+1,bcd+1
	mov x+0, bcd+0
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1
	
	pop psw
	pop acc
	ret
	
abort_temp_convert:
	push acc
	push psw
	clr mf
	clr a
	
	mov x+0,a
	mov x+1,a
	mov x+2,a
	mov x+3,a
	
	mov y+0,a
	mov y+1,a
	mov y+2,a
	mov y+3,a

	mov x+0, #0x50
	
	mov y+0, oven_temp+0
	mov y+1, oven_temp+1

	pop psw
	pop acc
	ret



;----------------;
;      FSM       ;
;----------------;

main:
	mov sp, #0x7f
	lcall Init_All
	lcall LCD_4BIT

	;--initialisation--;
	mov state, #0
	mov seconds, #0
	mov sec_display, #0
	lcall Display_format_off
	mov soak_time, #60
	mov soak_temp, #140
	mov reflow_temp, #230
	mov reflow_time, #30
	mov buzzer, #0
	clr FSM_run
	setb reflow_process
	clr reflow_process_r

FSM1:
	lcall LCD_PB
	jnb PB0, PB0_pressed
	jnb PB1, PB1_pressed ; jump if PB1 pressed
	jnb PB2, PB2_pressed ; jump if PB2 pressed
	jnb PB3, PB3_pressed ; jump if PB3 pressed
	jnb PB4, PB4_pressed ; jump if PB4 pressed

	sjmp FSM_state_0 ; jump to finish if no PB pressed

PB0_pressed:
	lcall PB0_checks
	sjmp FSM_state_0

PB1_pressed:
	lcall PB1_checks 
	sjmp FSM_state_0

PB2_pressed:
	lcall PB2_checks
	sjmp FSM_state_0

PB3_pressed:
	lcall PB3_checks
	sjmp FSM_state_0

PB4_pressed:
	lcall PB4_checks
	sjmp FSM_state_0


;---State0---;
FSM_state_0:
	mov a, state 
	cjne a, #0, FSM_state_1 ; checks if state0 else move to state1
	jb FSM_run, FSM_state0_state1 ; jump to transition to state1
	mov pwm, #0 ; change power to 0%
	mov seconds, #0 ; reset internal clock
	mov sec_display, #0 ; reset display run time
	lcall Display_format_off
	ljmp FSM_done ; exit FSM

FSM_state0_state1:
	inc state ; inc state 0-->1
	mov pwm, #100 ; keep power at 0%
	mov seconds, #0 ; reset internal clock
	mov sec_display, #0 ; reset display run time
	lcall Display_format_on
	ljmp FSM_done ; exit FSM


;---State1---;
FSM_state_1:
	mov a, state
	cjne a, #1, FSM_state_2 ; checks if state1 else move to state2
	mov pwm, #100 ; move power to 100% 
	mov a, seconds
	subb a, #60 ; checks if time < 60
	jnc abort_check ; if time >= 60 -> check if abort
	
	lcall oven_soak_convert ; move soak temp in x, oven temp in y
	lcall x_gteq_y
	jnb mf, FSM_state1_state2 ; jump if soak temp < oven temp
	
	lcall Display_format_on ; soak temp >= oven temp -> stay state1
	ljmp FSM_done ; exit FSM

abort_check:
	lcall abort_temp_convert ; abort temp in x, oven temp in y
	lcall x_gt_y 
	jb mf, abort_process ; 50C > oven temp -> abort
	; 50C <= oven temp -> stay state1
	lcall oven_soak_convert ; move soak temp in x, oven temp in y
	lcall x_gteq_y
	jnb mf, FSM_state1_state2 ; jump if soak temp < oven temp
	lcall Display_format_on ; soak temp >= oven temp -> stay state1
	ljmp FSM_done ; exit FSM

abort_process:
	mov pwm, #0 ; change power to 0%
	mov state, #0 ; change state to 0
	mov buzzer, #4
	clr FSM_run ; stop FSM
	;clr reflow_process ; clear reflow process start bit
	clr reflow_process_r ; clear reflow process ready bit
	lcall Display_format_off
	ljmp FSM_done ; exit FSM

FSM_state1_state2:
	mov buzzer, #2
	inc state ; change state 1 -> 2
	mov pwm, #20
	mov seconds, #0
	lcall Display_format_on
	ljmp FSM_done ; exit FSM

;---State2---;
FSM_state_2:
	mov a, state
	cjne a, #2, FSM_state_3 ; check if state2 else jump to state3
	mov pwm, #20 ; change power to 20%
	jb reflow_process_r, FSM_state2_state3
	mov a, seconds
	cjne a, soak_time, soak_time_check ; internal clock != soak time -> jump to soak time check
	lcall Display_format_on
	ljmp FSM_done ; exit FSM

soak_time_check:
	jnc FSM_state2_state3_pre ; internal clock > soak time -> change state2 to state3
	lcall Display_format_on ; internal clock < soak time -> stay state2
	ljmp FSM_done ; exit FSM

FSM_state2_state3_pre:
	setb reflow_process_r
	mov buzzer, #2

FSM_state2_state3:
	jb reflow_process, reflow_start_state
	mov pwm, #20 ; keep power at 20%
	mov seconds, #0 ; reset internal clock
	lcall Display_format_on
	ljmp FSM_done ; exit FSM

reflow_start_state:
	mov pwm, #100
	mov seconds, #0
	inc state
	lcall Display_format_on
	ljmp FSM_done

;---State3---;
FSM_state_3:
	mov a, state
	cjne a, #3, FSM_state_4 ; check if state3 else jump to state4
	mov pwm, #100 ; change power to 100%
	
	lcall oven_reflow_convert ; reflow temp in x, oven temp in y
	lcall x_gteq_y
	jnb mf,FSM_state3_state4 ; jump if reflow temp < oven temp
	
	lcall Display_format_on ; reflow temp >= oven temp -> stay state3
	ljmp FSM_done ; exit FSM


FSM_state3_state4:
	mov buzzer, #2
	mov pwm, #20 ; change power to 20%
	mov seconds, #0 ; reset internal clock
	inc state ; increment state from state3 to state4
	lcall Display_format_on
	ljmp FSM_done ; exit FSM


;---State4---;
FSM_state_4:
	mov a, state
	cjne a, #4, FSM_state_5
	mov pwm, #20 ; keep power 20%
	mov a, seconds
	cjne a, reflow_time, reflow_time_check ; internal clock != reflow time -> check reflow time
	lcall Display_format_on ; internal clock == reflow time -> stay state4
	ljmp FSM_done ; exit FSM


reflow_time_check:
	jnc FSM_state4_state5 ; internal time > reflow time -> change to state5
	lcall Display_format_on ; internal time < reflow time -> stay state4
	ljmp FSM_done ; exit FSM

FSM_state4_state5:
	mov buzzer, #2
	mov pwm, #0 ; change power to 0%
	mov seconds, #0 ; reset internal clock
	inc state ; change state from state4 -> state5
	lcall Display_format_on
	ljmp FSM_done ; exit FSM


;---State5---;
FSM_state_5:
	mov a, state
	cjne a, #5, reset_bug
	mov pwm, #0 ; keep power at 0%
	
	lcall oven_temp_convert ; move 60C in x, oven temp in y
	lcall x_lteq_y
	jnb mf, FSM_state5_state0 ; jump if 60C > oven temp
	
	lcall Display_format_on ; 60C <= oven temp -> stay state 5
	ljmp FSM_done ; exit FSM

FSM_state5_state0:
	mov buzzer, #6
	mov state, #0 ; change state from state5 to state0
	mov pwm, #0 ; change power to 0%
	mov seconds, #0 ; reset internal clock
	clr FSM_run ; stop FSM
	;clr reflow_process ; clear reflow process start bit
	;clr reflow_process_r ; clear reflow process ready bit
	lcall Display_format_on
	ljmp FSM_done ; exit FSM

reset_bug:
	mov state, #0
	mov pwm, #0
	mov seconds, #0
	mov sec_display, #0
	lcall Display_format_off
	ljmp FSM_done

FSM_done:
	ljmp FSM1

	END
