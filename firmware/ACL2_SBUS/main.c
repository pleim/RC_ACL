/*
 * ACL (Anti Collision Light)
 *
 * Created: 13.01.2015 
 * Author: Peter
 *
 * Review 11/2019 for new SMD board (ATMEGA328)
 *
 */ 

/*	Hardware

	F_CPU	16Mhz
	
	PD	RX S-Bus Robbe/Futaba 100kBit invertiert
		25Bytes (ca. 3ms), Intervall 15ms
		1 Start, 8 Data (MSB first), 1 Parity (even), 2 Stop  
		1 Byte 0xF0
		22 Bytes data
		1 Byte flags
		1 Byte 0x00 ??
		
		
	PB1	OUT1	Positionslicht fix ein mit kan7 pos1
	PD3	OUT2	ACL Blitz 1 mit kan7 pos1
	PD4	OUT3	ACL Blitz 2 mit kan7 pos1
	PD5	OUT4	ACL Blitz rot mit kan7 pos1
	PD6	OUT5	Einziehlandescheinwerfer 1 ein mit kan8 pos1 + servo 1
	PD7	OUT6	Einziehlandescheinwerfer 2 ein mit kan8 pos2 + servo 2	
	PC0	OUT7	Landescheinwerfer vorne mit kan7 pos2
	PC1	OUT8	Reserve

	PB0	Servo1
	PB1	Servo2

	T2 as timer-interrupt
		Prescaler	16MHz / 1024 = 15625Hz (64µs)
		Timer2		15625Hz / 256 = 61Hz (16,384ms)
		Impulse/Zyklus gewünscht 100  (100 / 1350Hz = 74ms,  74 / 16,4 = 4,5)
		Intervall = 16,384ms * 5 = 81.92ms (0.08192s * 1350Hz = 110.6 Impulse)
*/
//#define F_CPU 16000000UL  -> in Project/Toolchain/Symbols

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <util/delay.h>

// I/O PB2, PD3..PD7, PC0..PC5
#define OUT1_init() DDRB |= (1<<PB2)	// OUT1 = LED PB2
#define	OUT1_on() PORTB |= (1<<PB2)
#define	OUT1_off() PORTB &= ~(1<<PB2)

#define OUT2_init() DDRD |= (1<<PD3)	// OUT2 PD3
#define	OUT2_on() PORTD |= (1<<PD3)
#define	OUT2_off() PORTD &= ~(1<<PD3)

#define OUT3_init() DDRD |= (1<<PD4)	// OUT3 PD4
#define	OUT3_on() PORTD |= (1<<PD4)
#define	OUT3_off() PORTD &= ~(1<<PD4)

#define OUT4_init() DDRD |= (1<<PD5)	// OUT4 PD5
#define	OUT4_on() PORTD |= (1<<PD5)
#define	OUT4_off() PORTD &= ~(1<<PD5)

#define OUT5_init() DDRD |= (1<<PD6)	// OUT5 PD6
#define	OUT5_on() PORTD |= (1<<PD6)
#define	OUT5_off() PORTD &= ~(1<<PD6)

#define OUT6_init() DDRD |= (1<<PD7)	// OUT6 PD7
#define	OUT6_on() PORTD |= (1<<PD7)
#define	OUT6_off() PORTD &= ~(1<<PD7)	

#define OUT7_init() DDRC |= (1<<PC0)	// OUT7 PC0
#define	OUT7_on() PORTC |= (1<<PC0)
#define	OUT7_off() PORTC &= ~(1<<PC0)

#define OUT8_init() DDRC |= (1<<PC1)	// OUT8 PC1
#define	OUT8_on() PORTC |= (1<<PC1)
#define	OUT8_off() PORTC &= ~(1<<PC1)

#define OUT9_init() DDRC |= (1<<PC2)	// OUT9 PC2
#define	OUT9_on() PORTC |= (1<<PC2)
#define	OUT9_off() PORTC &= ~(1<<PC2)

#define OUT10_init() DDRC |= (1<<PC3)	// OUT10 PC3
#define	OUT10_on() PORTC |= (1<<PC3)
#define	OUT10_off() PORTC &= ~(1<<PC3)

#define OUT11_init() DDRC |= (1<<PC4)	// OUT11 PC4
#define	OUT11_on() PORTC |= (1<<PC4)
#define	OUT11_off() PORTC &= ~(1<<PC4)

#define OUT12_init() DDRC |= (1<<PC5)	// OUT12 PC5
#define	OUT12_on() PORTC |= (1<<PC5)
#define	OUT12_off() PORTC &= ~(1<<PC5)

// Servo PB0, PB1
#define Servo1_init() DDRB |= (1<<PB0)
#define	Servo1_on() PORTB |= (1<<PB0)
#define	Servo1_off() PORTB &= ~(1<<PB0)

#define Servo2_init() DDRB |= (1<<PB1)
#define	Servo2_on() PORTB |= (1<<PB1)
#define	Servo2_off() PORTB &= ~(1<<PB1)


// Uart
#define timer_start() TCNT1 = 0; TCCR1B = 2
#define timer_stop() TCCR1B = 0	
#define uart_startbyte	0x0F

// Functions
void par_read();
void led_init();
void led_hello();
void uart_init();
void delay(uint16_t j);

// VAR
char uart_rx[25];
uint16_t channel[16];
uint16_t servo1, servo2;
uint16_t beacon_icnt1;		// Intervall 1 for beacons
uint16_t beacon_icnt2;		// Intervall 2 for beacons
uint16_t beacon_icnt3;		// Intervall 3 for beacons
volatile int8_t uart_pnt;
volatile int8_t i;

// Parameter
uint16_t par_thresh1;	// Lower Threshold 
uint16_t par_thresh2;	// Upper Threshold
uint16_t par_binterv1;	// Beacon Interval (F-Bus cycles)
uint16_t par_blenght1;	// Beacon Duration
uint16_t par_binterv2;	// Beacon Interval (F-Bus cycles)
uint16_t par_blenght2;	// Beacon Duration
uint16_t par_binterv3;	// Beacon Interval (F-Bus cycles)
uint16_t par_blenght3;	// Beacon Duration
uint16_t par_posmin1;	// Servo 1, Position Minimum [µs]
uint16_t par_posmax1;	// Servo 1, Position Maximum [µs]
uint16_t par_speed1;	// Servo 1, Speed
uint16_t par_posmin2;	// Servo 2, Position Minimum [µs]
uint16_t par_posmax2;	// Servo 2, Position Maximum [µs]
uint16_t par_speed2;	// Servo 2, Speed
uint16_t par_reverse;	// Servo 1/2, Bit0 reverse Servo 1, Bit1 reverse Servo 2 

int main(void)
{
	par_read();
	led_init();
	uart_init();
	
	// Timer for RX Interval
	TCCR1A = 0; 			// no output compare, mode 0
	TCCR1B = 2;				// prescale 8 - 16MHz/8 = 0.5µs, 2^16 * 0.5µs = 32.768ms
	OCR1A = 40000;			// 20ms
	TIMSK1 |= (1<<TOIE1)|	// Overflow interrupt
			 (1<<OCIE1A);	// Output compare A interrupt
		
	_delay_ms(500);
	led_hello();
		
	servo1 = par_posmin1;
	servo2 = par_posmin2;
	
	OUT8_on();
	OUT9_on();
	OUT10_on();
	OUT11_on();
			
    while(1)
    {
		//timer_start();
		while(uart_pnt<25){}		// Warte bis Packet empfangen oder Timeout
		uart_pnt = 0;	
														
		// Kanalzuordnung (Quelle: https://github.com/mikeshub/FUTABA_SBUS/blob/master/FUTABA_SBUS/FUTABA_SBUS.cpp)
		channel[0] = ((uart_rx[1]    |uart_rx[2] <<8) & 0x07FF);
		channel[1] = ((uart_rx[2]>>3 |uart_rx[3] <<5) & 0x07FF);
		channel[2] = ((uart_rx[3]>>6 |uart_rx[4] <<2) & 0x07FF);
		channel[3] = ((uart_rx[5]>>1 |uart_rx[6] <<7) & 0x07FF);
		channel[4] = ((uart_rx[6]>>4 |uart_rx[7] <<4) & 0x07FF);
		channel[5] = ((uart_rx[7]>>7 |uart_rx[8] <<1) & 0x07FF);
		channel[6] = ((uart_rx[9]>>2 |uart_rx[10]<<6) & 0x07FF);		
		channel[7] = ((uart_rx[10]>>5|uart_rx[11]<<3) & 0x07FF);
		channel[8] = ((uart_rx[12]   |uart_rx[13]<<8) & 0x07FF);
		
		// Digitale I/O's
		if(channel[6]>par_thresh1){
			OUT1_on();
		}
		else{
			OUT1_off();
		}

		if(channel[6]>par_thresh2){
			OUT7_on();
		}
		else{
			OUT7_off();
		}
				
		// Flash 1
		if(!beacon_icnt1--)
			beacon_icnt1 = par_binterv1;
		
		if((beacon_icnt1 < par_blenght1)&(channel[6]>par_thresh1))
			OUT2_on();
		else
			OUT2_off();
				
		// Flash 2
		if(!beacon_icnt2--)
			beacon_icnt2 = par_binterv2;

		if((beacon_icnt2 < par_blenght2)&(channel[6]>par_thresh1))
			OUT3_on();
		else
			OUT3_off();
								
		// Flash 3
		if(!beacon_icnt3--)
			beacon_icnt3 = par_binterv3;

		if((beacon_icnt3 < par_blenght3)&(channel[6]>par_thresh1))
			OUT4_on();
		else
			OUT4_off();						
		
		// ramp for Servo-speed		
		if((channel[7]>par_thresh1)^(par_reverse & 0x01)){
			if(servo1 < par_posmax1)
				servo1 += par_speed1;
			else{
				servo1 = par_posmax1;
			}
		}
		else{
			if(servo1 > par_posmin1)
				servo1 -= par_speed1;
			else
				servo1 = par_posmin1;
		}
		
		if(((servo1==par_posmax1)&((par_reverse & 0x01) == 0)) | ((servo1==par_posmin1)&((par_reverse & 0x01) != 0)))
			OUT5_on();
		else
			OUT5_off();

		
		if((channel[7]>par_thresh2)^((par_reverse & 0x02) != 00)){
			if(servo2 < par_posmax2)
				servo2 += par_speed2;
			else{
				servo2 = par_posmax2;
			}
		}
		else{
			if(servo2 > par_posmin2)
				servo2 -= par_speed2;
			else
				servo2 = par_posmin2;
		}
		
		if(((servo2==par_posmax2)&((par_reverse & 0x02) == 0)) | ((servo2==par_posmin2)&((par_reverse & 0x02) != 0)))
			OUT6_on();
		else
			OUT6_off();		
				
		// Pulses for servos
		//if(servo1 > par_posmin1)
		Servo1_on();
		delay(servo1);
		Servo1_off();
		
		//if(servo2 > par_posmin2)
		Servo2_on();
		delay(servo2);
		Servo2_off();		
    }
}


void uart_init()
{	
	UBRR0H = 0;		// 16.000.000 / 16*(9 + 1) = 100.000
	UBRR0L = 9;

	UCSR0A = 0;		// no U2X, MPCM

 	UCSR0B =
	 (1<<RXCIE0)|		// RX Complete Interrupt Enable
 	 (0<<TXCIE0)|		// TX Complete Interrupt Enable
 	 (0<<UDRIE0)|		// Data Register Empty Interrupt Enable
 	 (1<<RXEN0)|		// Receiver Enable
 	 (0<<TXEN0)|		// Transmitter Enable
 	 (0<<UCSZ02);		// Character Size

 	UCSR0C =
	 (0<<UMSEL01)|		// 0,0 = asynchronous usart
	 (0<<UMSEL00)|
 	 (1<<UPM01)|		// even parity (!)
	 (0<<UPM00)|
 	 (1<<USBS0)|		// Stop Bit Select
 	 (1<<UCSZ01)|		// Character Size
	 (1<<UCSZ00)|
 	 (0<<UCPOL0);		// Clock Polarity (synchronous mode)

	sei();
}

void led_init()
{
	OUT1_init();
	OUT2_init();
	OUT3_init();
	OUT4_init();
	OUT5_init();
	OUT6_init();
	OUT7_init();
	OUT8_init();
	OUT9_init();
	OUT10_init();
	OUT11_init();
	OUT12_init();
	Servo1_init();
	Servo2_init();	
}

void led_hello()
{
	while(uart_pnt<25){
		OUT1_on();_delay_ms(100);OUT1_off();_delay_ms(300);
		OUT2_on();_delay_ms(100);OUT2_off();_delay_ms(300);
		OUT3_on();_delay_ms(100);OUT3_off();_delay_ms(300);
		OUT4_on();_delay_ms(100);OUT4_off();_delay_ms(300);
		OUT5_on();_delay_ms(100);OUT5_off();_delay_ms(300);
		OUT6_on();_delay_ms(100);OUT6_off();_delay_ms(300);
		OUT7_on();_delay_ms(100);OUT7_off();_delay_ms(300);
		OUT8_on();_delay_ms(100);OUT8_off();_delay_ms(300);
		OUT9_on();_delay_ms(100);OUT9_off();_delay_ms(300);
		OUT10_on();_delay_ms(100);OUT10_off();_delay_ms(300);
		OUT11_on();_delay_ms(100);OUT11_off();_delay_ms(300);
		OUT12_on();_delay_ms(100);OUT12_off();_delay_ms(300);
		for (int i = 0; i<20; i++)
		{
			Servo1_on();
			_delay_us(1250);
			Servo1_off();
			Servo2_on();
			_delay_us(1750);
			Servo2_off();
			_delay_ms(15);
		}
	}
}


// Parameter from eeprom
void par_read()
{
	//par_thresh1 = eeprom_read_word((uint16_t *)0);
	//par_thresh2 = eeprom_read_word((uint16_t *)2);
	//par_binterv1 = eeprom_read_word((uint16_t *)4);
	//par_blenght1 = eeprom_read_word((uint16_t *)6);
	//par_binterv2 = eeprom_read_word((uint16_t *)8);
	//par_blenght2 = eeprom_read_word((uint16_t *)10);	
	//par_binterv3 = eeprom_read_word((uint16_t *)12);
	//par_blenght3 = eeprom_read_word((uint16_t *)14);
	//par_posmin1 = eeprom_read_word((uint16_t *)16);
	//par_posmax1 = eeprom_read_word((uint16_t *)18);
	//par_speed1 = eeprom_read_word((uint16_t *)20);
	//par_posmin2 = eeprom_read_word((uint16_t *)22);
	//par_posmax2 = eeprom_read_word((uint16_t *)24);
	//par_speed2 = eeprom_read_word((uint16_t *)26);
	//par_reverse= eeprom_read_word((uint16_t *)28);
	
	par_thresh1 = 768; // Lower Threshold
	par_thresh2 = 1280; // Upper Threshold
	par_binterv1 = 80; // Beacon Interval (F-Bus cycles)
	par_blenght1 = 2; // Beacon Duration
	par_binterv2 = 81; // Beacon Interval (F-Bus cycles)
	par_blenght2 = 2; // Beacon Duration
	par_binterv3 = 79; // Beacon Interval (F-Bus cycles)
	par_blenght3 = 10; // Beacon Duration
	par_posmin1 = 1090; // Servo 1, Position Minimum [µs]
	par_posmax1 = 1750; // Servo 1, Position Maximum [µs]
	par_speed1 = 10; // Servo 1, Speed
	par_posmin2 = 940; // Servo 2, Position Minimum [µs]
	par_posmax2 = 1875; // Servo 2, Position Maximum [µs]
	par_speed2 = 10; // Servo 2, Speed
	par_reverse= 0x00; // Servo 1/2, Bit0 reverse Servo 1, Bit1 reverse Servo 2 
}

// Delay in µs for servo pulse
void delay(uint16_t j)
{
	while(j>0){
		j--;
		asm volatile ("nop");asm volatile ("nop");asm volatile ("nop");asm volatile ("nop");
		asm volatile ("nop");asm volatile ("nop");asm volatile ("nop");asm volatile ("nop");
		asm volatile ("nop");asm volatile ("nop");
	}
}

// Timer 1 compare A interrupt
ISR (TIMER1_COMPA_vect)
{
	timer_stop();
}

// USART RX Interrupt
ISR (USART_RX_vect)
{
	char c = UDR0;
	if(TCNT1 > 10000){		// Timer > 5ms
		OUT2_on();
		if(c==uart_startbyte)		// Start byte
		{
			uart_pnt = 0;
			timer_start();
			OUT2_off();
		}
	}
	if(uart_pnt<25){
		uart_rx[uart_pnt++] = c;
	}
}