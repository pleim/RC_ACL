#include "UART.h"

#define baudreg ((F_CPU/(16UL*baudrate))-1)

volatile char rx[buffer_size];		// Eingangsbuffer
volatile uint8_t pnt;				// Index auf zuletzt empfangenes Zeichen
static 	 uint8_t uart_echo;			// Echo aus = 0, ein > 0

//
// Parameter setzen 
//
//void uart_init(uint8_t e)
void uart_Init(uint8_t e)
{
	UBRR0H = (unsigned char)(baudreg>>8);	// Baudrate setzen
	UBRR0L = (unsigned char)baudreg;


 	UCSR0A = 0;					// no U2X, MPCM

  	UCSR0B =(1<<RXCIE0)|	// Interrupt bei empfangenem Zeichen		
			(0<<TXCIE0)|	// Interrupt wenn Zeichen gesendet
			(0<<UDRIE0)|	// Interrupt wenn UDR Register leer
			(1<<RXEN0)|		// Empfänger einschalten
			(1<<TXEN0)|		// Sender einschalten
			(0<<UCSZ02);	// Zeichenlänge

  	UCSR0C =(0<<UMSEL01)|(0<<UMSEL00)|	// asynchronous uart
			(0<<  UPM01)|(0<<  UPM00)|	// no parity
			(0<<USBS0)|					// 1 stop bit
			(1<< UCSZ01)|(1<< UCSZ00)|	// 8 Bit
			(0<< UCPOL0);				// polarity (only for synchronous mode)
 	

	sei();
	uart_echo = e;
}

//
// einzelnes Zeichen senden
//
int uart_WriteChar( char c, FILE *stream)
{
  while( (UCSR0A & 1<<UDRE0) == 0 );
  UDR0 = c;
  return 0;
}

//
// Zeichenfolge senden
//
void uart_WriteString(char *s)
{
  while( *s )
    uart_WriteChar( *s++, 0);
}

//
// Zeichenfolge + \r\n
//
void uart_WriteLine( char *s )
{  
  uart_WriteString(s);
  uart_WriteChar(0x0D, 0);
  uart_WriteChar(0x0A, 0);
}

//
// Abfrage ob Buffer ein Packet vom Format '@...\r' enthält
//
uint8_t uart_GetBufferState(void)
{
  return (pnt==20);
}

//
// Rückgabe des Buffers
//
void uart_GetInput(char *s)
{
	strcpy(s, (char*)rx);
	pnt = 0;
}

//
// Auf einzelnes Zeichen warten
//
char uart_WaitChar(void)
{
	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

// Interruptroutine 	(15us)
//
//
SIGNAL (USART_RX_vect)
{
	char c = UDR0;
	//if((c=='\n')|(c=='\r'))
	if(c=='\n')
	{
		rx[pnt] = 0;
		pnt = buffer_size;
	}
	else if((pnt<buffer_size)&(c>31))	// noch Platz im Buffer und kein Steuercode
	{
		rx[pnt++] = c;
	}
	if(uart_echo)
	{
		while( (UCSR0A & 1<<UDRE0) == 0 );
  		UDR0 = c;
	}
}

