#ifndef _uart_h_
#define _uart_h_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define buffer_size	20
#define baudrate 	38400


// Initialisieren, echo ein/ausschalten, gibt Zeiger auf Eingangsbuffer zurück
// void uart_init(uint8_t);
void uart_Init(uint8_t echo);

// einzelnes Zeichen senden
//void uart_WriteChar( char c );

// einzelnes Zeichen senden
int uart_WriteChar( char c, FILE *stream);


// Zeichenfolge senden
void uart_WriteString( char *s );

// Zeichenfolge + \r\n
void uart_WriteLine( char *s );

// Warte auf Zeichen
char uart_WaitChar(void);

// Lesen ob rx-buffer voll
uint8_t uart_GetBufferState(void);

// Gibt Zeiger auf Eingangsbuffer zurück
void uart_GetInput(char *s);


#endif
