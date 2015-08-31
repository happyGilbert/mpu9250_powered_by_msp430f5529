/*
 * msp430_uart.cpp
 *
 *  Created on: 2015年8月25日
 *      Author: 李健芃
 */

#include "msp430_clock.h"

#include <msp430.h>

#include "msp430_uart.h"

#if (!(defined (__USE_USCI_A0_AS_SERIAL__) && defined (__USE_USCI_A1_AS_SERIAL__)))  //only one of A0 and A1 can be used.
#if ((defined (__MSP430_HAS_USCI_A0__) && defined (__USE_USCI_A0_AS_SERIAL__)) || (defined (__MSP430_HAS_USCI_A1__) && defined (__USE_USCI_A1_AS_SERIAL__)))

#ifdef __USE_USCI_A1_AS_SERIAL__

#define UCAxCTLW0     UCA1CTLW0
#define UCAxCTL0      UCA1CTL0
#define UCAxCTL1      UCA1CTL1
#define UCAxBRW       UCA1BRW
#define UCAxBR0       UCA1BR0
#define UCAxBR1       UCA1BR1
#define UCAxMCTL      UCA1MCTL
#define UCAxMCTLW     UCA1MCTLW
#define UCAxMCTLW_L   UCA1MCTLW_L
#define UCAxMCTLW_H   UCA1MCTLW_H
#define UCAxSTAT      UCA1STAT
#define UCAxRXBUF     UCA1RXBUF
#define UCAxTXBUF     UCA1TXBUF
#define UCAxABCTL     UCA1ABCTL
#define UCAxIRCTL     UCA1IRCTL
#define UCAxIRTCTL    UCA1IRTCTL
#define UCAxIRRCTL    UCA1IRRCTL
#define UCAxICTL      UCA1ICTL
#define UCAxIE        UCA1IE
#define UCAxIFG       UCA1IFG
#define UCAxIV        UCA1IV

#elif defined __USE_USCI_A0_AS_SERIAL__

#define UCAxCTLW0     UCA0CTLW0
#define UCAxCTL0      UCA0CTL0
#define UCAxCTL1      UCA0CTL1
#define UCAxBRW       UCA0BRW
#define UCAxBR0       UCA0BR0
#define UCAxBR1       UCA0BR1
#define UCAxMCTL      UCA0MCTL
#define UCAxMCTLW     UCA0MCTLW
#define UCAxMCTLW_L   UCA0MCTLW_L
#define UCAxMCTLW_H   UCA0MCTLW_H
#define UCAxSTAT      UCA0STAT
#define UCAxRXBUF     UCA0RXBUF
#define UCAxTXBUF     UCA0TXBUF
#define UCAxABCTL     UCA0ABCTL
#define UCAxIRCTL     UCA0IRCTL
#define UCAxIRTCTL    UCA0IRTCTL
#define UCAxIRRCTL    UCA0IRRCTL
#define UCAxICTL      UCA0ICTL
#define UCAxIE        UCA0IE
#define UCAxIFG       UCA0IFG
#define UCAxIV        UCA0IV

#endif

HardwareSerial Serial;


void HardwareSerial::store_char(unsigned char c, struct ringBuf *buffer)
{
	uint8_t i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != buffer->tail) {
		(buffer->buf[buffer->head]) = c;
		buffer->head = i;
	}
}


void HardwareSerial::begin(unsigned long baud)
{
	uint8_t mod;
	unsigned long divider;
	unsigned char oversampling;
	unsigned long _smclk;

	_rxBuf.head = 0;
	_rxBuf.tail = 0;
	_txBuf.head = 0;
	_txBuf.tail = 0;

#ifdef __USE_USCI_A0_AS_SERIAL__
	P3SEL |= BIT3 + BIT4;
#elif defined __USE_USCI_A1_AS_SERIAL__
	P4SEL |= BIT4 + BIT5;
#endif
	msp430_get_smclk_freq(&_smclk);
	if (_smclk/baud >= 48) {                                                // requires SMCLK for oversampling
		oversampling = 1;
	}
	else {
		oversampling= 0;
	}

	divider=(_smclk<<4)/baud;

	UCAxCTL1  = UCSWRST;
	UCAxCTL1 |= UCSSEL_2;
	UCAxCTL0  = 0;
	UCAxABCTL = 0;

	if(!oversampling) {
		mod = ((divider&0xF)+1)&0xE;                    // UCBRSx (bit 1-3)
		divider >>=4;
	} else {
		mod = ((divider&0xf8)+0x8)&0xf0;                // UCBRFx (bit 4-7)
		divider>>=8;
	}

	UCAxBR0 = divider;
	UCAxBR1 = divider>>8;
	UCAxMCTL = (unsigned char)((oversampling ? UCOS16:0) | mod);
	UCAxCTL1 &= ~UCSWRST;
	UCAxIE |= UCRXIE;
}

void HardwareSerial::end()
{
	// wait for transmission of outgoing data
	while (_txBuf.head != _txBuf.tail);

	_rxBuf.head = _rxBuf.tail;
}

uint8_t HardwareSerial::available(void)
{
	return (uint8_t)(SERIAL_BUFFER_SIZE + _rxBuf.head - _rxBuf.tail) % SERIAL_BUFFER_SIZE;
}

int HardwareSerial::peek(void)
{
	if (_rxBuf.head == _rxBuf.tail) {
		return -1;
	} else {
		return _rxBuf.buf[_rxBuf.tail];
	}
}

int HardwareSerial::read(void)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rxBuf.head == _rxBuf.tail) {
		return -1;
	} else {
		unsigned char c = _rxBuf.buf[_rxBuf.tail];
		_rxBuf.tail = (unsigned int)(_rxBuf.tail + 1) % SERIAL_BUFFER_SIZE;
		return c;
	}
}

void HardwareSerial::flush()
{
	while (_txBuf.head != _txBuf.tail);
}

uint8_t HardwareSerial::wirte(uint8_t *data, uint8_t length)
{
	uint8_t i = (_txBuf.head + 1) % SERIAL_BUFFER_SIZE;
	uint8_t len;

	// If the output buffer is full, there's nothing for it other than to
	// wait for the interrupt handler to empty it a bit
	// ???: return 0 here instead?
	while (i == _txBuf.tail);

	_txBuf.buf[_txBuf.head] = *data;
	_txBuf.head = i;
	UCAxIE |= UCTXIE;
	for(len = 0; len <length - 1; len++)
	{
		data++;
		i = (_txBuf.head + 1) % SERIAL_BUFFER_SIZE;
		while (i == _txBuf.tail);
		{
			UCAxIE |= UCTXIE;
		}
		_txBuf.buf[_txBuf.head] = *data;
		_txBuf.head = i;
	}
	return 1;
}

void HardwareSerial::uart_rx_isr(void)
{
	unsigned char c = UCAxRXBUF;
	HardwareSerial::store_char(c, &_rxBuf);
}

void HardwareSerial::uart_tx_isr(void)
{
	if (_txBuf.head == _txBuf.tail) {
		UCAxIE &= ~UCTXIE;
		UCAxIFG |= UCTXIFG;    // Set Flag again
		return;
	}

	unsigned char c = _txBuf.buf[_txBuf.tail];
	_txBuf.tail = (_txBuf.tail + 1) % SERIAL_BUFFER_SIZE;
	UCAxTXBUF = c;
}

#ifdef __USE_USCI_A0_AS_SERIAL__

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	 switch(UCA0IV)
	  {
	  case 0:break;                             // Vector 0 - no interrupt
	  case 2:                                   // Vector 2 - RXIFG
		Serial.uart_rx_isr();
	    break;
	  case 4:
		Serial.uart_tx_isr();
		break;                                  // Vector 4 - TXIFG
	  default: break;
	  }
}

#elif defined __USE_USCI_A1_AS_SERIAL__

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	 switch(UCA1IV)
	  {
	  case 0:break;                             // Vector 0 - no interrupt
	  case 2:                                   // Vector 2 - RXIFG
		  Serial.uart_rx_isr();
	    break;
	  case 4:
		  Serial.uart_tx_isr();
		break;                                  // Vector 4 - TXIFG
	  default: break;
	  }
}


extern "C" uint8_t sendData(uint8_t *dat, uint8_t length)
{
	return Serial.wirte(dat, length);
}

#endif
#endif


#endif






