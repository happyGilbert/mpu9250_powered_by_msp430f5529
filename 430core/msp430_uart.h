/*
 * msp430_uart.h
 *
 *  Created on: 2015年8月24日
 *      Author: 李健芃
 */

#ifndef _MSP430_UART_H_
#define _MSP430_UART_H_

//#define __USE_USCI_A0_AS_SERIAL__
#define __USE_USCI_A1_AS_SERIAL__

#if (!(defined (__USE_USCI_A0_AS_SERIAL__) && defined (__USE_USCI_A1_AS_SERIAL__)))  //only one of A0 and A1 can be used.
#if ((defined (__MSP430_HAS_USCI_A0__) && defined (__USE_USCI_A0_AS_SERIAL__)) || (defined (__MSP430_HAS_USCI_A1__) && defined (__USE_USCI_A1_AS_SERIAL__)))


#include <inttypes.h>

#define SERIAL_BUFFER_SIZE 16
struct ringBuf{
	unsigned char buf[SERIAL_BUFFER_SIZE];
	volatile uint8_t head;
	volatile uint8_t tail;
};

class HardwareSerial
{
public:
	void begin(unsigned long);
	void end();
	uint8_t available(void);
	int peek(void);      //read data, but not delect for rx buffer.
	int read(void);
	void flush(void);
	uint8_t wirte(uint8_t *data, uint8_t length);

	void uart_rx_isr(void);
	void uart_tx_isr(void);

private:
	void store_char(unsigned char c, struct ringBuf *buffer);
	struct ringBuf _rxBuf;
	struct ringBuf _txBuf;
//	uint8_t _inWrite;

};

extern HardwareSerial Serial;


#endif
#endif

#endif /* _MSP430_UART_H_ */
