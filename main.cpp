#include <msp430.h>

#include "HAL_PMM.h"

#include "msp430_uart.h"
#include "msp430_clock.h"
#include "msp430_i2c.h"
#include "msp430_interrupt.h"
#include "mpu9250.h"



/* Set up MSP430 peripherals. */
static inline void msp430PlatformInit(void)
{
	WDTCTL = WDTPW | WDTHOLD;
    SetVCore(2);
    msp430_clock_init(12000000L, 2);
    Serial.begin(115200);
    msp430_i2c_enable();
    msp430_int_init();
    mpu9250.init();
    __enable_interrupt();
}

void main()
{
	msp430PlatformInit();
    P8DIR |= BIT1;
    P8OUT |= BIT1;
	while(1)
	{
		mpu9250.update();
	}
}
