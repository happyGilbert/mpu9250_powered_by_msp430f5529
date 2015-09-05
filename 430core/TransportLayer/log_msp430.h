/*
 * log_msp430.h
 *
 *  Created on: 2015Äê8ÔÂ27ÈÕ
 *      Author: jfanl
 */

#ifndef _LOG_MSP430_H_
#define _LOG_MSP430_H_

#ifdef __cplusplus
extern "C" {
#endif

//int _MLPrintLog (int priority, const char* tag, const char* fmt, ...);
void eMPL_send_quat(long *quat);
void eMPL_send_data(unsigned char type, long *data);

#ifdef __cplusplus
}
#endif

#endif /* 430CORE_LOG_MSP430_H_ */
