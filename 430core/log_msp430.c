/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   log_msp430.c
 *      @brief  Logging facility for the TI MSP430.
 */

//#include <stdio.h>
#include <stdint.h>
//#include <stdlib.h>
#include <string.h>
//#include <stdarg.h>


//#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
//#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"

#include "msp430.h"
#include "packet.h"
//#include "log.h"
#include "log_msp430.h"

extern uint8_t sendData(uint8_t *dat, uint8_t length);

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

/**
 *  @brief      Prints a variable argument log message.
 *  USB output will be formatted as follows:\n
 *  packet[0]       = $\n
 *  packet[1]       = packet type (1: debug, 2: quat, 3: data)\n
 *  packet[2]       = \n for debug packets: log priority\n
 *                    for quaternion packets: unused\n
 *                    for data packets: packet content (accel, gyro, etc)\n
 *  packet[3-20]    = data\n
 *  packet[21]      = \\r\n
 *  packet[22]      = \\n
 *  @param[in]  priority    Log priority (based on Android).
 *  @param[in]  tag         File specific string.
 *  @param[in]  fmt         String of text with optional format tags.
 *
 *  @return     0 if successful.
 */
//int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
//{
//    va_list args;
//    int length, ii;
//    char buf[BUF_SIZE], this_length;
//    uint8_t out[PACKET_LENGTH];
//
//    /* This can be modified to exit for unsupported priorities. */
//    switch (priority) {
//    case MPL_LOG_UNKNOWN:
//    case MPL_LOG_DEFAULT:
//    case MPL_LOG_VERBOSE:
//    case MPL_LOG_DEBUG:
//    case MPL_LOG_INFO:
//    case MPL_LOG_WARN:
//    case MPL_LOG_ERROR:
//    case MPL_LOG_SILENT:
//        break;
//    default:
//        return 0;
//    }
//
//    va_start(args, fmt);
//
//    length = vsprintf(buf, fmt, args);
//    if (length <= 0) {
//        va_end(args);
//        return length;
//    }
//
//    memset(out, 0, PACKET_LENGTH);
//    out[0] = '$';
//    out[1] = PACKET_DEBUG;
//    out[2] = priority;
//    out[21] = '\r';
//    out[22] = '\n';
//    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
//#define min(a,b) ((a < b) ? a : b)
//        this_length = min(length-ii, PACKET_LENGTH-5);
//        memset(out+3, 0, 18);
//        memcpy(out+3, buf+ii, this_length);
//        sendData(out, PACKET_LENGTH);
//    }
//    va_end(args);
//
//    return 0;
//}

void eMPL_send_quat(long *quat)
{
	uint8_t out[PACKET_LENGTH];
    if (!quat)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (uint8_t)(quat[0] >> 24);      //Quat data: q30 format!
    out[4] = (uint8_t)(quat[0] >> 16);
    out[5] = (uint8_t)(quat[0] >> 8);
    out[6] = (uint8_t)quat[0];
    out[7] = (uint8_t)(quat[1] >> 24);
    out[8] = (uint8_t)(quat[1] >> 16);
    out[9] = (uint8_t)(quat[1] >> 8);
    out[10] = (uint8_t)quat[1];
    out[11] = (uint8_t)(quat[2] >> 24);
    out[12] = (uint8_t)(quat[2] >> 16);
    out[13] = (uint8_t)(quat[2] >> 8);
    out[14] = (uint8_t)quat[2];
    out[15] = (uint8_t)(quat[3] >> 24);
    out[16] = (uint8_t)(quat[3] >> 16);
    out[17] = (uint8_t)(quat[3] >> 8);
    out[18] = (uint8_t)quat[3];
    out[21] = '\r';
    out[22] = '\n';
    sendData(out, PACKET_LENGTH);
}

void eMPL_send_data(unsigned char type, long *data)
{
	uint8_t out[PACKET_LENGTH];
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    /* Two bytes per-element. */
    case PACKET_DATA_ROT:      //Have no enough byte for complete rotation matrix,
    	                       //but can round down the lowest bits in fractional part.
    	                       //So in here, rotation matrix format is form q30 to q14
        out[3] = (uint8_t)(data[0] >> 24);  //ROT data: q14 format!
        out[4] = (uint8_t)(data[0] >> 16);
        out[5] = (uint8_t)(data[1] >> 24);
        out[6] = (uint8_t)(data[1] >> 16);
        out[7] = (uint8_t)(data[2] >> 24);
        out[8] = (uint8_t)(data[2] >> 16);
        out[9] = (uint8_t)(data[3] >> 24);
        out[10] = (uint8_t)(data[3] >> 16);
        out[11] = (uint8_t)(data[4] >> 24);
        out[12] = (uint8_t)(data[4] >> 16);
        out[13] = (uint8_t)(data[5] >> 24);
        out[14] = (uint8_t)(data[5] >> 16);
        out[15] = (uint8_t)(data[6] >> 24);
        out[16] = (uint8_t)(data[6] >> 16);
        out[17] = (uint8_t)(data[7] >> 24);
        out[18] = (uint8_t)(data[7] >> 16);
        out[19] = (uint8_t)(data[8] >> 24);
        out[20] = (uint8_t)(data[8] >> 16);
        break;
    /* Four bytes per-element. */
    /* Four elements. */
    case PACKET_DATA_QUAT:                  //use eMPL_send_quat() function to send quat data,not here!.
    	break;
//        out[15] = (uint8_t)(data[3] >> 24);
//        out[16] = (uint8_t)(data[3] >> 16);
//        out[17] = (uint8_t)(data[3] >> 8);
//        out[18] = (uint8_t)data[3];
    /* Three elements. */
    case PACKET_DATA_ACCEL:
    	out[3] = (uint8_t)(data[0] >> 24);  //Accel data: q16 format!
    	out[4] = (uint8_t)(data[0] >> 16);
    	out[5] = (uint8_t)(data[0] >> 8);
    	out[6] = (uint8_t)data[0];
    	out[7] = (uint8_t)(data[1] >> 24);
    	out[8] = (uint8_t)(data[1] >> 16);
    	out[9] = (uint8_t)(data[1] >> 8);
    	out[10] = (uint8_t)data[1];
    	out[11] = (uint8_t)(data[2] >> 24);
    	out[12] = (uint8_t)(data[2] >> 16);
    	out[13] = (uint8_t)(data[2] >> 8);
    	out[14] = (uint8_t)data[2];
    	break;
    case PACKET_DATA_GYRO:
    	out[3] = (uint8_t)(data[0] >> 24);  //Gyro data: q16 format!
    	out[4] = (uint8_t)(data[0] >> 16);
    	out[5] = (uint8_t)(data[0] >> 8);
    	out[6] = (uint8_t)data[0];
    	out[7] = (uint8_t)(data[1] >> 24);
    	out[8] = (uint8_t)(data[1] >> 16);
    	out[9] = (uint8_t)(data[1] >> 8);
    	out[10] = (uint8_t)data[1];
    	out[11] = (uint8_t)(data[2] >> 24);
    	out[12] = (uint8_t)(data[2] >> 16);
    	out[13] = (uint8_t)(data[2] >> 8);
    	out[14] = (uint8_t)data[2];
    	break;
    case PACKET_DATA_COMPASS:
    	out[3] = (uint8_t)(data[0] >> 24);  //Compass data: q16 format!
    	out[4] = (uint8_t)(data[0] >> 16);
    	out[5] = (uint8_t)(data[0] >> 8);
    	out[6] = (uint8_t)data[0];
    	out[7] = (uint8_t)(data[1] >> 24);
    	out[8] = (uint8_t)(data[1] >> 16);
    	out[9] = (uint8_t)(data[1] >> 8);
    	out[10] = (uint8_t)data[1];
    	out[11] = (uint8_t)(data[2] >> 24);
    	out[12] = (uint8_t)(data[2] >> 16);
    	out[13] = (uint8_t)(data[2] >> 8);
    	out[14] = (uint8_t)data[2];
    	break;
    case PACKET_DATA_EULER:
        out[3] = (uint8_t)(data[0] >> 24);  //Euler data: q16 format!
        out[4] = (uint8_t)(data[0] >> 16);
        out[5] = (uint8_t)(data[0] >> 8);
        out[6] = (uint8_t)data[0];
        out[7] = (uint8_t)(data[1] >> 24);
        out[8] = (uint8_t)(data[1] >> 16);
        out[9] = (uint8_t)(data[1] >> 8);
        out[10] = (uint8_t)data[1];
        out[11] = (uint8_t)(data[2] >> 24);
        out[12] = (uint8_t)(data[2] >> 16);
        out[13] = (uint8_t)(data[2] >> 8);
        out[14] = (uint8_t)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (uint8_t)(data[0] >> 24);  //Heading data: q16 format!
        out[4] = (uint8_t)(data[0] >> 16);
        out[5] = (uint8_t)(data[0] >> 8);
        out[6] = (uint8_t)data[0];
        break;
    case PACKET_DATA_PEDOMETER:
        out[3] = (uint8_t)(data[0] >> 24);  //Number of steps
        out[4] = (uint8_t)(data[0] >> 16);
        out[5] = (uint8_t)(data[0] >> 8);
        out[6] = (uint8_t)data[0];
        out[7] = (uint8_t)(data[1] >> 24);  //Walk time in milliseconds.
        out[8] = (uint8_t)(data[1] >> 16);
        out[9] = (uint8_t)(data[1] >> 8);
        out[10] = (uint8_t)data[1];
    	break;
    default:
        return;
    }
    sendData(out, PACKET_LENGTH);
}

/**
 * @}
**/


