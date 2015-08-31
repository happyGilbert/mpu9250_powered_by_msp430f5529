/*
 * mpu9250.cpp
 *
 *  Created on: 2015Äê8ÔÂ28ÈÕ
 *      Author: jfanl
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <msp430.h>

#include "mpu9250.h"
#include "HAL_FLASH.h"
#include "log_msp430.h"
#include "msp430_interrupt.h"
#include "msp430_clock.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "packet.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
static struct platform_data_s accel_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
//        TODO: add code you wanted!
        break;
    case TAP_X_DOWN:
//        TODO: add code you wanted!
        break;
    case TAP_Y_UP:
//        TODO: add code you wanted!
        break;
    case TAP_Y_DOWN:
//        TODO: add code you wanted!
        break;
    case TAP_Z_UP:
//        TODO: add code you wanted!
        break;
    case TAP_Z_DOWN:
//        TODO: add code you wanted!
        break;
    default:
        break;
    }
//    TODO: also can add code according to the count!
}

static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {
	case ANDROID_ORIENT_PORTRAIT:
//        TODO: add code you wanted!
        break;
	case ANDROID_ORIENT_LANDSCAPE:
//        TODO: add code you wanted!
        break;
	case ANDROID_ORIENT_REVERSE_PORTRAIT:
//        TODO: add code you wanted!
        break;
	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
//        TODO: add code you wanted!
        break;
	default:
		return;
	}
}

static inline void msp430_reset(void)
{
    PMMCTL0 |= PMMSWPOR;
}

unsigned char rx_new;
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
//TODO: rename function name and rx_new
static void data_ready_cb(void)
{
    rx_new = 1;
}

MPUCLASS mpu9250;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";
MPUCLASS::MPUCLASS()
{
	memset(&hal, 0, sizeof(hal));
}


/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
void MPUCLASS::read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    if(hal.report & PRINT_QUAT){
    	if (inv_get_sensor_type_quat(data, &accuracy,
    		(inv_time_t*)&timestamp)) {
    		/* Sends a quaternion packet to the PC. Since this is used by the Python
    		 * test app to visually represent a 3D quaternion, it's sent each time
    	     * the MPL has new data.
    	     */
    		eMPL_send_quat(data);
    	}
    }
    if (hal.report & PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_ACCEL, data);
        }
    }
    if (hal.report & PRINT_GYRO) {
        if (inv_get_sensor_type_gyro(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_GYRO, data);
        }
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS) {
        if (inv_get_sensor_type_compass(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_COMPASS, data);
        }
    }
#endif
    if (hal.report & PRINT_EULER) {
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_EULER, data);
        }
    }
    if (hal.report & PRINT_ROT_MAT) {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_ROT, data);
        }
    }
    if (hal.report & PRINT_HEADING) {
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp)) {
            eMPL_send_data(PACKET_DATA_HEADING, data);
        }
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
//        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy,
//        	(inv_time_t*)&timestamp)) {
//        	TODO: add code you wanted!
//         }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
//        if (inv_get_sensor_type_gravity(float_data, &accuracy,
//            (inv_time_t*)&timestamp)){
//            	TODO: add code you wanted!
//        }
    }
    if (hal.report & PRINT_PEDO) {
        unsigned long timestamp;
        msp430_get_clock_ms(&timestamp);
        if (timestamp > hal.next_pedo_ms) {
        	unsigned long dat[2];
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            dmp_get_pedometer_step_count(&dat[0]);
            dmp_get_pedometer_walk_time(&dat[1]);
            eMPL_send_data(PACKET_DATA_PEDOMETER, (long *)dat);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
            INV_MSG_NO_MOTION_EVENT);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
//            TODO: add code you wanted!
        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
//            TODO: add code you wanted!
        }
    }
}

/* Handle sensor on/off combinations. */
void MPUCLASS::setup_sensor(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* Switching out of LP accel, notify MPL of new accel sampling rate. */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}

void MPUCLASS::printSensor(uint8_t accel, uint8_t gyro, uint8_t compass,
	  uint8_t euler, uint8_t quat, uint8_t heading, uint8_t rot, uint8_t pedometer,
	  uint8_t linerAccel, uint8_t gravityVector)
{
	if(accel)
	{
		if (!(hal.sensors & ACCEL_ON))
		{
			hal.sensors |= ACCEL_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_ACCEL;
	}
	else
	{
		hal.report &= ~PRINT_ACCEL;
	}
	if(gyro)
	{
		if (!(hal.sensors & GYRO_ON))
		{
			hal.sensors |= GYRO_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_GYRO;
	}
	else
	{
		hal.report &= ~PRINT_GYRO;
	}
#ifdef COMPASS_ENABLED
	if(compass)
	{
		if (!(hal.sensors & COMPASS_ON))
		{
			hal.sensors |= COMPASS_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_COMPASS;
	}
	else
	{
		hal.report &= ~PRINT_COMPASS;
	}
#endif
	if(euler)
	{
		if (!((hal.sensors & ACCEL_ON) && (hal.sensors & COMPASS_ON)
				&& (hal.sensors & GYRO_ON)))
		{
			hal.sensors |= ACCEL_ON + COMPASS_ON + GYRO_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_EULER;
	}
	else
	{
		hal.report &= ~PRINT_EULER;
	}
	if(quat)
	{
		if (!((hal.sensors & ACCEL_ON) && (hal.sensors & COMPASS_ON)
				&& (hal.sensors & GYRO_ON)))
		{
			hal.sensors |= ACCEL_ON + COMPASS_ON + GYRO_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_QUAT;
	}
	else
	{
		hal.report &= ~PRINT_QUAT;
	}
	if(heading)
	{
		if (!((hal.sensors & ACCEL_ON) && (hal.sensors & COMPASS_ON)
				&& (hal.sensors & GYRO_ON)))
		{
			hal.sensors |= ACCEL_ON + COMPASS_ON + GYRO_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_HEADING;
	}
	else
	{
		hal.report &= ~PRINT_HEADING;
	}
	if(rot)
	{
		if (!((hal.sensors & ACCEL_ON) && (hal.sensors & COMPASS_ON)
				&& (hal.sensors & GYRO_ON)))
		{
			hal.sensors |= ACCEL_ON + COMPASS_ON + GYRO_ON;
			MPUCLASS::setup_sensor();
		}
		hal.report |= PRINT_ROT_MAT;
	}
	else
	{
		hal.report &= ~PRINT_ROT_MAT;
	}
	if(pedometer)
	{
		hal.report |= PRINT_PEDO;
	}
	else
	{
		hal.report &= ~PRINT_PEDO;
	}
	if(linerAccel)
	{
//		TODO: add code you wanted!
	}
	else
	{
//		TODO: add code you wanted!
	}
	if(gravityVector)
	{
//		TODO: add code you wanted!
	}
	else
	{
//		TODO: add code you wanted!
	}
}

void MPUCLASS::lpAccelMode()
{
    if (hal.dmp_on)
        /* LP accel is not compatible with the DMP. */
        return;
//	TODO: change sample rate to you wanted!
    mpu_lp_accel_mode(20);
    /* When LP accel mode is enabled, the driver automatically configures
     * the hardware for latched interrupts. However, the MCU sometimes
     * misses the rising/falling edge, and the hal.new_gyro flag is never
     * set. To avoid getting locked in this state, we're overriding the
     * driver's configuration and sticking to unlatched interrupt mode.
     *
     * TODO: The MCU supports level-triggered interrupts.
     */
//    mpu_set_int_latched(0);
    mpu_set_int_latched(1);
    hal.sensors &= ~(GYRO_ON|COMPASS_ON);
    hal.sensors |= ACCEL_ON;
    hal.lp_accel_mode = 1;
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
    MPUCLASS::printSensor(1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

int8_t MPUCLASS::loadMplState()
{
	size_t store_size;
    inv_get_mpl_state_size(&store_size);
    if (store_size > FLASH_SIZE) {
    	return -1;
    }
    FCTL3 = FWKEY;
    if(inv_load_mpl_states((unsigned char *)FLASH_MEM_START, store_size))
    	return -1;
    FCTL3 = FWKEY + LOCK;
    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
    return 0;
}

int8_t MPUCLASS::saveMplState()
{
	size_t store_size;
    inv_get_mpl_state_size(&store_size);
    if (store_size > FLASH_SIZE) {
    	return -1;
    } else {
    	/*TODO: size of mpl_states must be modified if some modifies have done in storage function in storage_manager.c*/
    	/*Here, two storage function had been registered to deal with two type data which need save in flash!*/
    	/*Making these two type data to be a storage format form, then total size are 108 bytes, so the size of mpl_states is 110!*/
        unsigned char mpl_states[110] = {0}, tries = 5, erase_result;
        inv_save_mpl_states(mpl_states, store_size);
        while (tries--) {
            /* Multiple attempts to erase current data. */
            Flash_SegmentErase((uint16_t*)FLASH_MEM_START);
            erase_result = Flash_EraseCheck((uint16_t*)FLASH_MEM_START,
                store_size>>1);
            if (erase_result == FLASH_STATUS_OK)
                break;
        }
        if (erase_result == FLASH_STATUS_ERROR) {
        	return -1;
        }
        FlashWrite_8(mpl_states, (uint8_t *)FLASH_MEM_START, store_size);
    }
    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
    return 0;
}

void MPUCLASS::setSampleRate(unsigned short accel_gyro_rate_hz, unsigned short compass_rate_hz)
{
    if (hal.dmp_on) {
        dmp_set_fifo_rate(accel_gyro_rate_hz);
        inv_set_quat_sample_rate((long)(1000000L/accel_gyro_rate_hz));
    } else
    {
        mpu_set_sample_rate(accel_gyro_rate_hz);
    }
    inv_set_gyro_sample_rate((long)(1000000L/accel_gyro_rate_hz));
    inv_set_accel_sample_rate((long)(1000000L/accel_gyro_rate_hz));
    mpu_set_compass_sample_rate(compass_rate_hz); //Default: 10HZ
}

/*@param[in]  mode    DMP_INT_GESTURE or DMP_INT_CONTINUOUS.*/
void MPUCLASS::dmpInterruptMode(unsigned char mode)
{
	dmp_set_interrupt_mode(mode);
}

void MPUCLASS::dmpEnable(uint8_t enable)
{
    if (hal.lp_accel_mode)
        /* LP accel is not compatible with the DMP. */
        return;
    if (!enable) {
        unsigned short dmp_rate;
        unsigned char mask = 0;
        hal.dmp_on = 0;
        mpu_set_dmp_state(0);
        /* Restore FIFO settings. */
        if (hal.sensors & ACCEL_ON)
            mask |= INV_XYZ_ACCEL;
        if (hal.sensors & GYRO_ON)
            mask |= INV_XYZ_GYRO;
        if (hal.sensors & COMPASS_ON)
            mask |= INV_XYZ_COMPASS;
        mpu_configure_fifo(mask);
        /* When the DMP is used, the hardware sampling rate is fixed at
         * 200Hz, and the DMP is configured to downsample the FIFO output
         * using the function dmp_set_fifo_rate. However, when the DMP is
         * turned off, the sampling rate remains at 200Hz. This could be
         * handled in inv_mpu.c, but it would need to know that
         * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
         * put the extra logic in the application layer.
         */
        dmp_get_fifo_rate(&dmp_rate);
        mpu_set_sample_rate(dmp_rate);
        inv_quaternion_sensor_was_turned_off();
    } else {
        unsigned short sample_rate;
        hal.dmp_on = 1;
        /* Preserve current FIFO rate. */
        mpu_get_sample_rate(&sample_rate);
        dmp_set_fifo_rate(sample_rate);
        inv_set_quat_sample_rate(1000000L / sample_rate);
        mpu_set_dmp_state(1);
    }
}

void MPUCLASS::motionInterruptEnable(uint8_t enable)
{
	if(enable)
	{
		mpu_lp_motion_interrupt(500, 1, 5);
		hal.motion_int_mode = 1;
		/* Notify the MPL that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		inv_quaternion_sensor_was_turned_off();
		/* Wait for the MPU interrupt.
		 * TODO: enable motion interrupt, use LPM0 mode and wait interrupt!
         *while (!hal.new_gyro)
           __bis_SR_register(LPM0_bits + GIE);*/
	}
    else
    {
    	mpu_lp_motion_interrupt(0, 0, 0);
    	hal.motion_int_mode = 0;
	}
}

void MPUCLASS::runSelfTest(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

    	/* Don't use "USE_CAL_HW_REGISTERS", because use hardware registers, bias data can't build in
         * data_builder.c and we can't save bias to flash!*/
#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *=  2048.f; //convert to +-16G (bug fix from +-8G)
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short test_accel_sens = 32768/2;
    	float test_gyro_sens = 32768/250;

//		mpu_get_accel_sens(&accel_sens);
		accel[0] *= test_accel_sens;
		accel[1] *= test_accel_sens;
		accel[2] *= test_accel_sens;
		inv_set_accel_bias(accel, 3);
//		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * test_gyro_sens);
		gyro[1] = (long) (gyro[1] * test_gyro_sens);
		gyro[2] = (long) (gyro[2] * test_gyro_sens);
		inv_set_gyro_bias(gyro, 3);
		if(MPUCLASS::saveMplState() != 0)
		{
			/*TODO: save bias failed! Do something!*/
		}
#endif
    }
    else {
            if (!(result & 0x1))
            {
            	/*Gyro failed!*/
            }
            if (!(result & 0x2))
            {
            	/*Accel failed!*/
            }
            if (!(result & 0x4))
            {
            	/*Compass failed!*/
            }
    }
/*    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();*/
}

void MPUCLASS::init(void)
{
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned short compass_fsr;
#endif

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    int_param.cb = data_ready_cb;
    int_param.pin = INT_PIN_P20;
    int_param.lp_exit = INT_EXIT_LPM0;
    int_param.active_low = 1;
    result = mpu_init(&int_param);
    if (result) {
        msp430_reset();
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    result = inv_init_mpl();
    if (result) {
        msp430_reset();
    }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
    MPUCLASS::runSelfTest();
//    if(MPUCLASS::loadMplState() != 0)  /*load bias from flash, if failed, run self test to get new bias.*/
//    {
//    	MPUCLASS::runSelfTest();
//    }
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            /*Not authorize!*/
            msp430_delay_ms(5000);
        }
    }
    if (result) {
        /*Could not start the MPL!*/
        msp430_reset();
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(accel_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    if (dmp_load_motion_driver_firmware()) {
//            MPL_LOGE("Could not download DMP.\n");
            msp430_reset();
    }
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    inv_set_quat_sample_rate(1000000L / DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
   /* MPUCLASS::printSensor(1, 1, 1, 1, 1, 0, 0, 0, 0, 0);*/
    MPUCLASS::printSensor(0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
}

void MPUCLASS::update()
{
    unsigned long sensor_timestamp;
    unsigned long timestamp;
    unsigned char new_temp = 0;
    #ifdef COMPASS_ENABLED
        unsigned short new_compass = 0;
    #endif
    int new_data = 0;

    if(rx_new)
    {
    	msp430_get_clock_ms(&timestamp);
#ifdef COMPASS_ENABLED
	    /* We're not using a data ready interrupt for the compass, so we'll
		 * make our compass reads timer-based instead.
		 */
		if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode && (hal.sensors & COMPASS_ON)) {
			hal.next_compass_ms = timestamp + COMPASS_READ_MS;
			new_compass = 1;
		}
#endif
		/* Temperature data doesn't need to be read with every gyro sample.
		 * Let's make them timer-based like the compass reads.
		 */
		if (timestamp > hal.next_temp_ms) {
			hal.next_temp_ms = timestamp + TEMP_READ_MS;
			new_temp = 1;
		}

		if (hal.lp_accel_mode) {
			short accel_short[3];
			long accel[3];
			mpu_get_accel_reg(accel_short, &sensor_timestamp);
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
			rx_new = 0;
		} else if (hal.dmp_on) {
			short gyro[3], accel_short[3], sensors;
			unsigned char more;
			long accel[3], quat[4], temperature;
			/* This function gets new data from the FIFO when the DMP is in
			 * use. The FIFO can contain any combination of gyro, accel,
			 * quaternion, and gesture data. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
			 * the FIFO isn't being filled with accel data.
			 * The driver parses the gesture data to determine if a gesture
			 * event has occurred; on an event, the application will be notified
			 * via a callback (assuming that a callback function was properly
			 * registered). The more parameter is non-zero if there are
			 * leftover packets in the FIFO.
			 */
			dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
			if (!more)
				rx_new = 0;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
			if (sensors & INV_WXYZ_QUAT) {
				inv_build_quat(quat, 0, sensor_timestamp);
				new_data = 1;
			}
		} else {
			short gyro[3], accel_short[3];
			unsigned char sensors, more;
			long accel[3], temperature;
			/* This function gets new data from the FIFO. The FIFO can contain
			 * gyro, accel, both, or neither. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
			 * being filled with accel data. The more parameter is non-zero if
			 * there are leftover packets in the FIFO. The HAL can use this
			 * information to increase the frequency at which this function is
			 * called.
			 */
			rx_new = 0;
			mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
				&sensors, &more);
			if (more)
				rx_new = 1;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
		}
#ifdef COMPASS_ENABLED
		if (new_compass) {
			short compass_short[3];
			long compass[3];
			new_compass = 0;
			/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
			 * magnetometer registers are copied to special gyro registers.
			 */
			if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
				compass[0] = (long)compass_short[0];
				compass[1] = (long)compass_short[1];
				compass[2] = (long)compass_short[2];
				/* NOTE: If using a third-party compass calibration library,
				 * pass in the compass data in uT * 2^16 and set the second
				 * parameter to INV_CALIBRATED | acc, where acc is the
				 * accuracy from 0 to 3.
				 */
				inv_build_compass(compass, 0, sensor_timestamp);
			}
			new_data = 1;
		}
#endif
		if (new_data) {
			if(inv_execute_on_data()) {
               /* ERROR execute on data!*/
			}
			/* This function reads bias-compensated sensor data and sensor
			 * fusion outputs from the MPL. The outputs are formatted as seen
			 * in eMPL_outputs.c. This function only needs to be called at the
			 * rate requested by the host.
			 */
			MPUCLASS::read_from_mpl();
		}
	}
    else
    {
		__bis_SR_register(LPM0_bits + GIE);
    }
}
