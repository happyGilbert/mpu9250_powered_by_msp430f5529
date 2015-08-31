/*
 * mpu9250.h
 *
 *  Created on: 2015Äê8ÔÂ29ÈÕ
 *      Author: jfanl
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char rx_new;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

class MPUCLASS{
public:
	MPUCLASS();
	void init(void);
	void update();
	void printSensor(uint8_t accel, uint8_t gyro, uint8_t compass,
		  uint8_t euler, uint8_t quat, uint8_t heading, uint8_t rot, uint8_t pedometer,
		  uint8_t linerAccel, uint8_t gravityVector);
	void lpAccelMode();
	void setSampleRate(unsigned short accel_gyro_rate_hz, unsigned short compass_rate_hz);
	void dmpEnable(uint8_t enable);
	/*@param[in]  mode    DMP_INT_GESTURE or DMP_INT_CONTINUOUS.*/
	void dmpInterruptMode(unsigned char mode);
	void motionInterruptEnable(uint8_t enable);
private:
	void read_from_mpl(void);
	void setup_sensor(void);
	int8_t loadMplState();
	int8_t saveMplState();
	void runSelfTest(void);
	struct hal_s hal;
};

extern MPUCLASS mpu9250;

#endif /* _MPU9250_H_ */
