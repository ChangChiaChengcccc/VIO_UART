#include "geometry_msgs/Point.h"
#ifndef __IMU_THREAD_H__
#define __IMU_THREAD_H__

#define IMU_SERIAL_MSG_SIZE 7
#define IMU_CHECKSUM_INIT_VAL 0
typedef struct {
	float mass;

	float thrust;

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;

	uint8_t buf[];

} imu_t ;

double calc_deviation(double*);

uint8_t generate_imu_checksum_byte(uint8_t *, int);

int imu_decode(uint8_t *);

void imu_buf_push(uint8_t);

int imu_thread_entry();

#endif
