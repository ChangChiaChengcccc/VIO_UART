#include <iostream>
#include <upboard_ukf/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Inertia.h>
#include <std_msgs/Int32.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "upboard_ukf/serial.hpp"
#include "ros_thread.h"
#include "proj_conf.h"

using namespace std;

float ukf_force[3] = {0.0f};
float controller_force[3] = {0.0f};

mutex imu_mutex;
imu_t imu;
float imu_average[3] = {0,0,-9.8};
int N = 100;
double calc_deviation(float* x){
	double sum = 0;
	double average = 0;
	double result = IMU_CHECKSUM_INIT_VAL;
	for(int i = 0 ; i<N ; i++ ){
		sum += x[i];
	}
	average = sum / N ;
	sum =0;
	for(int i = 0 ; i<N ; i++){
		sum += pow((x[i]-average),2);
	}
	result = sqrt(sum/N);
	return result;
}

void send_force_to_imu_thread(float* output_force);

uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	static float x_array_uart[100];
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[2], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		printf("oh no garbage message!\n");
		return 1; //error detected
	}

	memcpy(&imu.mass, &buf[2], sizeof(float));
	return 0;
}

void imu_buf_push(uint8_t c)
{
	if(imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			imu.buf[i - 1] = imu.buf[i];
		}

		/* save new byte to the last array element */
		imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		imu.buf[imu.buf_pos] = c;
		imu.buf_pos++;
	}
}


int imu_thread_entry(){

	geometry_msgs::Inertia inertia;
	ros::NodeHandle n;

	ros::Publisher inertia_pub = n.advertise<geometry_msgs::Inertia>("/inertia", 5);
	char c;
	imu.buf_pos = 0;
	int count1 =0;
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c);
			/*
			if(c == '-')
				count1 = 1;
			else if(c == '+') {
				count1++;
				printf("count : %d\n",count1);}
			else
				count1++;

			*/

			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE - 1 ] == '+')
			{
				for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
					cout << "s";

				if(imu_decode(imu.buf)==0)
				{
					
					inertia.m = imu.mass;
					inertia_pub.publish(inertia);
				}
			}
		}
	}
}
