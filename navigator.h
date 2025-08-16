#ifndef _NAVIGATOR_H
#define _NAVIGATOR_H
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <controller.h>
#include <sys/time.h>
#include "./math.h"
#include "./gpio.h"
#include "./device.h"
#include <stdbool.h>
struct feedback_guandao_data
	{
		float ax;
		float ay;
		float az;
		float Gyro_x;
		float Gyro_y;
		float Gyro_z;
		float Roll;
		float Pitch;
		float Yaw;
		float Height;
		float GPSYaw;
		double longitude;
		double Lattitude;
		float GPSV;
	}feedback_guandao_data1;

struct navigator_data
{
	double GPSWeek;
	double socond_of_week;
	double yaw;
	double pitch;
	double roll;
	double latitude;
	double longitude;
	double altitude;
	double East_velocity;
	double North_velocity;
	double Virtical_velocity;
	double Baseline_length;
	double NSV1;
	double NSV2;
	double satellite_state;
	double angular_velocity;
	double ax;
	double Z_ACCEL_OUTPUT;
	double Y_ACCEL_OUTPUT;
	double X_ACCEL_OUTPUT;
	double Z_GYRO_OUTPUT;
	double Y_GYRO_OUTPUT;
	double X_GYRO_OUTPUT;

}navigator_data1;
void navigator();

char buff_navigator[1024];
struct timeval tv_navigator;
double navi_yaw_old;
double navi_North_velocity_old;
long double navi_time_old;
long double navi_time_new;
int first_analysis_flag;
//滑动平均  速度
double navi_velocity[50];
int navi_velocity_i;
double navi_velocity_sum;
double navi_velocity_speed;
//滑动平均 角速度
double navi_angular[50];
int navi_angular_i;
double navi_angular_sum;
double navi_angular_speed;
bool cheat_flag;
#endif
