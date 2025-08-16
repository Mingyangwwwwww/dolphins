#ifndef DEVICE_H
#define DEVICE_H

#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h> 
#include <pthread.h>
#include <controller.h>
#include "./serial.h"


char buff_temp[256];//临时储存用
int temp;           
extern struct timeval tv;  //超时时间设定


//文件描述符
FILE *file_data;    //记录数据

#define FD_NUM 10
fd_set rd[FD_NUM];  //文件描述符集合
int fd[FD_NUM];     //用于打开串口
int fd_i[FD_NUM];   //用于配置串口
int maxfd[FD_NUM];  //最大文件描述符
int nread[FD_NUM];
int nwrite[FD_NUM];

fd_set rd_NV;       //组合导航
int fd_NV;          //组合导航
int fd_i_NV;
int maxfd_NV;       //组合导航

//AD模块采集值
typedef struct {
		double rudder;//舵角
		double vol;   //推力
		pthread_mutex_t mutex;//互斥所
	}AD_data;           
extern AD_data AD_DATA;
char buff_AD[256];

//组合导航采集值
void AD_sample(int);
void pwm_rudder(double);
void pwm_thrust(double);
void fd_initialize();
void fd_cleanup();

void* AD_thread();

#endif