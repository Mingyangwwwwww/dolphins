


#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <./pathtrack.h>
#include <./controller.h>
#include "./device.h"
#include "./navigator.h"
#define  EARTH_RADIUS  6371000.0 // 地球半径,单位为米


//定义全局变量，航路点缓冲池子
typedef struct {
    int    math_label;      //数字标签，用于与上位机对齐  
    char   timestamp[27];  //时间戳
    double longitude;    //经度
    double latitude;     //纬度
    double USV[2][3];    //USV状态
    double yaw;          //根据坐标差分计算的航向
    double UUV[2][3]; 
    double USV_velocity; //USV速度
    double UUV_velocity;
    double delta_d;      // 舵角命令
    double vol_d;        // 电压命令   
    double delta;        // 实际舵角
    double vol;          // 实际电压
    double psid;         // 期望航速
    double delta_c;      // 舵角补正量，涵盖积分量
    double vd;           // 期望速度
    double rd;           // 期望艏向角速度
    int controller_switch;//是否开启控制器控制
    int pathtrack_switch;//是否开启航路点制导
    int zigzag_switch;   //是否开启Z字机动
    pthread_mutex_t mutex;
} State;



extern double longitude_base;
extern double latitude_base;
void* thread_simulation(void* arg) ;
// 坐标转换函数声明
double y2lo(double x);
double x2la(double y);
double lo2y(double longitude);
double la2x(double latitude);
#endif // SIMULATION_H