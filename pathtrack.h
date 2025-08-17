


#ifndef PATHTRACK_H
#define PATHTRACK_H

#include <stdio.h> 
#include <string.h> 
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include "simulation.h"
#define PI 3.14159265358979


typedef struct {
    double *x;  //纬度方向
    double *y;  //经度方向
    double *psi;//艏向
    int num;    //航路店数量
    int index;  //当前航路店索引
    pthread_mutex_t lock; // 互斥锁
} PathPoint;
  

double LOS(double psi, double psit, double error, double delta);
double psi_single(double x0,double y0,
                  double x1,double y1,
                  double x2,double y2);
void psi_all(PathPoint *point) ;
typedef struct{  
    //初始化控制器参数
    double delta; //视线角制导
    double y_int;
    double sigma;
    double ye_1;
} guidance_Data;  

double ILOS(double);
void init_Point(PathPoint *p, int size);
void free_Point(PathPoint *p);
double trackguidance(PathPoint *point,double USV[3]);
extern PathPoint Point;
#endif // PATHTRACK_H