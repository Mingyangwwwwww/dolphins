
#ifndef CONTROLLER_H  
#define CONTROLLER_H  

#include <stdio.h> 
#include <string.h> 
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h> 
//艏向角
typedef struct{  
    // 添加互斥锁
    pthread_mutex_t mutex;//
    //控制器模式
    int mode;
    double Ts;                  //采样时间
    double last_time;  // 上一次调用这个函数的时间

    double gain;//艏向角增益
    //初始化控制器参数
    double kp;double ki;double kd;//PID控制器参数，
    double k1;double k2;//S面控制器
    double lamda;double p[3];double yita;double miu;double epsilon;double Kr;double FY_1[3]; double FY0[3];//MFAC控制器 
   
    double kxi;//强跟踪系数
    //舵角限幅 
    double rudder_max;
    //普通控制器需要的参数
    double y_1[3];   //航向角  
    double r_1[3];     //角速度     
    double u_1[3];     //舵角
    //CFDL-MFAC控制器需要的参数

    //PFDL和全格式控制器需要的参数

    double yd_1[3];  //期望航向角，强跟踪可能需要
    
    //PID需要
    double e_1[3];         //上一时刻error
    double ErrorSum;     //积分err

     //积分补偿
    double rudder_int;
    double sigma;

} controller_Data_psi;  



//速度
typedef struct{  
    // 添加互斥锁
    pthread_mutex_t mutex;
    //控制器模式
    int mode;
    double gain;
    //初始化控制器参数
    double kp;double ki;double kd;//PID控制器参数，
    double lamda;double p[3];double yita;double miu;double epsilon;double Kr;double FY_1[3]; double FY0[3];//MFAC控制器 
    //舵角限幅 
    double n_max;
    //普通控制器需要的参数
    double y_1[3];   //航向角    
    double u_1[3];     //舵角
    //MFAC控制器需要的参数
    double Ts;         //采样时间
    //PID需要
    double e_1[3];  //上一时刻error
    double ErrorSum;     //积分err
    double last_time;  // 上一次调用这个函数的时间
} controller_Data_v;  



//角速度控制器
typedef struct{  
    // 添加互斥锁
    pthread_mutex_t mutex;
    //控制器模式
    int mode;
    double gain;
    //初始化控制器参数
    double kp;double ki;double kd;//PID控制器参数，
    double lamda;double p[3];double yita;double miu;double epsilon;double Kr;double FY_1[3]; double FY0[3];//MFAC控制器 
    //舵角限幅 
    double rudder_max;
    //普通控制器需要的参数
    double y_1[3];   //航向角    
    double u_1[3];     //舵角
    //MFAC控制器需要的参数
    double Ts;         //采样时间
    //PID需要
    double e_1[3];  //上一时刻error
    double ErrorSum;     //积分err
    double last_time;  // 上一次调用这个函数的时间

} controller_Data_r;  

extern int HEADING_MODE;                 //定义为艏向模式，如果是0就是艏向控制，定义为1就是角速度控制         
extern controller_Data_psi CTRL_DATA_PSI;//设置为全局变量，也就是可以进行修改
extern controller_Data_v CTRL_DATA_V;    //设置为全局变量，也就是可以进行修改
extern controller_Data_r CTRL_DATA_R;    //设置为全局变量，也就是可以进行修改

//声明
double angle_to_pi(double angle);
double heading_control(double yd, double psi, double r,int index);
double v_control(double vd, double v,int index);
#endif