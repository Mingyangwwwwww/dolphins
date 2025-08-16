
#include "./controller.h"

int HEADING_MODE=0;//判断是谁是艏向角还是角速度
/*--------------------------------------------艏向角控制器-----------------------------------------------*/
controller_Data_psi CTRL_DATA_PSI = { // 初始化全局变量  
     
    .mutex = PTHREAD_MUTEX_INITIALIZER,// 初始化互斥锁,通信修改这个的时候需要解锁
    .mode=1,
    .gain=1.0,//输入增益系数
    .Ts = 0.1,//控制器采样时间
    .rudder_max=30.0,//最大舵角
    .kxi=0,

    // //PID
    .kp = 1.8, 
    .ki = 0.0001,  
    .kd = 0.4,
    // .kp = 3.2, 
    // .ki = 0.0001,  
    // .kd = 1.2,
    //积分补偿
    .rudder_int=0, //虚拟输入
    .sigma=0.00,   //积分限幅

    //S面
    .k1=0.5,
    .k2=0.1,

    //紧格式MFAC,
    // .lamda=0.2,
    // .p={0,0.5,0}, //全格式、紧格式、偏格式，
    // .FY0={0,1,0}, //全格式、紧格式，偏格式
    // .FY_1={0,1,0}, 
    // .yita=1,        //η
    // .miu=100.0,       //μ
    // .epsilon=0.001, //ε

    //偏格式
    .lamda=0.5,//0.5,
    .p={0,0.9,0.8}, //全格式、紧格式、偏格式，
    .FY0={0,1,0}, //全格式、紧格式，偏格式
    .FY_1={0,1,0}, 
    .yita=1,        //η
    .miu=100.0,       //μ
    .epsilon=0.001, //ε

    //全格式
    // .lamda=0.2,//0.5,
    // .p={0.1,0.4,0}, //全格式、紧格式、偏格式，
    // .FY0={0.6,0.5,0}, //全格式、紧格式，偏格式
    // .FY_1={0.6,0.5,0}, 
    // .yita=1,        //η
    // .miu=100.0,       //μ
    // .epsilon=0.001, //ε
    //改进参数
    .Kr=1,//角速度分量

    // 其他字段默认为0，因为它们是基本类型的数组或变量  
    // 如果需要非零初始化，可以在这里设置  
};  

int sign(double x) {
    return (x > 0) - (x < 0);
}

double rudder_limit(double delta){
    if (fabs(delta)>CTRL_DATA_PSI.rudder_max){
        return CTRL_DATA_PSI.rudder_max*sign(delta);
    }else{
        return delta;
    }
}

static double PID(double psid, double psi, double r) {                  //实际上这里不需要r，但总有些控制器要r的，统一一下吧  
    double Error = psid - psi;                                   //计算偏差量
    double dError=(Error - CTRL_DATA_PSI.e_1[0])/CTRL_DATA_PSI.Ts;    //计算微分量
    double delta = CTRL_DATA_PSI.kp*Error                            //计算
                    +CTRL_DATA_PSI.ki*CTRL_DATA_PSI.ErrorSum
                    +(CTRL_DATA_PSI.kd*dError);   
    return delta;
}

//求解二范数
static double norm(const double* vector, int size) {
    double sum = 0.0;
    int i;
    for ( i = 0; i < size; i++) {
        sum += vector[i] * vector[i];
    }
    return sqrt(sum);
}

//对应元素相乘
static double vector_multiply(const double* p,const double *q, int size){
    double sum = 0;
    int i;
    for ( i = 0; i < size; i++) {
        sum += p[i] * q[i];
    }
    return sum;

}

//位置式全格式
static double P_MFAC(double yd, double y, double r) {
    double dy = y - CTRL_DATA_PSI.y_1[0];
    double P_1[3] = {CTRL_DATA_PSI.y_1[0] - CTRL_DATA_PSI.y_1[1], CTRL_DATA_PSI.u_1[0],CTRL_DATA_PSI.u_1[1]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        P_1[i] = (CTRL_DATA_PSI.p[i]>= 1e-3) ? P_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_PSI.FY_1[0] + (CTRL_DATA_PSI.yita * P_1[0]) / (CTRL_DATA_PSI.miu + vector_multiply(P_1,P_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    FY[1] = CTRL_DATA_PSI.FY_1[1] + (CTRL_DATA_PSI.yita * P_1[1]) / (CTRL_DATA_PSI.miu + vector_multiply(P_1,P_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    FY[2] = CTRL_DATA_PSI.FY_1[2] + (CTRL_DATA_PSI.yita * P_1[2]) / (CTRL_DATA_PSI.miu + vector_multiply(P_1,P_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_PSI.epsilon) || 
        (norm(P_1, 3) <= CTRL_DATA_PSI.epsilon) ||
        (FY[1] * CTRL_DATA_PSI.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_PSI.FY_1,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_PSI.lamda*fabs(FY[1])+FY[1]*FY[1]);
    double u = kxi_FY*(CTRL_DATA_PSI.p[1]*(yd - y)-FY[0]*CTRL_DATA_PSI.p[0]*P_1[0]-FY[2]*CTRL_DATA_PSI.p[2]*P_1[1]);
            // 检查 u 是否为 NaN
    //输出控制输入
    memcpy(CTRL_DATA_PSI.FY_1, FY, sizeof(FY));
    return u;
}

//标准全格式
static double MFAC(double yd, double y, double r) {
    double dy = y - CTRL_DATA_PSI.y_1[0];
    double H_1[3] = {CTRL_DATA_PSI.y_1[0] - CTRL_DATA_PSI.y_1[1], CTRL_DATA_PSI.u_1[0]-CTRL_DATA_PSI.u_1[1],CTRL_DATA_PSI.u_1[1]-CTRL_DATA_PSI.u_1[2]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        H_1[i] = (CTRL_DATA_PSI.p[i]>= 1e-3) ? H_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_PSI.FY_1[0] + (CTRL_DATA_PSI.yita * H_1[0]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[1] = CTRL_DATA_PSI.FY_1[1] + (CTRL_DATA_PSI.yita * H_1[1]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[2] = CTRL_DATA_PSI.FY_1[2] + (CTRL_DATA_PSI.yita * H_1[2]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_PSI.epsilon) || 
        (norm(H_1, 3) <= CTRL_DATA_PSI.epsilon) ||
        (FY[1] * CTRL_DATA_PSI.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_PSI.FY0,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_PSI.lamda+FY[1]*FY[1]);
    double u = CTRL_DATA_PSI.u_1[0]+kxi_FY*(CTRL_DATA_PSI.p[1]*(yd - y)-FY[0]*CTRL_DATA_PSI.p[0]*H_1[0]-FY[2]*CTRL_DATA_PSI.p[2]*H_1[1]);
            // 检查 u 是否为 NaN
    //输出控制输入
    memcpy(CTRL_DATA_PSI.FY_1, FY, sizeof(FY));

    return u;
}

//强跟踪全格式
static double ST_MFAC(double yd, double y, double r) {
    double dy = y - CTRL_DATA_PSI.y_1[0];
    double H_1[3] = {CTRL_DATA_PSI.y_1[0] - CTRL_DATA_PSI.y_1[1], CTRL_DATA_PSI.u_1[0]-CTRL_DATA_PSI.u_1[1],CTRL_DATA_PSI.u_1[1]-CTRL_DATA_PSI.u_1[2]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        H_1[i] = (CTRL_DATA_PSI.p[i]>= 1e-3) ? H_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_PSI.FY_1[0] + (CTRL_DATA_PSI.yita * H_1[0]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[1] = CTRL_DATA_PSI.FY_1[1] + (CTRL_DATA_PSI.yita * H_1[1]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[2] = CTRL_DATA_PSI.FY_1[2] + (CTRL_DATA_PSI.yita * H_1[2]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_PSI.epsilon) || 
        (norm(H_1, 3) <= CTRL_DATA_PSI.epsilon) ||
        (FY[1] * CTRL_DATA_PSI.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_PSI.FY0,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_PSI.lamda+FY[1]*FY[1]);
    double u = CTRL_DATA_PSI.u_1[0]+kxi_FY*((CTRL_DATA_PSI.p[1]*(yd - y)-CTRL_DATA_PSI.Kr*(CTRL_DATA_PSI.y_1[0]-CTRL_DATA_PSI.y_1[1]+CTRL_DATA_PSI.yd_1[0]-CTRL_DATA_PSI.yd_1[1]))-FY[0]*CTRL_DATA_PSI.p[0]*H_1[0]-FY[2]*CTRL_DATA_PSI.p[2]*H_1[1]);
    
            // 检查 u 是否为 NaN
    //输出控制输入
    memcpy(CTRL_DATA_PSI.FY_1, FY, sizeof(FY));

    return u;
}

//差分全格式
static double D_MFAC(double yd, double y, double r) {
    double dy = y - CTRL_DATA_PSI.y_1[0];
    double H_1[3] = {CTRL_DATA_PSI.y_1[0] - CTRL_DATA_PSI.y_1[1], CTRL_DATA_PSI.u_1[0]-CTRL_DATA_PSI.u_1[1],CTRL_DATA_PSI.u_1[1]-CTRL_DATA_PSI.u_1[2]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        H_1[i] = (CTRL_DATA_PSI.p[i]>= 1e-3) ? H_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_PSI.FY_1[0] + (CTRL_DATA_PSI.yita * H_1[0]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[1] = CTRL_DATA_PSI.FY_1[1] + (CTRL_DATA_PSI.yita * H_1[1]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    FY[2] = CTRL_DATA_PSI.FY_1[2] + (CTRL_DATA_PSI.yita * H_1[2]) / (CTRL_DATA_PSI.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_PSI.FY_1,H_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_PSI.epsilon) || 
        (norm(H_1, 3) <= CTRL_DATA_PSI.epsilon) ||
        (FY[1] * CTRL_DATA_PSI.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_PSI.FY0,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_PSI.lamda+FY[1]*FY[1]);
    double u = CTRL_DATA_PSI.u_1[0]+kxi_FY*((CTRL_DATA_PSI.p[1]*(yd - y)-CTRL_DATA_PSI.Kr*(CTRL_DATA_PSI.y_1[0]-CTRL_DATA_PSI.y_1[1])/CTRL_DATA_PSI.Ts)-FY[0]*CTRL_DATA_PSI.p[0]*H_1[0]-FY[2]*CTRL_DATA_PSI.p[2]*H_1[1]);
    
    // 检查 u 是否为 NaN
    //输出控制输入
    memcpy(CTRL_DATA_PSI.FY_1, FY, sizeof(FY));

    return u;
}



// static double S_plane(double psid, double psi, double r){               //s面控制
//     double Error = psid - psi;                                   //计算偏差量    
//     double dError=(Error - CTRL_DATA_PSI.e_1[1])/CTRL_DATA_PSI.Ts;    //计算微分量
//     double delta=2/(1+exp(-CTRL_DATA_PSI.k1*Error-CTRL_DATA_PSI.k2*dError))-1;
//     delta *=CTRL_DATA_PSI.rudder_max;                                //注意输出是-1到1,因此需要
//     return delta;
// }


//智能转向器
double angle_to_pi(double angle) {
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
}

/*转向器*/
static double convert_direction(double psid, double psi) {
        psid=psi+angle_to_pi(psid-psi);
        return psid;
    }


static double psi_control(double psid, double psi, double r,int index){
    // 计算时间间隔（仅当 last_time 已初始化时）
    struct timeval tv_current;
    gettimeofday(&tv_current, NULL);
    double current_time = tv_current.tv_sec * 1000.0 + tv_current.tv_usec * 0.001;
    if (CTRL_DATA_PSI.last_time != 0) {
        CTRL_DATA_PSI.Ts = (current_time- CTRL_DATA_PSI.last_time) / 1000.0; // 转为秒
    } else {
        CTRL_DATA_PSI.Ts = 0.1; // 默认值
    }
    CTRL_DATA_PSI.last_time = current_time; // 存储毫秒时间戳

    //定义函数指针
    psid=convert_direction(psid, psi);
    // if (index==4){psi=psi+CTRL_DATA_PSI.Kr*r;}//选择重构输出
    double (*controller[])(double, double,double) = {PID, P_MFAC, MFAC ,ST_MFAC, D_MFAC};//设定index对应的标号
                                                    //0、   1 、    2 、   3 、    4            
    double delta=controller[index](psid, psi, r);
    //记录
    delta=rudder_limit(delta); //舵角限幅
    //赋值回传
    CTRL_DATA_PSI.u_1[2] = CTRL_DATA_PSI.u_1[1];
    CTRL_DATA_PSI.u_1[1] = CTRL_DATA_PSI.u_1[0];
    CTRL_DATA_PSI.u_1[0] = delta;
    CTRL_DATA_PSI.y_1[2] = CTRL_DATA_PSI.y_1[1];
    CTRL_DATA_PSI.y_1[1] = CTRL_DATA_PSI.y_1[0];
    CTRL_DATA_PSI.y_1[0] = psi;
    CTRL_DATA_PSI.yd_1[2] = CTRL_DATA_PSI.yd_1[1];
    CTRL_DATA_PSI.yd_1[1] = CTRL_DATA_PSI.yd_1[0];
    CTRL_DATA_PSI.yd_1[0] = psid;
    CTRL_DATA_PSI.r_1[2] = CTRL_DATA_PSI.r_1[1];
    CTRL_DATA_PSI.r_1[1] = CTRL_DATA_PSI.r_1[0];
    CTRL_DATA_PSI.r_1[0] = r;
    CTRL_DATA_PSI.e_1[2] = CTRL_DATA_PSI.e_1[1];
    CTRL_DATA_PSI.e_1[1] = CTRL_DATA_PSI.e_1[0];
    CTRL_DATA_PSI.e_1[0] = psid - psi;
    CTRL_DATA_PSI.ErrorSum +=CTRL_DATA_PSI.e_1[0]*CTRL_DATA_PSI.Ts;  //计算积分量

    CTRL_DATA_PSI.rudder_int=CTRL_DATA_PSI.kxi*(psid-CTRL_DATA_PSI.yd_1[1]);//控制器执行

    // CTRL_DATA_PSI.rudder_int+=delta/(1+pow(delta+CTRL_DATA_PSI.sigma*CTRL_DATA_PSI.rudder_int,2));//积分环节
    // if (fabs(CTRL_DATA_PSI.sigma*CTRL_DATA_PSI.rudder_int)>5){CTRL_DATA_PSI.rudder_int=5/CTRL_DATA_PSI.sigma*signbit(CTRL_DATA_PSI.rudder_int);}
    // delta+=CTRL_DATA_PSI.sigma*CTRL_DATA_PSI.rudder_int;//添加积分
    delta=rudder_limit(delta+CTRL_DATA_PSI.rudder_int);                          //舵角限幅(可选)
    return CTRL_DATA_PSI.gain*delta;
}


/*--------------------------------------------航速控制器-----------------------------------------------*/
controller_Data_v CTRL_DATA_V = { // 初始化全局变量  
    // 初始化互斥锁
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .mode=1,//MFAC
    .gain=1,
    .Ts = 0.1,
    .n_max=12,
    //PID
    .kp = 0.6, 
    .ki = 2.2,  
    .kd = 0.4,

    //MFAC
    // //紧格式
    // .lamda=2, 
    // .p={0,0.5,0}, //全格式、紧格式、偏格式
    // .FY0={0,1,0}, //全格式、紧格式，偏格式
    // .FY_1={0,1,0}, 
    // .yita=1,  //η
    // .miu=100, //μ
    // .epsilon=0.001, //ε
    // .u_1={0,0,0}

    //偏格式
    .lamda=2.5, 
    .p={0,0.6,1}, //全格式、紧格式、偏格式
    .FY0={0,1,0}, //全格式、紧格式，偏格式
    .FY_1={0,1,0}, 
    .yita=1,  //η
    .miu=100, //μ
    .epsilon=0.001, //ε
    .u_1={0,0,0}

    // //全格式
    // .lamda=1, 
    // .p={0.4,0.2,0}, //全格式、紧格式、偏格式
    // .FY0={1,1,0}, //全格式、紧格式，偏格式
    // .FY_1={1,1,0}, 
    // .yita=1,  //η
    // .miu=100, //μ
    // .epsilon=0.001, //ε
    // .u_1={0,0,0}
    // 其他字段默认为0，因为它们是基本类型的数组或变量  
    // 如果需要非零初始化，可以在这里设置  
};  


static double volt_limit(double n){
    if (n>CTRL_DATA_V.n_max){
        return CTRL_DATA_V.n_max;
    }else if(n<0){
        return 0;
    }else{
        return n;
    }
}


// static double PID_V(double vd, double v) {     //增量型            
//     double e  = vd - v;                                   //积分
//     double de =e-CTRL_DATA_V.e_1[0];                                //比例
//     double dde=e-2*CTRL_DATA_V.e_1[0]+CTRL_DATA_V.e_1[1];                         //微分
//     double n = CTRL_DATA_R.u_1[0]+CTRL_DATA_V.kp*de + CTRL_DATA_V.ki*e+ CTRL_DATA_V.kd*dde;   
//     return n;
// }
static double PID_V(double vd, double v) {       //位置型        
    double Error = vd - v;                                   //计算偏差量
    double dError=(Error - CTRL_DATA_V.e_1[0])/CTRL_DATA_V.Ts;    //计算微分量
    double n = CTRL_DATA_V.kp*Error                            //计算
                    +CTRL_DATA_V.ki*CTRL_DATA_V.ErrorSum
                    +(CTRL_DATA_V.kd*dError);   
    return n;
}

//标准全格式
static double MFAC_V(double yd, double y) {
    double dy = y - CTRL_DATA_V.y_1[0];
    double H_1[3] = {CTRL_DATA_V.y_1[0] - CTRL_DATA_V.y_1[1], CTRL_DATA_V.u_1[0]-CTRL_DATA_V.u_1[1],CTRL_DATA_V.u_1[1]-CTRL_DATA_V.u_1[2]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        H_1[i] = (CTRL_DATA_V.p[i]>= 1e-3) ? H_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_V.FY_1[0] + (CTRL_DATA_V.yita * H_1[0]) / (CTRL_DATA_V.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_V.FY_1,H_1,3));
    FY[1] = CTRL_DATA_V.FY_1[1] + (CTRL_DATA_V.yita * H_1[1]) / (CTRL_DATA_V.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_V.FY_1,H_1,3));
    FY[2] = CTRL_DATA_V.FY_1[2] + (CTRL_DATA_V.yita * H_1[2]) / (CTRL_DATA_V.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_V.FY_1,H_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_V.epsilon) || 
        (norm(H_1, 3) <= CTRL_DATA_V.epsilon) ||
        (FY[1] * CTRL_DATA_V.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_V.FY0,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_V.lamda+FY[1]*FY[1]);
    double u = CTRL_DATA_V.u_1[0]+kxi_FY*(CTRL_DATA_V.p[1]*(yd - y)-FY[0]*CTRL_DATA_V.p[0]*H_1[0]-FY[2]*CTRL_DATA_V.p[2]*H_1[1]);
            // 检查 u 是否为 NaN
    //输出控制输入
    memcpy(CTRL_DATA_V.FY_1, FY, sizeof(FY));

    return u;
}

double v_control(double vd, double v,int index){


    // 计算时间间隔（仅当 last_time 已初始化时）
    struct timeval tv_current;
    gettimeofday(&tv_current, NULL);
    double current_time = tv_current.tv_sec * 1000.0 + tv_current.tv_usec * 0.001;
    if (CTRL_DATA_V.last_time != 0) {
        CTRL_DATA_V.Ts = (current_time- CTRL_DATA_V.last_time) / 1000.0; // 转为秒
    } else {
        CTRL_DATA_V.Ts = 0.1; // 默认值
    }
    CTRL_DATA_V.last_time = current_time; // 存储毫秒时间戳
    //定义函数指针
    double (*controller[])(double, double) = {PID_V,MFAC_V};//设定index对应的标号
    double n=controller[index](vd, v);//控制器执行
    //记录
    n=volt_limit(n); //限幅
    //  printf("After calling controller[%d](%f, %f), vd = %f, v = %f, n = %f\n", index, vd, v, vd, v, n);  
 
    //赋值回传
    CTRL_DATA_V.u_1[2] = CTRL_DATA_V.u_1[1];
    CTRL_DATA_V.u_1[1] = CTRL_DATA_V.u_1[0];
    CTRL_DATA_V.u_1[0] = n;
    CTRL_DATA_V.y_1[2] = CTRL_DATA_V.y_1[1];
    CTRL_DATA_V.y_1[1] = CTRL_DATA_V.y_1[0];
    CTRL_DATA_V.y_1[0] = v;
    CTRL_DATA_V.e_1[2] = CTRL_DATA_V.e_1[1];
    CTRL_DATA_V.e_1[1] = CTRL_DATA_V.e_1[0];
    CTRL_DATA_V.e_1[0] = vd-v;
    CTRL_DATA_V.ErrorSum +=CTRL_DATA_V.e_1[0]*CTRL_DATA_V.Ts;  //计算积分量
    return CTRL_DATA_V.gain*n;
}


/*--------------------------------------------角速度控制器-----------------------------------------------*/

controller_Data_r CTRL_DATA_R = { // 初始化全局变量  
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .mode=1,//MFAC
    .gain=1.0,
    .Ts = 0.1,
    .rudder_max=30.0,
    //PID
    .kp = 0.2, 
    .ki = 0.1,  
    .kd = 0.03,

    //MFAC
    .lamda=1.5, 
    .p={0,0.1,0.4}, //全格式、紧格式、偏格式
    .FY0={0,1.0,0}, //全格式、紧格式，偏格式
    .FY_1={0,1.0,0}, 
    .yita=1.0,  //η
    .miu=20.0, //μ
    .epsilon=0.001, //ε
    // 其他字段默认为0，因为它们是基本类型的数组或变量  
    // 如果需要非零初始化，可以在这里设置  
};  


// static double PID_R(double rd, double r) {                 
//     double e  = rd - r;                                   //积分
//     double de =e-CTRL_DATA_R.e_1[0];                                //比例
//     double dde=e-2*CTRL_DATA_R.e_1[0]+CTRL_DATA_R.e_1[1];                         //微分
//     double n = CTRL_DATA_R.u_1[0]+CTRL_DATA_R.kp*de + CTRL_DATA_R.ki*e+ CTRL_DATA_R.kd*dde;   
//     return n;
// }
static double PID_R(double rd, double r) {       //位置型        
    double Error = rd - r;                                   //计算偏差量
    double dError=(Error - CTRL_DATA_R.e_1[0])/CTRL_DATA_R.Ts;    //计算微分量
    double delta = CTRL_DATA_R.kp*Error                            //计算
                    +CTRL_DATA_R.ki*CTRL_DATA_R.ErrorSum
                    +(CTRL_DATA_R.kd*dError);   
    return delta;
}

static double MFAC_R(double yd, double y) {
    double dy = y - CTRL_DATA_R.y_1[0];
    double H_1[3] = {CTRL_DATA_R.y_1[0] - CTRL_DATA_R.y_1[1], CTRL_DATA_R.u_1[0]-CTRL_DATA_R.u_1[1],CTRL_DATA_R.u_1[1]-CTRL_DATA_R.u_1[2]};   //计算P
    //判断是紧格式、偏格式、还是全格式
    int i;
    for (i = 0; i < 3; i++) {
        H_1[i] = (CTRL_DATA_R.p[i]>= 1e-3) ? H_1[i]: 0;
    }
    //更新PG估计
    double FY[3]={0.0,0.0,0.0};
    FY[0] = CTRL_DATA_R.FY_1[0] + (CTRL_DATA_R.yita * H_1[0]) / (CTRL_DATA_R.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_R.FY_1,H_1,3));
    FY[1] = CTRL_DATA_R.FY_1[1] + (CTRL_DATA_R.yita * H_1[1]) / (CTRL_DATA_R.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_R.FY_1,H_1,3));
    FY[2] = CTRL_DATA_R.FY_1[2] + (CTRL_DATA_R.yita * H_1[2]) / (CTRL_DATA_R.miu + vector_multiply(H_1,H_1,3)) * (dy - vector_multiply(CTRL_DATA_R.FY_1,H_1,3));
    // printf("FY[1] = %.6f, dy = %.6f, P_1[1] = %.6f,dy-dy_hat%.6f\n", FY[1], dy, P_1[1],vector_multiply(CTRL_DATA_PSI.FY_1,P_1,3));
    //伪梯度重置
    if ((norm(FY, 3) <= CTRL_DATA_R.epsilon) || 
        (norm(H_1, 3) <= CTRL_DATA_R.epsilon) ||
        (FY[1] * CTRL_DATA_R.FY0[1] <= 0)) {
        memcpy(FY,CTRL_DATA_R.FY0,sizeof(FY));
    }
    //计算控制
    double kxi_FY=FY[1]/(CTRL_DATA_R.lamda+FY[1]*FY[1]);
    double u = CTRL_DATA_R.u_1[0]+kxi_FY*(CTRL_DATA_R.p[1]*(yd - y)-FY[0]*CTRL_DATA_R.p[0]*H_1[0]-FY[2]*CTRL_DATA_R.p[2]*H_1[1]);
    //输出控制输入
    memcpy(CTRL_DATA_R.FY_1, FY, sizeof(FY));
    // printf("y=%f,yd=%f,u=%f,FY = [%.6f, %.6f, %.6f]\n",y,yd,u ,FY[0], FY[1], FY[2]);
    return u;
}

static double r_control(double rd, double psi, double r,int index){

    // 计算时间间隔（仅当 last_time 已初始化时）
    struct timeval tv_current;
    gettimeofday(&tv_current, NULL);
    double current_time = tv_current.tv_sec * 1000.0 + tv_current.tv_usec * 0.001;
    if (CTRL_DATA_R.last_time != 0) {
        CTRL_DATA_R.Ts = (current_time- CTRL_DATA_R.last_time) / 1000.0; // 转为秒
    } else {
        CTRL_DATA_R.Ts = 0.1; // 默认值
    }
    CTRL_DATA_R.last_time = current_time; // 存储毫秒时间戳

    //定义函数指针
    double (*controller[])(double, double) = {PID_R, MFAC_R};//设定index对应的标号
    double delta=controller[index](rd, r);//控制器执行

    //记录
    delta=rudder_limit(delta); //舵角限幅

    //赋值回传
    CTRL_DATA_R.u_1[2] = CTRL_DATA_R.u_1[1];
    CTRL_DATA_R.u_1[1] = CTRL_DATA_R.u_1[0];
    CTRL_DATA_R.u_1[0] = delta;
    CTRL_DATA_R.y_1[2] = CTRL_DATA_R.y_1[1];
    CTRL_DATA_R.y_1[1] = CTRL_DATA_R.y_1[0];
    CTRL_DATA_R.y_1[0] = r;
    CTRL_DATA_R.e_1[2] = CTRL_DATA_R.e_1[1];
    CTRL_DATA_R.e_1[1] = CTRL_DATA_R.e_1[0];
    CTRL_DATA_R.e_1[0] = rd-r;
    CTRL_DATA_R.ErrorSum +=CTRL_DATA_R.e_1[0]*CTRL_DATA_R.Ts;  //计算积分量
    return CTRL_DATA_R.gain*delta;
}


double heading_control(double yd, double psi, double r,int index){
    double (*controller[])(double,double,double,int) ={psi_control, r_control};
    return  controller[HEADING_MODE](yd, psi, r, index);
}
