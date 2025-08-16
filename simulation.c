#include <./simulation.h>


double longitude_base =126.6224487;//基站经度
double latitude_base  =45.8006640;   //基站纬度

//此程序贴标签
void label(State* state) {
    // 提取经纬度的小数部分后两位
    int long_frac = (int)(fabs(state->longitude) * 10000000) % 10000000; // 经度小数部分后两位
    int lat_frac = (int)(fabs(state->latitude) * 10000000) %10000000;   // 纬度小数部分后两位
    // 选择USV_velocity和delta_d的小数部分后一位
    int vel_frac = (int)(fabs(state->USV_velocity) * 10) % 10; // 速度小数部分后一位
    int delta_d_frac = (int)(fabs(state->psid) * 10) % 10;     // 舵角命令小数部分后一位
    // 简单组合成一个四位数字以内的标签
    state->math_label = (long_frac * 100 + lat_frac * 10 + vel_frac * 3 + delta_d_frac) % 10000;
}

static void get_state(State* state) {    
    
    //AD_sample(6);                                      //获取AD状态，6不是文件描述符，而是文件描述符的序号
    //printf('采样结束\n');
	navigator();                                       //获取导航信息

    //更新状态
    pthread_mutex_lock(&state->mutex);                  //准备更新状态，调用串口获取数据  
    pthread_mutex_lock(&AD_DATA.mutex);                  //准备更新状态，调用串口获取数据            
    state->longitude =feedback_guandao_data1.longitude;//经度
    state->latitude  =feedback_guandao_data1.Lattitude;//纬度
    state->USV[0][0] =feedback_guandao_data1.Yaw;      //艏向
    state->USV[1][0] =feedback_guandao_data1.Gyro_z;   //艏向角速度
    state->USV[0][1] =la2x(state->latitude);           //x坐标
    state->USV[1][1] =navigator_data1.North_velocity;  //向北速度
    state->USV[0][2] =lo2y(state->longitude);          //y坐标
    state->USV[1][2] =navigator_data1.East_velocity;   //向东速度
    state->yaw = atan2(state->USV[1][2], state->USV[1][1])/M_PI*180;
    if(CTRL_DATA_PSI.mode!=0){
        state->delta_c=CTRL_DATA_PSI.sigma*CTRL_DATA_PSI.rudder_int;//积分量补正 
    }else{
        state->delta_c=CTRL_DATA_PSI.ki*CTRL_DATA_PSI.ErrorSum;     //PID输出积分量
    }
    state->USV_velocity=feedback_guandao_data1.GPSV;   //速度
    state->delta=AD_DATA.rudder;                       //实际舵角
    state->vol  =AD_DATA.vol;                          //实际推力
    label(state);                                      //为数字打上标签

    // 写入dat文件
    struct timeval tv_t;
    gettimeofday(&tv_t, NULL);  // 获取当前时间（秒 + 微秒）
    struct tm *tm_info = localtime(&tv_t.tv_sec);
    strftime(state->timestamp, sizeof(state->timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    snprintf(state->timestamp + strlen(state->timestamp), sizeof(state->timestamp) - strlen(state->timestamp), ".%06ld", tv_t.tv_usec);

    fprintf(file_data, "%s,%d,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        state->timestamp,   //时间戳
        state->math_label,  //数字标签
        state->longitude,   //经度
        state->latitude,    //纬度
        state->USV[0][0],   //艏向 
        state->USV[1][0],   //角速度
        state->USV[0][1],   //x坐标 
        state->USV[1][1],   //向北速度
        state->USV[0][2],   //y坐标
        state->USV[1][2],   //向东速度
        state->psid,        //期望艏向
        state->vd,          //期望速度
        state->rd,          //期望角速度
        state->delta_d,     //期望舵角
        state->vol_d,       //期望推进器电压
        state->delta_c,     //舵角积分补偿
        state->USV_velocity,//航速
        state->delta,       //实际舵角
        state->vol,         //实际推进器电压
        CTRL_DATA_PSI.FY_1[0],//艏PG1
        CTRL_DATA_PSI.FY_1[1],//艏PG2
        CTRL_DATA_PSI.FY_1[2],//艏PG3
        CTRL_DATA_R.FY_1[0],  //角PG1
        CTRL_DATA_R.FY_1[1],  //角PG2
        CTRL_DATA_R.FY_1[2],  //角PG3
        CTRL_DATA_V.FY_1[0],  //速PG1
        CTRL_DATA_V.FY_1[1],  //速PG2
        CTRL_DATA_V.FY_1[2]   //速PG3
        );        //实际电压        
    //记录文件
    fflush(file_data); 
    pthread_mutex_unlock(&AD_DATA.mutex);                   //准备更新状态，调用串口获取数据
    pthread_mutex_unlock(&state->mutex);                    //状态更新结束，调用串口获取数据
}

void* thread_simulation(void* arg) {
    State* state = (State*)arg;
    pthread_mutex_init(&state->mutex, NULL);//初始化互斥锁

    // // 创建或打开 .dat 文件
    // FILE* fp = fopen("state_data.dat", "w");
    // if (fp == NULL) {
    //     perror("Failed to open file");
    //     pthread_mutex_destroy(&state->mutex);
    //     return NULL;
    // }


    while (1) {
        // fprintf(fp, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
        //         state->USV[0][0], state->USV[0][1], state->USV[0][2],
        //         state->UUV[0][0], state->UUV[0][1], state->UUV[0][2]);
        // fprintf(fp, "%.5f,%.5f,%.5f,%.5f\n",
        //         state->USV_velocity, state->UUV_velocity, state->vol, state->delta);
        if(state->pathtrack_switch==1){//表明现在是轨迹跟踪模式
            pthread_mutex_lock(&Point.lock);
            state->psid=trackguidance(&Point,state->USV[0]);
            pthread_mutex_unlock(&Point.lock);
        }
        
        if(state->controller_switch>0 && state->controller_switch<=2 && state->zigzag_switch==0){              //表明现在是期望航向航速模式
            HEADING_MODE=state->controller_switch-1; //HEADING MODE为0，则是艏向控制，为1则是角速度控制
            state->delta_d=heading_control((HEADING_MODE ==1) ? state->rd:state->psid ,state->USV[0][0],state->USV[1][0],CTRL_DATA_PSI.mode);
            state->vol_d=v_control(state->vd,state->USV_velocity,CTRL_DATA_V.mode);
        }

        if(state->controller_switch==3 && state->zigzag_switch==0){//表明现在是期望航向模式,但是推进器不用控制器
            HEADING_MODE=0;
            state->delta_d=heading_control((HEADING_MODE ==1) ? state->rd:state->psid ,state->USV[0][0],state->USV[1][0],CTRL_DATA_PSI.mode);     //CTRL_DATA_PSI.mode
        }

        if(state->zigzag_switch==1){ //表明现在是Z型机动
            state->vol_d=v_control(1.0,state->USV_velocity,CTRL_DATA_V.mode);//期望航速1.0
            if ((state->USV[0][0]) >= 20.0){ //
		    state->delta_d = -20.0;
            }else if (state->USV[0][0] <= -20.0){
			state->delta_d= 20.0;
		    }
		 }
        // state->vol_d=0.0;
        // state->delta_d=10.0;
        pwm_rudder(state->delta_d);//操舵printf("你好");
        pwm_thrust(state->vol_d);  //推进器
        usleep(100000);            //暂停0.1s
        get_state(state);          //获取位置信息
        
    }
    // fclose(fp);
    pthread_mutex_destroy(&state->mutex);//摧毁互斥锁
    return NULL;
}

double y2lo(double x) {
    return longitude_base + (x / (EARTH_RADIUS * cos(latitude_base * M_PI/180.0))) * 180.0/M_PI;
}

double x2la(double y) {
    return latitude_base + (y / EARTH_RADIUS) * 180.0/M_PI;
}

double lo2y(double longitude) {
    return EARTH_RADIUS * cos(latitude_base * M_PI/180.0) * (longitude - longitude_base) * M_PI/180.0;
}

double la2x(double latitude) {
    return EARTH_RADIUS * (latitude - latitude_base) * M_PI/180.0;
}

