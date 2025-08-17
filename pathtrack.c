
#include<./pathtrack.h>
#define EARTH_RADIUS 6371000.0 // 地球半径，单位为米

////////////////////////初始化航路点///////////////////
PathPoint Point;


guidance_Data GUI_DATA = { // 初始化全局变量  
    .delta = 10,
    .y_int=0,
    .sigma=0.1,
    .ye_1=0
};



void init_Point(PathPoint* point, int num) {
    // 分配内存
    point->x = (double*)malloc(num * sizeof(double));
    
    point->y = (double*)malloc(num * sizeof(double));
    if (point->y == NULL) {
        free(point->x);
    }
    
    point->psi = (double*)malloc(num * sizeof(double));
    if (point->psi == NULL) {
        free(point->x);
        free(point->y);
    }
    
    // 初始化其他成员
    point->num = num;
    point->index = 0;
    if (pthread_mutex_init(&point->lock, NULL) != 0) {
        free(point->x);
        free(point->y);
        free(point->psi);
    }
}

void free_Point(PathPoint* point) {
    // 释放内存
    if (point->x != NULL) {
        free(point->x);
    }
    if (point->y != NULL) {
        free(point->y);
    }
    if (point->psi != NULL) {
        free(point->psi);
    }
}

//智能转向器
static double convert_direction(double psid, double psi) {
    double psid_c = psid;
    while (fabs(psid_c - psi - 360.0) < fabs(psid - psi)) {
        psid_c -= 360.0;
    }
    while (fabs(psid_c - psi + 360.0) < fabs(psid - psi)) {
        psid_c += 360.0;
    }
    if (fabs(psid_c - psi) < 120.0){
    psid = psid_c;
    }
    return psid;
}

double ILOS(double ye) {
    double psid;
    if(fabs(ye)<3){//离航线比较近
        double delta=GUI_DATA.delta;
        GUI_DATA.y_int+=(ye*delta)/(pow(delta,2)+pow(ye+ GUI_DATA.sigma*GUI_DATA.y_int,2));
        psid = -atan((ye+ GUI_DATA.sigma*GUI_DATA.y_int)/delta) / M_PI * 180.0;
    }else{
        double delta=GUI_DATA.delta;
        GUI_DATA.y_int=0;//清理积分项
        psid = -atan(ye/delta) / M_PI * 180.0;

    }
    GUI_DATA.ye_1=ye;
    return psid;
}


//参考航向角，切向角，横向误差，制导参数
double LOS(double psi, double psit, double error, double delta) {
    double psid = psit + atan(error / delta) / PI * 180.0;
    psid = convert_direction(psid,psi);
    return psid;
}

//位置误差求解器
void error(double psit, double delta_x, double delta_y, double *xe, double *ye) {
    psit = psit / 180.0 * PI;
    *xe = delta_x * cos(psit) + delta_y * sin(psit);
    *ye = delta_y * cos(psit) - delta_x * sin(psit);
}



//航路点切线角处理系统
double psi_single(double x0,double y0,
                  double x1,double y1,
                  double x2,double y2) {  
    double angle1, angle2,angle;  
    if ((x0 == x1 && y0 == y1) && (x1 == x2 && y1 == y2) && (x0 == x2 && y0 == y2) ) {  
  
        return 0.0;  
        }else if(x0 == x1 && y0 == y1){
            return  atan2(y2 - y1, x2 - x1)/PI*180;   //计算角度
        }else if(x1 == x2 && y1 == y2){
            return  atan2(y1 - y0, x1 - x0)/PI*180;   //计算角度
        }

    angle1 = atan2(y2 - y1, x2 - x1);   //计算角度
    angle2 = atan2(y1 - y0, x1 - x0);  // 计算角度
    if (angle1 * angle2 >= 0 || (x0 < x1 && x1 < x2)) {  
            angle= (angle1 + angle2) / 2;  
        } else {  
            angle=  (angle1 + angle2 + 2 * PI)/2;  
        }  
    return angle/PI*180;
    }  


//航路点q系统,更新最近索引和路径切线角
void psi_all(PathPoint *point) {
        int i=0;
        for (i = 0; i < point->num; i++) {
            // 计算当前航路点的角度
            if (i > 0 && i< point->num - 1) {
                point->psi[i] = psi_single(point->x[i - 1], point->y[i - 1],
                                           point->x[i    ], point->y[i    ],
                                           point->x[i + 1], point->y[i + 1]);
            } else if (i == 0) {
                point->psi[0] = psi_single(point->x[0], point->y[0],
                                                      point->x[0], point->y[0],
                                                      point->x[1], point->y[1]);
            } else if (point->index == point->num - 1) {
                point->psi[point->num - 1] = psi_single(point->x[point->num - 2], point->y[point->num - 2],
                                                        point->x[point->num - 1], point->y[point->num - 1],
                                                        point->x[point->num - 1], point->y[point->num - 1]);
     
            }
        }
}




//航路点处理系统,更新最近索引和路径切线角
double trackguidance(PathPoint *point, double USV[3]) {
    double dis;
    double dis_1 = pow((point->x[point->index] - USV[1]), 2) + pow((point->y[point->index] - USV[2]), 2);
    if (point->index< point->num-1){
        for (point->index = point->index + 1; point->index < point->num; point->index++) {
            // 航路点
            dis = pow((point->x[point->index] - USV[1]), 2) + pow((point->y[point->index] - USV[2]), 2);
            if (dis > dis_1) {
                point->index--;
                break;
            }
            dis_1 = dis;
        }
}
    if (point->index >= 0 && point->index < point->num) {
        double delta_x= (point->x[point->index] - USV[1]);//纬度方向x
        double delta_y=(point->y[point->index] - USV[2]);//经度方向y
        double xe, ye;
        error(point->psi[point->index], delta_x, delta_y, &xe, &ye);
        double psid = LOS(USV[0], point->psi[point->index], ye, GUI_DATA.delta); 
        printf("Cross-track Error: /(%f, %f) meters   ", xe, ye);
        printf("Path Angle: %f degrees\n", point->psi[point->index]);
        printf("LOS Guidance Angle: %f degrees\n", psid );
    
        return psid;
    } else {
        return 0.0; // 如果 point->index 越界，返回 0.0
    }
}