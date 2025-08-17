
#define _XOPEN_SOURCE 700  // 启用 POSIX 2008 标准（包含 strptime）
#include "./instruction_cache.h"
#define MAX_PARTS 96


void* execute_instructions(void* arg) {    //void*不规定具体的格式
    ThreadArgs* args = (ThreadArgs*)arg;   //将输入的指令传入
    instruction_list_t* list = args->list; 
    State* state             = args->state;
    while (1) {
        pthread_mutex_lock(&list->mutex);
        while (list->head == NULL) {
            pthread_cond_wait(&list->cond, &list->mutex);
        }
        
        // 获取第一个节点，即命令
        instruction_node_t* curr = list->head;
        list->head = curr->next;
        if (list->head == NULL) {
            list->tail = NULL;
        }
        list->count--;
        pthread_mutex_unlock(&list->mutex);


        //执行指令开始
        pthread_mutex_lock(&state->mutex);
        // printf("Executing instruction: %s\n", curr->data);
        //请注意curr->data是命令的字符串,然后
        execute_TI(state, curr->data);
        execute_CM(state, curr->data);
        execute_PP(state, curr->data);
        execute_PA(state, curr->data);
        execute_ZT(state, curr->data);
        execute_UU(state, curr->data);
        pthread_mutex_unlock(&state->mutex);
  
        //执行指令结束
        // 释放节点
        free(curr);
    }

    return NULL;
}
//添加链表元素
void add_instruction(instruction_list_t* list, char* data) {
    instruction_node_t* node = malloc(sizeof(instruction_node_t));//定义一个节点地址
    strncpy(node->data, data, BUFFER_SIZE - 1);  //复制的最大字符数BUFFER_SIZE - 1，为/0留下空间                 
    node->data[BUFFER_SIZE - 1] = '\0';
    node->next = NULL;//指向下一个节点的指针，初始化NULL

    pthread_mutex_lock(&list->mutex);  //列表的互斥锁
    if (list->tail == NULL) {          //如果链表的尾部指针是NULL
        list->head = list->tail = node;//那么链表的头部和尾部都是这个节点
    } else {
        list->tail->next = node;        //否则尾部节点指向新节点
        list->tail = node;              //新节点作为尾部节点
    }
    list->count++;                      //链表节点数量+1
    pthread_mutex_unlock(&list->mutex);
    pthread_cond_signal(&list->cond);
}
void reset_instruction_list(instruction_list_t* list) {
    pthread_mutex_lock(&list->mutex);

    // 清空链表中的所有节点
    while (list->head != NULL) {                 //头部节点存在
        instruction_node_t* temp = list->head;   //定义temp获得头部节点
        list->head = list->head->next;           //头部节点获得下一节点
        if (temp != NULL) {
            free(temp);
        }
    }
    list->tail = NULL;
    list->count = 0;

    // // 重置 mutex 和 condition variable
    pthread_mutex_unlock(&list->mutex);
    // pthread_mutex_init(&list->mutex, NULL);
    // pthread_cond_init(&list->cond, NULL);
}

void execute_CM(State* state, char* data) {
    // 检查开头是否为'$'和结尾是否为'*&'
    if (data[0] != '$' || data[1] != 'C' || data[2] != 'M' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合控制指令格式\n");
        return;
    }
    state->pathtrack_switch=0;//关闭
    state->zigzag_switch=0;
    // CTRL_DATA_PSI.ErrorSum=0;
    // CTRL_DATA_PSI.rudder_int=0;
    // CTRL_DATA_V.ErrorSum=0;
    // CTRL_DATA_R.ErrorSum=0;
    // memcpy(CTRL_DATA_PSI.FY_1,CTRL_DATA_PSI.FY0,sizeof(CTRL_DATA_PSI.FY0));
    // memcpy(CTRL_DATA_V.FY_1,CTRL_DATA_V.FY0,sizeof(CTRL_DATA_V.FY0));
    // memcpy(CTRL_DATA_R.FY_1,CTRL_DATA_R.FY0,sizeof(CTRL_DATA_R.FY0));
    // 找到第一个和第二个逗号的位置
    char* first_comma = strchr(data, ',');
    char* second_comma = strchr(first_comma + 1, ',');

    // 检查第一个和第二个逗号之间的数字是否表示从第二个逗号到结尾的字符数
    int length = atoi(first_comma + 1);
    int length_c=(int)strlen(second_comma);
    if (length != length_c) {
        printf("长度信息不正确:长度为%d,不等于%d\n", length , length_c);
        return;
    }

    // 去除开头的'$'和结尾的'*&'
    data[0] = ',';
    data[strlen(data) - 2] = '\0';

    // 按逗号分割字符串,并将各部分存储在part[]数组中
    char* part[MAX_PARTS];
    char* ptr = strtok(data, ",");
    int i = 0;
    while (ptr != NULL && i < MAX_PARTS) {
        part[i++] = ptr;
        ptr = strtok(NULL, ",");//传入 NULL，表示继续从上次的位置分割
    }

    // 打印各部分
    // for (int j = 0; j < i; j++) {
    //     printf("Part %d: %s\n", j, part[j]);
    // }
    // printf("1\n");  
    if (strcmp(part[2],"RC") == 0){//遥控模式
        if (strcmp(part[5], "null") == 0 || strcmp(part[5], "NULL") == 0) { //纯调速
            state->vol_d=atof(part[6]);  //获取推进器
            state->controller_switch=0;//关闭
            // printf("退出自动控制状态\n");
        }else{              //需要改变航向策略，退出航迹模式
            state->delta_d=atof(part[5]);//获取舵角
            state->vol_d=atof(part[6]);  //获取推进器
            state->controller_switch=0;//关闭
            // printf("退出自动控制状态\n");
            state->pathtrack_switch=0;//关闭
            // printf("退出航路点跟踪\n");
        }
    }else if(strcmp(part[2],"AC") == 0){//自动控制模式
        if (strcmp(part[3], "null") == 0 || strcmp(part[3], "NULL") == 0) {//判断是纯调速还是角速度跟踪模式
            // printf("不是航向角跟踪模式\n");  
            if (strcmp(part[8], "null") == 0 || strcmp(part[8], "NULL") == 0) {//纯调速
                        state->vd=atof(part[4]);  //获取期望航速
                        state->controller_switch=1;//打开
                        // printf("进入自动控制状态");  
            }else{//角速度跟踪模式
                state->vd=atof(part[4]);  //获取期望航速
                state->rd=atof(part[8]);  //获取期望艏向角速度 
                state->controller_switch=2;//角速度跟踪模式
                // printf("进入角速度跟踪模式\n");  
            }      
        }else{
            state->psid=atof(part[3]);//获取期望角度
            state->vd=atof(part[4]);  //获取期望航速
            state->rd=atof(part[8]);  //获取期望艏向角速度  
            state->controller_switch=1;//开启控制器控制。1表示对psid进行跟踪，2表示对rd进行跟踪
            // printf("进入自动控制状态\n");
            state->pathtrack_switch=0;//关闭航路点跟踪
            // printf("退出航路点跟踪\n");
        }
        }else if(strcmp(part[2],"VC") == 0){
            state->psid=atof(part[3]); //获取期望角度
            state->vol_d=atof(part[6]);  //获取推进器
            state->controller_switch=3;//开启控制器控制。0表示关闭控制模式，1表示对psid进行跟踪，2表示对rd进行跟踪，3表示对psid跟踪，但是推进器靠电压控制
            state->pathtrack_switch=0; //关闭航路点跟踪

    }
}


void execute_ZT(State* state, char* data) {
    if (data[0] != '$' || data[1] != 'Z' || data[2] != 'T' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合Z型试验指令格式\n");
        return;
    }
    state->zigzag_switch=1; //开启z字机动
    state->delta_d=20;      //初始期望舵角为20°
}



void execute_PP(State* state, char* data) {
    // 检查开头是否为'$'和结尾是否为'*&'
    if (data[0] != '$' || data[1] != 'P' || data[2] != 'P' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合航迹指令格式\n");
        return;
    }

    // 找到第一个和第二个逗号的位置
    char* first_comma = strchr(data, ',');
    char* second_comma = strchr(first_comma + 1, ',');

    // 检查第一个和第二个逗号之间的数字是否表示从第二个逗号到结尾的字符数
    int length = atoi(first_comma + 1);
    int length_c=(int)strlen(second_comma);
    if (length != length_c) {
        printf("长度信息不正确:长度为%d,不等于%d\n", length , length_c);
        return;
    }

    // 去除开头的'$'和结尾的'*&'
    data[0] = ',';
    data[strlen(data) - 2] = '\0';

    // 按逗号分割字符串,并将各部分存储在part[]数组中
    char* part[MAX_PARTS];
    char* ptr = strtok(data, ",");
    int i = 0;//一共用i个部分
    while (ptr != NULL && i < MAX_PARTS) {
        part[i++] = ptr;
        ptr = strtok(NULL, ",");
    }   
    pthread_mutex_lock(&Point.lock);
    printf("上锁\n");
    printf("释放航路点\n");
    free_Point(&Point);
    printf("构建航路点中\n");
    init_Point(&Point,(int)((i-3)/3));//去掉数据头，报文长度，航迹点总数，
    printf("构建完毕\n");
    int j=0;
    for (j = 0; j < (int)((i-3)/3); j++){
            Point.x[j]=(atof(part[3*(j+1)+2])-latitude_base)* EARTH_RADIUS* PI / 180.0;//纬度;
            Point.y[j]=(atof(part[3*(j+1)+1])-longitude_base)* EARTH_RADIUS* PI / 180.0 * cos(latitude_base* PI / 180.0);//经度;//经度
            printf("j: %d\n", j);
            printf("Point->x[%d]: %f\n", j, Point.x[j]);
            printf("Point->y[%d]: %f\n", j, Point.y[j]);
    }
    psi_all(&Point);//计算切角

    pthread_mutex_unlock(&Point.lock);
    state->controller_switch=1;//期望控制模式打开
    state->pathtrack_switch=1;//轨迹跟踪模式打开

    // // 打印各部分
    // for (int j = 0; j < i; j++) {
    //     printf("Part %d: %s\n", j, part[j]);
    // }
}


void execute_PA(State* state, char* data) {
    // 检查开头是否为'$'和结尾是否为'*&'
    if (data[0] != '$' || data[1] != 'P' || data[2] != 'A' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合参数更新指令格式\n");
        return;
    }

    // 找到第一个和第二个逗号的位置
    char* first_comma = strchr(data, ',');
    char* second_comma = strchr(first_comma + 1, ',');

    // 检查第一个和第二个逗号之间的数字是否表示从第二个逗号到结尾的字符数
    int length = atoi(first_comma + 1);
    int length_c=(int)strlen(second_comma);
    if (length != length_c) {
        printf("长度信息不正确:长度为%d,不等于%d\n", length , length_c);
        return;
    }

    // 去除开头的'$'和结尾的'*&'
    data[0] = ',';
    data[strlen(data) - 2] = '\0';

    // 按逗号分割字符串,并将各部分存储在part[]数组中
    char* part[MAX_PARTS];
    char* ptr = strtok(data, ",");
    int i = 0;//一共用i个部分
    while (ptr != NULL && i < MAX_PARTS) {
        part[i++] = ptr;
        ptr = strtok(NULL, ",");
    }   

    
    //这是表示这正在索取航向控制器
    if (atoi(part[2])==0) { //航向控制器
        //{PID, PCFDL_MFAC,PPFDL_MFAC,PFFDL_MFAC,FO_CFDL_MFAC,VFO_CFDL_MFAC,ICFDL_MFAC}
        pthread_mutex_lock(&CTRL_DATA_PSI.mutex);
        printf("当前航向控制器为%d\n",CTRL_DATA_PSI.mode);
        CTRL_DATA_PSI.kp=atof(part[3]);
        CTRL_DATA_PSI.ki=atof(part[4]);
        CTRL_DATA_PSI.kd=atof(part[5]);
        CTRL_DATA_PSI.p[0]=atof(part[6]);
        CTRL_DATA_PSI.p[1]=atof(part[7]);
        CTRL_DATA_PSI.p[2]=atof(part[8]);
        CTRL_DATA_PSI.lamda=atof(part[9]);
        CTRL_DATA_PSI.Kr=atof(part[10]);
        CTRL_DATA_PSI.sigma=atof(part[11]);
        CTRL_DATA_PSI.mode=atoi(part[12]);  
        // printf("kp: %f\n", CTRL_DATA_PSI.kp);
        // printf("ki: %f\n", CTRL_DATA_PSI.ki);
        // printf("kd: %f\n", CTRL_DATA_PSI.kd);
        // printf("lamda: %f\n", CTRL_DATA_PSI.lamda);
        // printf("p[0]: %f\n", CTRL_DATA_PSI.p[0]);
        // printf("p[1]: %f\n", CTRL_DATA_PSI.p[1]);
        // printf("p[2]: %f\n", CTRL_DATA_PSI.p[2]);
        // printf("Kr: %f\n", CTRL_DATA_PSI.Kr);
        // printf("sigma: %f\n", CTRL_DATA_PSI.sigma);
        // printf("mode: %d\n", CTRL_DATA_PSI.mode);
        pthread_mutex_unlock(&CTRL_DATA_PSI.mutex);

    }else if(atoi(part[2])==1){//航速控制器
        pthread_mutex_lock(&CTRL_DATA_V.mutex);
        printf("当前航速控制器为%d\n",CTRL_DATA_V.mode);
            CTRL_DATA_V.kp=atof(part[3]);
            CTRL_DATA_V.ki=atof(part[4]);
            CTRL_DATA_V.kd=atof(part[5]);
            CTRL_DATA_V.lamda=atof(part[6]);
            CTRL_DATA_V.p[0]=atof(part[7]);
            CTRL_DATA_V.p[1]=atof(part[8]);
            CTRL_DATA_V.p[2]=atof(part[9]);
            CTRL_DATA_V.mode=atoi(part[12]);    
            pthread_mutex_lock(&CTRL_DATA_V.mutex);

    }else if(atoi(part[2])==2){//角速度控制器
            pthread_mutex_lock(&CTRL_DATA_R.mutex);
            printf("当前艏向角速度控制器为%d\n",CTRL_DATA_R.mode);
            CTRL_DATA_R.kp=atof(part[3]);
            CTRL_DATA_R.ki=atof(part[4]);
            CTRL_DATA_R.kd=atof(part[5]);  
            CTRL_DATA_R.lamda=atof(part[6]);         
            CTRL_DATA_R.p[0]=atof(part[7]);
            CTRL_DATA_R.p[1]=atof(part[8]);
            CTRL_DATA_R.p[2]=atof(part[9]);
            CTRL_DATA_R.mode=atof(part[12]);  
            pthread_mutex_lock(&CTRL_DATA_R.mutex);
    }
}

//无人艇命令接口
void execute_UU(State* state, char* data) {
    if (data[0] != '$' || data[1] != 'U' || data[2] != 'U' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合Z型试验指令格式\n");
        return;
    }
    state->zigzag_switch=0;
    state->controller_switch=1;//控制器控制。0表示关闭控制模式，1表示对psid进行跟踪，2表示对rd进行跟踪，3表示对psid跟踪，但是推进器靠电压控制
    state->pathtrack_switch=1; //开启航路点跟踪
}



//对面发送时间
void execute_TI(State* state, char* data) {
    // 检查开头是否为'$'和结尾是否为'*&'
    if (data[0] != '$' || data[1] != 'T' || data[2] != 'I' || data[strlen(data) - 2] != '*' || data[strlen(data) - 1] != '&') {
        // printf("不符合参数更新指令格式\n");
        return;
    }
    
    // 去除开头的'$'和结尾的'*&'
    data[0] = ',';
    data[strlen(data) - 2] = '\0';

    // 按逗号分割字符串,并将各部分存储在part[]数组中
    char* part[MAX_PARTS];
    char* ptr = strtok(data, ",");
    int i = 0;//一共用i个部分
    while (ptr != NULL && i < MAX_PARTS) {
        part[i++] = ptr;
        ptr = strtok(NULL, ",");
    }   
    // 关闭旧文件（如果已打开）
    if (file_data != NULL) {
        fclose(file_data);
        file_data = NULL;
    }
    // 以 part[1] 作为文件名，追加模式打开

    struct tm tm = {0};
    strptime(part[1], "%Y-%m-%d %H:%M:%S", &tm);
    time_t new_time = mktime(&tm);

    struct timespec ts = {
        .tv_sec = new_time,  // 秒
        .tv_nsec = 0         // 纳秒（可设为 0）
    };
    if (clock_settime(CLOCK_REALTIME, &ts) == 0) {
        printf("系统时间已同步: %s", ctime(&new_time));
    } else {
        perror("系统时间同步失败");
    }

    char filename[256];
    snprintf(filename, sizeof(filename), "data/%s.csv", part[1]); 
    printf("%s\n",filename);
    file_data = fopen(filename, "a");
    if (file_data == NULL) {
        perror("Failed to open file");
        return;
    }
    fprintf(file_data, "\xEF\xBB\xBF");  // UTF-8 BOM
    fprintf(file_data, "时间戳,数字标签,经度,纬度,艏向,角速度,x坐标,向北速度,y坐标,向东速度,期望艏向,期望速度,期望角速度,期望舵角,期望推进器电压,舵角积分补偿,航速,实际舵角,实际推进器电压,艏PG1,艏PG2,艏PG3,角PG1,角PG2,角PG3,速PG1,速PG2,速PG3\n");
}