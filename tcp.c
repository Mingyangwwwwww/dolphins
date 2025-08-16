
#include "./tcp.h"


char recvbuf[BUFFER_SIZE] = {0};
char sendbuf[BUFFER_SIZE] = {0};

// void *tcp_thread(void *arg){
//     Thread_args* args = (Thread_args*)arg;//将输入的指令传入
//     instruction_list_t* list = args->list;
//     State* state             = args->state;
//     printf("创立收发线程\n");
//     //设置为非阻塞模式
//     int flags = fcntl(sockfd, F_GETFL, 0);
//     fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
//     while(1){
//         recv_msg(list);
//         send_msg(state);  
//         usleep(100000);
//         fflush(stdout);
//     }
//     return NULL;
// }

void*  recv_thread(void *arg ) {
        ThreadArgs* args = (ThreadArgs*)arg;//将输入的指令传入
        instruction_list_t* list_command = args->list;
        int sockfd              = args->sockfd;
        while(1){
        memset(recvbuf, 0, BUFFER_SIZE);
        int ret = recv(sockfd, recvbuf, BUFFER_SIZE, 0);//阻塞模式，断连会返回0
        if (ret <= 0) {
            printf("服务端已断开连接\n");
            close(sockfd);
            sockfd = -1; // 将sockfd重置为-1
            printf("信息接收线程退出\n");
            pthread_exit(NULL);
        }
        if (ret > 0) {//如果通信正常，就添加指令
            if (recvbuf != NULL && strlen(recvbuf) > 0) {
                int i, start = 0;
                char *cmd;
                int len=strlen(recvbuf);
                for (i = 0; i < len; i++) {
                    if (recvbuf[i] == '$') {
                        start = i;
                    } else if (recvbuf[i] == '*' && recvbuf[i+1] == '&') {
                        // 找到一条完整的命令
                        cmd = &recvbuf[start];
                        recvbuf[i+2] = '\0'; // 截断命令字符串
                        add_instruction(list_command, cmd);
                        // printf("添加指令");
                        // printf("from client: %s\n", cmd);
                        i += 2; // 跳过*&
                    }
                }   
            }
        }
        usleep(100000); // 等待100毫秒,再次尝试连接
        }

}


//发信息
void* send_thread(void *arg ) {
        ThreadArgs* args = (ThreadArgs*)arg;//将输入的指令传入
        State* state = args->state;
        int sockfd   = args->sockfd;
        while (1) {
            memset(sendbuf, 0, BUFFER_SIZE);
            construct_ST(state, sendbuf);//将无人艇的状态发送给基站
            int ret = send(sockfd, sendbuf, strlen(sendbuf), 0);
            // printf("sendbuf: %s\n", sendbuf);
            if (ret <= 0) {
                    printf("信息发送失败，准备退出信息发送线程\n");
                    pthread_exit(NULL);
                    
            }
            usleep(100000); // 等待100毫秒,再次尝试连接
        }
    
}

void construct_ST(State *state, char buff[]) {
    // 1. 先生成一个临时的 buffer 用于存储数据
    char temp_buff[BUFFER_SIZE];
    int len = 0;
    // 2. 写入状态数据到临时 buffer
    len += snprintf(temp_buff , sizeof(temp_buff), "%d,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f*&",     
                   state->math_label,       //数据标签
                   state->longitude,        //经度
                   state->latitude,         //纬度
                   state->USV[0][0],        //艏向
                   state->yaw,              //航向
                   state->USV[1][0],        //角速度
                   state->delta,            //舵角
                   state->delta_d,          //控制指令
                   state->delta_c,          //积分量补正
                   state->USV_velocity,     //速度
                   state->vol,              //推力信号
                   state->vol_d,            //控制指令
                   CTRL_DATA_PSI.FY_1[0],   //伪梯度
                   CTRL_DATA_PSI.FY_1[1],   //伪梯度
                   CTRL_DATA_PSI.FY_1[2]    //伪梯度
                   );
                   
                
    // 3. 找到第一个逗号的位置
    char *first_comma = strchr(temp_buff, ',');
    int first_comma_index = first_comma - temp_buff;

    // 4. 计算第一个逗号到结尾的长度
    int char_count = len - first_comma_index;

    // 5. 将数据写入最终的 buff 中
    snprintf(buff, BUFFER_SIZE, "$ST,%d,%s", char_count, temp_buff);
}