#include "./tcp.h"
#include "./simulation.h"
#include "./pathtrack.h"
#include "./gpio.h"

#define SERVER_PORT 8888               //定义服务器端口
#define SERVER_IP "192.168.17.150"     //定义服务器地址

int main(void) {
    printf("客户端开始运行\n");
 
    /////////////////////////////////初始化线程结构体///////////////////////////////
    State state ;//初始化当前无人艇状态
    memset(&state, 0, sizeof(State));
    // 显式初始化互斥锁（默认属性，初始为“开”状态）
    if (pthread_mutex_init(&state.mutex, NULL) != 0) {
        perror("Failed to initialize mutex");
        exit(EXIT_FAILURE);
    }
    printf("初始化线程结构体已完成\n");

    //命令清单，在头文件中初始化一个全局变量时，如果多个源文件包含了该头文件，每个源文件都会有一个该变量的副本，这会导致链接时的问题（如多重定义错误）
    instruction_list_t list_command = {
    .head = NULL,                      //头指针
    .tail = NULL,                      //尾指针
    .count = 0,                        //记录链表中节点的数量
    .mutex = PTHREAD_MUTEX_INITIALIZER,//PTHREAD_MUTEX_INITIALIZER是一个宏，用于静态地初始化互斥锁
    .cond = PTHREAD_COND_INITIALIZER   //PTHREAD_COND_INITIALIZER是一个宏，用于静态地初始化条件变量
    };

    //通过结构体进行线程函数的传递
    ThreadArgs thread_args;
    thread_args.state=&state;      //传递信息
    thread_args.list=&list_command;//缓冲命令
    thread_args.sockfd=-1;         //通信状态

    /////配置硬件////////////
    gpio();
    fd_initialize();

    ///////////////////////////仿真线程////////////////////////////////
    pthread_t tid_sim;                                         //创立线程的标识符，模拟UUV真实运动
    pthread_create(&tid_sim, NULL, thread_simulation, &state);// 创建线程
    printf("初始化传感与控制线程已完成\n");                    

    ////////////////////////////数采线程///////////////////////////////
    pthread_t tid_AD;
    pthread_create(&tid_AD, NULL, AD_thread, NULL);          //创建数采线程
    printf("数采线程初始化已完成\n");

    ////////////////////////////指令处理线程/////////////////////////////
    pthread_t tid_cammand;  //一旦有指令引入就立即执行
    //指令处理线程，需要无人艇当前的状态，和当前缓冲区的命令
    pthread_create(&tid_cammand, NULL, execute_instructions, &thread_args);
    printf("初始化指令处理线程已完成\n");

    ////////////////////////////通信开始/////////////////////////////////
    //创建通信线程传递结构体的套接字
    thread_args.sockfd= socket(AF_INET, SOCK_STREAM, 0);//AF_INET：指定地址族为 IPv4。SOCK_STREAM：指定套接字类型为流式套接字。即TCP
    // int flags = fcntl(thread_args.sockfd, F_GETFL, 0);  //获取thread_args.sockfd 套接字描述符当前的文件状态标志
    // fcntl(thread_args.sockfd, F_SETFL, flags & ~O_NONBLOCK);//F_SETFL是fcntl 函数的一个操作命令，用于设置文件描述符的标志位。清除（即关闭）O_NONBLOCK 标志位；
    //上述代码关闭非阻塞模式，非阻塞模式下，如果操作无法立即完成（如读操作没有数据可读），函数会立即返回，而不是阻塞等待。
    if (0 > thread_args.sockfd) {//thread_args.sockfd为套接字描述符，大于0表示创建套接字成功
        perror("创建套接字失败\n");
        exit(EXIT_FAILURE);
    }

    //创建结构体
    struct sockaddr_in server_addr;                     //struct sockaddr_in 结构体用于表示一个互联网地址和端口号。
    memset(&server_addr, 0, sizeof(server_addr));       //将内存区域设置为0，初始化结构体
    server_addr.sin_family = AF_INET;                   // IPV4     
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP); // 连接到服务器地址
    server_addr.sin_port = htons(SERVER_PORT);          // 端口

    //仅作声明
    pthread_t recv_tid;
    pthread_t send_tid;
    printf("通信初始化已完成\n");

while (1) { // 如果没有连接上服务器就一直尝试连接
    //将套接字描述符 thread_args.sockfd 与 指向服务器地址信息的指针 绑定在一起
    //struct sockaddr_in是struct sockaddr的一个具体实现（或称为派生类型），用于IPv4网络通信，因此需要 (struct sockaddr *)强制转化
    printf("断线重连中\n");
    if (connect(thread_args.sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == 0) {//连接成功
        printf("连接服务器成功\n");
        //////////////////创立新线程////////////////
        pthread_create(&recv_tid, NULL, recv_thread, &thread_args);//创建tcp线程
        pthread_create(&send_tid, NULL, send_thread, &thread_args);//创建tcp线程   
        // 等待收发线程退出
        pthread_join(recv_tid, NULL);
        pthread_join(send_tid, NULL);
        printf("连接中断，通信线程已退出\n");
        //如果断连，就立刻停止操控无人艇
        pwm_rudder(0.0);              //舵角归0
        pwm_thrust(0.0);               //推进器归0
        memset(&state,0,sizeof(state));//重置无人艇状态
        printf("舵角已调整，推进器已关闭\n");


    } else {//连接失败了
        printf("连接失败\n");
        //重新创立套接字
        thread_args.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        // int flags = fcntl(thread_args.sockfd, F_GETFL, 0);
        // fcntl(thread_args.sockfd, F_SETFL, flags & ~O_NONBLOCK);
        usleep(1000000); // 等待1000毫秒,再次尝试连接
        printf("已重新创建套接字\n");
    }
}
    //退出
    printf("程序退出\n");

    // 等待执行线程退出
    pthread_join(tid_cammand, NULL);
    pthread_join(tid_sim, NULL);
    pthread_join(tid_AD, NULL);
    fd_cleanup();
    printf("串口已关闭\n");
    //关闭套接字
    close(thread_args.sockfd);
    exit(EXIT_SUCCESS);
}