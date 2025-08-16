#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);//设置串口
int open_port(int fd,int comport);                                //打开串口，第一个是文件号，第二个是串口号，把串口号绑定文件号


#endif