#ifndef TCP_H
#define TCP_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "./instruction_cache.h"
#include <fcntl.h>

extern char recvbuf[BUFFER_SIZE];
extern char sendbuf[BUFFER_SIZE];
extern char buff_memory[BUFFER_SIZE];


void construct_ST(State *state, char buff[]);
void *send_thread(void *arg) ;
void *recv_thread(void *arg) ;
// void *tcp_thread(void *arg);
#endif // TCP_H