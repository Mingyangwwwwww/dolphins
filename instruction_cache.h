#ifndef INSTRUCTION_CACHE_H
#define INSTRUCTION_CACHE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include "simulation.h"
#include "./controller.h"
#include "./pathtrack.h"
#include "./device.h"
#define  BUFFER_SIZE 1024
typedef struct instruction_node {
    char data[BUFFER_SIZE];
    struct instruction_node* next;
} instruction_node_t;

typedef struct {
    instruction_node_t* head;
    instruction_node_t* tail;
    int count;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} instruction_list_t;


typedef struct {
    instruction_list_t* list;
    State* state;
    int sockfd;
} ThreadArgs;



extern instruction_list_t list_command;

void execute_CM(State*, char*);
void execute_PP(State*, char*);
void execute_PA(State*, char*);
void execute_TI(State*, char*);
void execute_ZT(State*, char*);
void execute_UU(State*, char*);
void* execute_instructions(void* arg);
void add_instruction(instruction_list_t* list, char* data);
void reset_instruction_list(instruction_list_t* list);


#endif // INSTRUCTION_CACHE_H