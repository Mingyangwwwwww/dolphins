#ifndef GPIO_H
#define GPIO_H


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <linux/ioctl.h>

 
float adc_0;
float adc_1;
float adc_Pv[11];
float adc_Li[11];

char buff_gpio[1024];
char buff_ctd[1024];
char Command[16];
float PA[6];
char strPA[10][256];
char strPC[8][256];
void gpio();

#endif // !GPIO_H