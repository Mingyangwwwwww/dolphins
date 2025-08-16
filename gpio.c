
#include "./gpio.h"

#define DEV_GPIO "/dev/atmel_gpio"
#define	AT91_PIN_PC18	(0x40 + 18)
#define AT91_PIN_PB23	(0x20 + 23)
#define CARD_SWITCH	AT91_PIN_PC18
#define GPIO_IOC_MAGIC   'G'
#define IOCTL_GPIO_SETOUTPUT              _IOW(GPIO_IOC_MAGIC, 0, int)       
#define IOCTL_GPIO_SETINPUT               _IOW(GPIO_IOC_MAGIC, 1, int)
#define IOCTL_GPIO_SETVALUE               _IOW(GPIO_IOC_MAGIC, 2, int) 
#define IOCTL_GPIO_GETVALUE    		      _IOR(GPIO_IOC_MAGIC, 3, int)
typedef struct {
	int pin;
	int data;
	int usepullup;
}at91_gpio_arg;

//static char pins[] = {
//	PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23, PC30, PA30, PA31 
//};


 int out_gpio(char argv[32],int argc,int fd_gpio)
{
	at91_gpio_arg arg,arg2,arg_arr[11];
	//int fd = -1;
	int pin_index = 0,temp,dat;
	char varg[4];
	/// str => pin_index
	if ((argc != 0)&&(argc != 1))
	{
		printf("%d PXX eror",argv[0]);
		return -1;
	}
	strcpy(varg,argv);
	if (varg[0] != 'P')
	{
		return -1;
	}
	switch(varg[1])
	{
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
			break;
		default:
			printf("%d PXX on/off\n",argv[0]);
			return -1;
	}
	pin_index = (varg[1] - 0x41)*0x20;
	
	temp = atoi(&varg[2]);
	if (temp > 31)
	{
		//printf("%s PXX on/off\n",argv[0]);
		return -1;
	}
	
	pin_index += temp;
	//printf("pin_index = %d\n",pin_index);

	
		dat = argc;
		if (!(dat == 0 || dat == 1))
		{
			return -1;
		}
		//printf("data = %d\n",dat);
		
	//fd = open(DEV_GPIO, O_RDWR);
	
	
	if (argc == 0||argc == 1) 
	{
		arg.pin = pin_index ;
		arg.data = argc;
		arg.usepullup = 1;
		ioctl(fd_gpio, IOCTL_GPIO_SETOUTPUT, &arg);
	}

/*
	while(1) {
		arg.pin = CARD_SWITCH;
		arg.data = 0;
		ioctl(fd, IOCTL_GPIO_GETVALUE, &arg);
		//printf("get value: 0x%x\n", arg.data);
		sleep(1);
	}
*/
//	close(fd);
	return 0;
}


//遍历引脚列表，将每个引脚设置为输出模式，并根据 buff_gpio 数组的值（字符格式的 '0' 或 '1'）输出低电平（0V）或高电平（3.3V/5V）。
void gpio()
{
	memset(buff_gpio, 0, 1024 * sizeof(char));
	strcpy(buff_gpio, "00000000000");//锟诫海锟斤拷1锟斤拷一锟斤拷
	char *pins[] = {"PA16", "PA31", "PA18", "PA19", "PA20", "PA21", "PA22", "PA23", "PC30", "PA30", "PA17"};
    at91_gpio_arg arg,arg2,arg_arr[11];
	int fd_gpio = -1;	
	int i;
  char gpio_pin[8];
  int gpio_dat;
	fd_gpio = open(DEV_GPIO, O_RDWR);
	if (fd_gpio < 0) 
	{
		printf("Error device open fail! %d\n", fd_gpio);
		return -1;
	}

       for (i = 0;i < 11;i++)
	   {            //总共11个gpio开关
       //gpio_pin = pins[i];
		   strcpy(gpio_pin,pins[i]);
        //arg.pin_dir = AT91PIO_DIR_OUT;

        gpio_dat = buff_gpio[i] - 48;
       out_gpio(gpio_pin,gpio_dat,fd_gpio);
	   }
	printf("GPIO配置完毕\n");
	close (fd_gpio);

	return;
}


int ADC()
{
	char *device = "/dev/at91_spi2adc0";
	char *device1 = "/dev/at91_spi2adc1";
	int fd;
	int fd1;
	unsigned short data;
	unsigned short data1;
	fd = open(device, O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "Error opening %s: %s\n", device, strerror(errno));
		exit(1);
	}
	fd1 = open(device1, O_RDONLY);
	if (fd1 == -1) {
		fprintf(stderr, "Error opening : %s\n",  strerror(errno));
		exit(1);
	}

	read (fd, &data, sizeof(data));
	read (fd1, &data1, sizeof(data1));


	adc_0 = (float)((long)(data*3300)/4096)/1000;
	adc_1 = (float)((long)(data1*3300)/4096)/1000;
	//printf("%s data: 0x%.4x Voltage: %0.4f V\n",device, data, (float)((long)(data*3300)/4096)/1000);
	//printf("%s data: 0x%.4x Voltage: %0.4f V\n",device1, data1, (float)((long)(data1*3300)/4096)/1000);
	close(fd);
	close(fd1);

	return 0;
}











