# CROSS_COMPILE ?= arm-linux-gnueabihf-
CROSS_COMPILE = arm-none-linux-gnueabi-
CC = $(CROSS_COMPILE)gcc  
LD = $(CROSS_COMPILE)ld  

OBJCOPY = $(CROSS_COMPILE)objcopy  
OBJDUMP = $(CROSS_COMPILE)objdump  

TARGET = yang

# 定义构建目录
BUILD_DIR = build

# 源文件定义
CFILES = $(wildcard *.c)  
HFILES = $(wildcard *.h)  
# 对象文件放在构建目录中
OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CFILES))  
DEP_FILES = $(OBJS:.o=.d)  

INCLUDE = -I.  

# CFLAGS = -Wall  -O2 $(INCLUDE) -std=gnu99 -MMD -MP  
CFLAGS = -Wall  -O2 $(INCLUDE)  -MMD -MP  
LDFLAGS = -lpthread -lm  -lrt
LDFLAGS += -Wl,-dynamic-linker,/lib/ld-linux.so.3  # 两个编译器不太一样，无人艇板子用这个和none编译器，仿真板子不要加这一行

all: $(TARGET)  

# 确保构建目录存在
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)  
	$(CC) $(OBJS) $(LDFLAGS) -o $@  

# 编译并生成依赖文件（在构建目录中）
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@  

# 包含自动生成的依赖文件  
-include $(DEP_FILES)  

clean:  
	rm -f $(TARGET)
	rm -rf $(BUILD_DIR)

.PHONY: all clean