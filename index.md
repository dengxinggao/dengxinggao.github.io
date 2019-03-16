# Nuttx Getting Started

## 建立编译环境

### 1.安装构建编译工具
安装编译器，依赖库，调试工具，以及串口终端
~~~sh
sudo apt install automake bison build-essential flex gcc-arm-none-eabi gperf git libncurses5-dev libtool pkg-config openocd minicom
~~~
### 2.安装nuttx tools
获取nuttx tools，配置，编译，安装
~~~sh
git clone https://bitbucket.org/nuttx/tools
cd tools/kconfig-frontends/
./configure
make
sudo make install
sudo ldconfig
~~~
### 3.获取nuttx和apps
~~~sh
git clone https://bitbucket.org/nuttx/nuttx
git clone https://bitbucket.org/nuttx/apps
~~~
### 4.编译测试
配置一个nuttx工程
~~~sh
cd nuttx
./tools/configure.sh stm32f103-minimum/nsh
make menuconfig
make
~~~
### 5.烧写固件
使用openocd烧写固件
~~~sh
openocd -f interface/jlink.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
~~~
interface/jlink.cfg是下载工具配置

target/stm32f1x.cfg是目标芯片配置

以上openocd配置脚本存放在/usr/share/openocd/scripts/，可根据自己情况选择使用配置，也可以修改。
终端中Ctrl+R，可以快速搜索执行过的命令

### 6.测试
打开minicom，连接串口线，重启mcu，即在minicom中看到nuttx nsh信息
~~~sh
NuttShell (NSH)
nsh> echo hello nuttx
hello nuttx
~~~
dmesg 查看新接入usb串口号

minicom 中 Ctrl+A 然后 Z 跳出设置菜单

或直接`sudo minicom -s`设置串口号等

## 增加自己的板配置
### 1.复制相似的配置文件夹
板配置文件在nuttx/configs文件夹内，复制和自己板mcu相近板级配置文件夹，我的板上mcu是stm32f405rg芯片,复制stm32f4discovery文件夹，更名为**mysmt32f405rg**。
### 2.清理文件夹
保留include、nsh、scripts、src文件夹，删除其他所有
### 3.修改脚本文件
由于我们只用的是自定义板，所以修改scripts文件夹中Make.defs文件
~~~C
$(CONFIG_ARCH_BOARD)
~~~
修改为
~~~c
$(CONFIG_ARCH_BOARD_CUSTOM_NAME)
~~~
ld.script为链接脚本，我们可以根据自己mcu修改flash和sram的大小
~~~script
MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    sram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}
~~~
### 4.重新配置
在Nuttx路径执行
~~~sh
./tools/configure.sh stm32f405rg/nsh
make menuconfig
~~~
`Build Setup  --->Build Host Platform (Linux)`

`System Type  --->ARM MCU selection (STMicro STM32 F1/F2/F3/F4/L1)`

`STM32 Chip Selection `(**STM32F405RG**)

支持的芯片在nuttx/code/nuttx/arch/arm/include/stm32/chip.h

`STM32 Peripheral Support`  --->选择需要使用的外设，这里仅开启`[*] USART1`

`Boot Memory Configuration`  --->修改mcu sram配置

`Board Selection  --->Select target board   --->(X) Custom development board`

`Custom Board Configuration  --->Custom board name --->`**mystm32f405rg**

`Custom board directory ---> `configs/**mystm32f405rg**

`Device Drivers ---> [*] Serial Driver Support  --->Serial console (USART1) `启用串口驱动
`USART1 Configuration` 中配置串口

保存退出

### 5.根据自己情况修改源代码
可能由于nsh使用的串口不同需要修改串口引脚定义
~~~c
#define GPIO_USART1_RX GPIO_USART1_RX_1
#define GPIO_USART1_TX GPIO_USART1_TX_1
~~~
引脚重定义可在nuttx/arch/arm/src/stm32/chip/stm32f40xxx_pinmap.h文件中找到对应。
### 6.编译烧写测试
~~~sh
make
openocd -f interface/jlink.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
~~~
## 自定义led驱动
### 1.增加源文件
在configs/**mystm32f405rg**/src/文件夹中新建stm32_leds.c
~~~c
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>

#include "stm32_gpio.h"

#define GPIO_LED0 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3)
#define GPIO_LED1 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN6)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)

typedef FAR struct file		file_t;

static int     leds_open(file_t *filep);
static int     leds_close(file_t *filep);
static ssize_t leds_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t leds_write(file_t *filep, FAR const char *buf, size_t buflen);

static const struct file_operations leds_ops = {
	leds_open,		/* open */
	leds_close,		/* close */
	leds_read,		/* read */
	leds_write,		/* write */
	0,			/* seek */
	0,			/* ioctl */
};
/****************************************************************************
 * HW access
 ****************************************************************************/

/****************************************************************************
 * LEDs: 文件操作 定义和结构体
 ****************************************************************************/

typedef FAR struct file		file_t;

static int     leds_open(file_t *filep);
static int     leds_close(file_t *filep);
static ssize_t leds_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t leds_write(file_t *filep, FAR const char *buf, size_t buflen);

static const struct file_operations leds_ops = {
	leds_open,		/* open */
	leds_close,		/* close */
	leds_read,		/* read */
	leds_write,		/* write */
	0,			/* seek */
	0,			/* ioctl */
};

/****************************************************************************
 * leds: 文件操作
 ****************************************************************************/

static int leds_open(file_t *filep)
{
	/* Nothing to do here, maybe I should increase a counter like for Linux driver? */

	return OK;
}

static int leds_close(file_t *filep)
{
	/* Nothing to do here, maybe I should decrease a counter like for Linux driver?*/

	return OK;
}

static ssize_t leds_read(file_t *filep, FAR char *buf, size_t buflen)
{
	register uint8_t reg;

	if(buf == NULL || buflen < 1)
		/* Well... nothing to do */
		return -EINVAL;

	/* These LEDs are actived by low signal (common anode), then invert signal we read*/
	reg = stm32_gpioread(GPIO_LED2);reg = reg << 1;
	reg = reg | (stm32_gpioread(GPIO_LED1));reg = reg << 1;
	reg = reg | (stm32_gpioread(GPIO_LED0));

	buf[0] = (char) (reg + '0');
        buf[1] = '\n';

	return 2;
}

static ssize_t leds_write(file_t *filep, FAR const char *buf, size_t buflen)
{
	register uint8_t reg;

	if(buf == NULL || buflen < 1)
		/* Well... nothing to do */
		return -EINVAL;

	reg = (uint8_t) *buf;

        if (reg < '0' || reg > '7')
	{
		return -EINVAL;
	}

	stm32_gpiowrite(GPIO_LED0, (reg - '0')&0x01);
	stm32_gpiowrite(GPIO_LED1, (reg - '0')&0x02);
	stm32_gpiowrite(GPIO_LED2, (reg - '0')&0x04);

	return 1;
}


/****************************************************************************
 * 初始化设备, 添加 /dev/... 节点
 ****************************************************************************/

int up_leds(void)
{
  int ret;

  stm32_configgpio(GPIO_LED0);
  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);

  ret = register_driver("/dev/leds", &leds_ops, 0444, NULL);
  if (ret < 0)
  {
    _err("Failed to register driver: %d\n", ret);
  }
  return ret;
}
~~~
### 2.将文件包含到makefile
在CSRCS 追加 stm32_leds.c
~~~c
CSRCS = stm32_boot.c stm32_bringup.c stm32_leds.c
~~~
### 3.初始化调用
在stm32_bringup.c文件stm32_bringup函数中调用leds初始化
~~~c
int stm32_bringup(void)
{
  int ret = OK;
  up_leds();
  return ret;
}
~~~
### 4.编译，烧写，运行，测试
运行后在/dev下即可看到leds设备,可以通过 echo dd 等命令操作leds。
~~~sh
nsh> cd dev
nsh> ls
/dev:
 console
 leds
 null
 ttyS0
nsh> echo 7 > /dev/leds
nsh> echo 0 > /dev/leds
nsh> dd if=/dev/leds of=/dev/console bs=2 count=1
0
nsh>
~~~
## 新增自己的APP程序
### 1.复制Hello应用文件夹
复制apps/examples/hello/ 重命名为 **myapp**
### 2.修改Kconfig配置
~~~
config EXAMPLES_HELLO
	tristate "\"Hello, World!\" example"
	default n
	---help---
		Enable the \"Hello, World!\" example

if EXAMPLES_HELLO

config EXAMPLES_HELLO_PROGNAME
	string "Program name"
	default "hello"
	depends on BUILD_LOADABLE
	---help---
		This is the name of the program that will be use when the NSH ELF
		program is installed.

config EXAMPLES_HELLO_PRIORITY
	int "Hello task priority"
	default 100

config EXAMPLES_HELLO_STACKSIZE
	int "Hello stack size"
	default 2048

endif
~~~
直接使用替换，修改配置名称**EXAMPLES_HELLO**为**EXAMPLES_MYAPP**，以及说明。
~~~
config EXAMPLES_MYAPP
	tristate "\"My App!\" example"
	default n
	---help---
		Enable the \"My App!\" example

if EXAMPLES_MYAPP

config EXAMPLES_MYAPP_PROGNAME
	string "Program name"
	default "myapp"
	depends on BUILD_LOADABLE
	---help---
		This is the name of the program that will be use when the NSH ELF
		program is installed.

config EXAMPLES_MYAPP_PRIORITY
	int "Myapp task priority"
	default 100

config EXAMPLES_MYAPP_STACKSIZE
	int "Myapp stack size"
	default 2048

endif
~~~
### 3.修改Makefile
根据Kconfig配置修改
~~~
-include $(TOPDIR)/Make.defs

# Hello, World! built-in application info

CONFIG_EXAMPLES_HELLO_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_HELLO_STACKSIZE ?= 2048

APPNAME = hello

PRIORITY  = $(CONFIG_EXAMPLES_HELLO_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_HELLO_STACKSIZE)

# Hello, World! Example

ASRCS =
CSRCS =
MAINSRC = hello_main.c

CONFIG_EXAMPLES_HELLO_PROGNAME ?= hello$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_HELLO_PROGNAME)

MODULE = CONFIG_EXAMPLES_HELLO

include $(APPDIR)/Application.mk
~~~
替换**EXAMPLES_HELLO**为**EXAMPLES_MYAPP**
替换**hello**为**myapp**

~~~
-include $(TOPDIR)/Make.defs

# My App! built-in application info

CONFIG_EXAMPLES_MYAPP_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_MYAPP_STACKSIZE ?= 2048

APPNAME = myapp

PRIORITY  = $(CONFIG_EXAMPLES_MYAPP_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_MYAPP_STACKSIZE)

# My App! Example

ASRCS =
CSRCS =
MAINSRC = myapp_main.c

CONFIG_EXAMPLES_MYAPP_PROGNAME ?= myapp$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_MYAPP_PROGNAME)

MODULE = CONFIG_EXAMPLES_MYAPP

include $(APPDIR)/Application.mk
~~~
### 4.修改Make.defs
根据以上配置修改
~~~
ifneq ($(CONFIG_EXAMPLES_HELLO),)
CONFIGURED_APPS += examples/hello
endif
~~~
替换**EXAMPLES_HELLO**为**EXAMPLES_MYAPP**
替换**hello**为**myapp**

~~~
ifneq ($(CONFIG_EXAMPLES_MYAPP),)
CONFIGURED_APPS += examples/myapp
endif
~~~
### 5.修改.c文件
重命名`hello_main.c`为 **myapp**_main.c
~~~c
/****************************************************************************
 * hello_main
 ****************************************************************************/
#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
  printf("Hello, World!!\n");
  return 0;
}
~~~
修改函数名`hello_main`为`myapp_main`
~~~c
/****************************************************************************
 * myapp_main
 ****************************************************************************/

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int myapp_main(int argc, char *argv[])
#endif
{
  printf("Hello, My App!!\n");
  return 0;
}
~~~
### 6.启用APP
~~~
make menuconfig
~~~
`Application Configuration  ---> Examples`
找到，并启用` [*] "My App!" example`
保存退出

### 7.编译，烧写，运行，测试
运行后可以看到 Builtin Apps 中有 myapp，输入后看到执行效果
~~~sh
nsh> ?
help usage:  help [-v] [<cmd>]

  [         cp        exec      kill      mv        set       uname
  ?         cmp       exit      ls        mw        sh        umount
  basename  dirname   false     mb        ps        sleep     unset
  break     dd        free      mkdir     pwd       test      usleep
  cat       df        help      mh        rm        time      xd
  cd        echo      hexdump   mount     rmdir     true

Builtin Apps:
  myapp
nsh> myapp
my app! Hello, World!!
nsh>
~~~
### 8.App中操作leds
修改myapp_main.c，增加leds操作
~~~c
#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int myapp_main(int argc, char *argv[])
#endif
{
  int fd;
  uint8_t num[2];

  num[0]='0';
  num[1]=0;
  fd = open("/dev/leds",O_WRONLY);
  while(1)
  {
    num[0]++;
    if(num[0]>'7')num[0]='0';
    write(fd,num,2);
    usleep(500000);
  }
  close(fd);
  return 0;
}
~~~

编译后 在nsh执行myapp，可以看到LED灯闪烁

~~~sh
nsh> myapp
~~~

nsh 命令 可以 用 myapp& 后台执行
~~~sh
nsh> myapp &
myapp[4:00]
nsh>
~~~
后台运行功能可以关闭

`Application Configuration  --->NSH Library  --->Command Line Configuration  --->[*] Disable background commands`

## 自启动脚本

上节的myapp必须通过nsh调用才能执行，可以通过启动脚本，让其自动运行
### 1.安装genromfs
~~~sh
sudo apt install genromfs
~~~
### 2.启用romfs支持
~~~sh
make menuconfig
~~~
启用rmfs

`File Systems  ---> [*] ROMFS file system`

启用romfs启动脚本

`Application Configuration  ---> NSH Library  ---> Scripting Support  ---> [*] Support ROMFS start-up script`

设置romfs头文件路径到板配置文件夹

`ROMFS header location (Default ROMFS header path)  ---> (X) Architecture-specific ROMFS path`

### 3.生成romfs.h

创建 `etc/init.d/rcS` 文件

编辑rcS文件

~~~sh
echo "hello"
myapp &
~~~
保存

在etc上级文件夹执行

~~~sh
genromfs -f romfs_img -d etc
xxd -i romfs_img >nsh_romfsimg.h
~~~
生成nsh_romfsimg.h，复制到configs/**mystm32f405**/include/中。

### 4.编译，烧写，运行，测试

重启后看到LED闪烁，myapp已运行。

~~~sh
hello
myapp [3:100]

NuttShell (NSH)
nsh>
~~~

## Nuttx板级初始化函数

从`stm32_bringup.c` `stm32_boot.c`  `stm32_appinit.c` 几个文件可以看出

`stm32_boardinitialize()`无条件首先调用，此函数由MCU初始化函数`up_initialize()`调用，此时操作系统还初始化，无法使用操作系统资源。

若通过`RTOS Features > RTOS hooks > [*] Custom board late initialization`，定义`CONFIG_BOARD_LATE_INITIALIZE`，`board_late_initialize()`调用`stm32_bringup()`。此函数在MCU初始化后调用，能使用的系统资源很少。

若未定义`CONFIG_BOARD_LATE_INITIALIZE`，通过`Application Configuration > NSH Library > [*]   Have architecture-specific initialization`定义`CONFIG_NSH_ARCHINIT`，启用了`Board Selection > [*] Enable boardctl() interface`定义`CONFIG_LIB_BOARDCTL`，则`stm32_bringup()`通过`NSH library`启动`board_app_initialize()`调用。能使用操作系统资源。

## 通过Nsh查看系统线程任务的办法

Nuttx支持挂载 procfs文件系统 /proc，使用 `ps free`查看系统任务，内存信息等。

### 1.启用 PROCFS文件系统支持

menuconfig 中启用

`File Systems > [*] PROCFS File System`

编译，烧写，运行后，执行挂载命令`mount -t procfs /proc`后，便可查看线程任务内存等信息。

~~~sh
nsh> mount -t procfs /proc
nsh> ls
/:
 dev/
 etc/
 proc/
nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK    CPU COMMAND
    0   0 FIFO     Kthread N-- Ready              00000000 000000   0.0% Idle Task
    2 100 FIFO     Task    --- Running            00000000 002028   0.0% init
    3 100 RR       Task    --- Waiting  Signal    00000000 002028   0.0% myapp
nsh> free
             total       used       free    largest
Umem:       190288      10880     179408     122560
nsh>
~~~

若无法看到CPU使用率，需要在menuconfig中启用

`RTOS Features > Performance Monitoring > [*] Enable CPU load monitoring `

### 2.自动挂载办法

还可以通过初始化代码，自动挂载/proc

nuttx/config/**mystm32f405rg**/src/中mystmf405rg.h添加

~~~c
#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif
~~~

在 stm32_bringup.c 中`stm32_bringup()`添加 

~~~c
#ifdef CONFIG_FS_PROCFS
/* Mount the procfs file system */
ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
if (ret < 0)
{
    serr("ERROR: Failed to mount procfs at %s: %d\n",STM32_PROCFS_MOUNTPOINT, ret);
}
#endif
~~~

重新编译、烧写、运行后，可以直接使用`ps`，`free`等命令。

## Nuttx初始化流程

宏观来看，Nuttx初始化流程可以表示为如下三个步骤：

1. 特定的硬件上电复位初始化，
2. Nuttx系统初始化，
3. 应用程序初始化。

Nuttx初始化流程真的很简单，因为系统在应用程序启动入口前，是以单线程模式运行。这意为着初始化流程只是简单的直线的函数调用。

在启动应用程序之前，系统进入多线程模式，事情才会变得更加复杂。

以下各段将更详细地讨论这些中的每一个。

### 上电复位初始化

处理器复位后，软件开始执行。这通常是在通电时，但无论是因为上电，按下复位按钮还是看门狗定时器到期，所有复位都基本相同。处理器复位时执行的软件对于特定的CPU架构是唯一的，并且不是NuttX的常见部分。特定构架的CPU复位操作必须完成的事情包括：

1. 将处理器置于运行状态。这可能包括设置CPU模式;初始化协处理器等。
2. 设置时钟以使软件和外设按预期运行，
3. 设置C堆栈指针（和其他处理器寄存器），
4. 初始化内存，和
5. 启动Nuttx

#### 初始化内存

在C实现中，存在两种通用的变量存储类。首先是初始化变量。比如，研究全局变量x:

~~~c
int x = 5;
~~~

C代码必须确保复位后,`x`的值为`5`,这种初始化变量被分配在名为`data`（或`.data`）的指定内存段。

其他不用初始化的变量，比如全局变量`y`：

~~~c
int y;
~~~

但是C代码仍然期望`y`具有初始值。该初始值将为零。此类型的所有未初始化变量的值都为零。这些未初始化的变量分配在名为`bss`（或`.bss`）的内存段中。

当我们说复位处理逻辑初始化内存时，我们指两件事：

1. 它通过将FLASH中的值复制到`.data`段中来提供初始化变量的（初始）值，和
2. 它将所有不需要初始化的变量重置为零。它清零`.bss`段。

#### STM32 F4 复位

让我们来看一个特定处理器的复位流程。我们来看看STM32 F4 MCU的NuttX初始化。这个逻辑逻辑可以在下面两个文件中找到：

1. nuttx/arch/arm/src/stm32_vectors.S
2. nuttx/arch/arm/src/stm32_start.c

##### nuttx/arch/arm/src/stm32_vectors.S

`stm32_vectors.S`在复位流程中很小。该文件提供了所有STM32异常向量，但上电复位只是另一个异常向量。关于此文件需要注意的一些重要事项：

1. `.section` `.vectors` `“ax”`. 这个伪操作将所有向量放入一个特殊的段调用`.vectors`。STM32 F4链接器位于`nuttx/configs/stm3240g-eval/nsh/ld.script`。在该文件中，您可以看到`.vectors`部分被强制位于FLASH内存的最开头。STM32 F4可配置为以不同方式启动。如果配置为从FLASH启动，那么当复位发生时，STM32 FLASH存储器将被别名为地址0x0000 0000。这是上电复位中断向量的地址。

2. 向量表中的前两个32位入口表示上电异常向量（当发生复位时，我们知道它将位于地址0x0000 0000）。这两个入口是：

   ~~~c
   .word IDLE_STACK /* Vector  0: Reset stack pointer */
   .word __start    /* Vector  1: Reset vector */
   ~~~

Cortex-M系列在处理复位向量方面是独一无二的。请注意，有两个值：启动线程的堆栈指针（IDLE线程）和IDLE线程中的入口点。发生复位时，堆栈指针自动设置为第一个值，然后处理器跳转到第二个条目中指定的复位入口点`__start`。

##### nuttx/arch/arm/src/stm32_start.c

复位向量`__start`位于文件`stm32_start.c`中，执行真正的低级别体系结构特定的初始化。此初始化包括：

1. `stm32_clockconfig();`初始化电路板所需的PPL和外设时钟。

2. `stm32_fpuconfig();` 如果初始化STM32 F4的硬件浮点，则配置FPU并启用对FPU协处理器的访问。

3. `stm32_lowsetup();`启用低级UART。这是在初始化的早期完成的，这样我们就可以尽快将串行调试输出到控制台。如果你正在做板启动，这非常重要。

4. `stm32_gpioinit();`执行所需的全部GPIO重映射（F4是多余的，F1系列必须这步）。

5. `showprogress('A');`这只是在串行控制台上输出字符'A'（仅当启用`CONFIG_DEBUG`时）。如果启用了调试，您将始终在控制台上看到字母`ABDE`输出。该输出全部来自此文件。

6. 接下来，初始化内存：

   1. `.bss`段赋值为零（如果启用`CONFIG_DEBUG`，则输出字母'B'），然后
   2. `.data`段赋值为其初始值（如果启用了调试，则输出字母'C'）

7. 然后初始化指定板的逻辑：

   1. `stm32_boardinitialize();`这个函数在指定的板逻辑中。对于STM3240G-EVAL板的情况，可以在`configs/ stm3240g-eval/src/stm32_boot.c`中找到该板初始化逻辑。
   2. 对于STM3240G-EVAL板的情况，`stm32_boardinitialize()`执行以下操作：
      1. `stm32_spiinitialize();`如果启用了SPI，则初始化SPI片选。
      2. `stm32_selectsram();`如果启用了外部SRAM支持，则将STM32 FSMC配置为支持外部SRAM。
      3. `stm32_autoled_initialize();`初始化板载LED。

8. 当`stm32_boardinitialize()`返回到`__start()`，低级，特定于体系结构的初始化已完成，NuttX开始启动：

   1.`nx_start();`这是NuttX的入口点。它执行RTOS指定初始化的下一阶段，然后启动应用程序。

### NuttX RTOS 初始化

#### nx_start()

当低级，特定于体系结构的初始化完成并且通过调用函数`nx_start()`启动NuttX时。该函数位于文件

`nuttx/sched/init/nx_start.c`中。`nx_start()`执行的操作总结如下。请注意，可以从NuttX配置文件中禁用其中许多功能，在这种情况下，不会执行某些操作：

1. 初始化一些NuttX全局数据结构，
2. 初始化空闲线程的任务控制块（即执行初始化的线程）
3. `sem_initialize();`初始化POSIX信号量设施。这需要首先完成，因为几乎所有的其他操作系统特性依赖POSIX计数信号量。
4. `kmm_initialize();`初始化内存管理（在大多数配置中，`kmm_initialize()`是常见`mm_initialize()`的别名）。
5. `irq_initialize();`初始化中断处理子系统。这只初始化数据结构，CPU中断任然关闭。
6. `wd_initialize();`初始化看门狗定时器设备。
7. `clock_initialize();`初始化系统时钟。
8. `timer_initialize();`初始化POSIX定时器设备。
9. `sig_initialize();`初始化POSIX信号设备。
10. `mq_initialize();`初始化POSIX消息线程设备
11. `pthread_initialize();`初始化POSIX线程设备
12. `fs_initialize();`初始化文件系统设备
13. `net_initialize();`初始化网络设备

到目前为止，所有初始化步骤都只是软件初始化。没有任何东西与硬件互动。相反，所有这些步骤都只是准备好环境，以便中断和线程之类的东西可以正常运行。下面的步骤依赖这些操作。

1. `up_initialize();`这里将处理运行操作系统的处理器特定细节。诸如设置中断服务例程和启动时钟之类的事情是每个处理器和硬件平台不同的一些事情。有关此函数的ARM版本执行的初始化步骤的具体示例，请参见下文。
2. `board_early_initialize();`如果选中`CONFIG_BOARD_EARLY_INITIALIZE`,则会在启动流程中调用`board_early_initialize()`函数执行额外的初始化。`up_initialize()`之后会立即调用`board_early_initialize()`（可以被认为是特殊电路板的`up_initialize()`的扩充）并且`board_early_initialize()`被调用前，初始化程序已经启动。`board_early_initialize()`执行的上下文适用于大多数简单设备驱动程序的早期初始化。这将是需要执行低级硬件配置的地方，例如GPIO引脚的配置和简单设备驱动程序的初始化。但是，有些初始化操作不能在`board_early_initialize()`中执行，因为它们需要更多的操作系统已经初始化。因此，必须将某些驱动程序初始化延迟到`board_late_initialize()`。
3. `board_late_initialize();`如果在Nuttx配置中使能`CONFIG_BOARD_LATE_INITIALIZE`，将会调用用户定义的额外初始化`board_late_initialize()`函数。如果有需要执行特定板的初始化操作，则应该定义`CONFIG_BOARD_LATE_INITIALIZE`。注意上面有一个早期的，特定于板的初始化调用（上面的`stm32_board_initialize()` 和 `board_early_initialize()`）。不同的是，首先，低级初始化在操作系统启动前调用。`board_late_initialize()`，相反的，在初始化流程的后期调用。在OS初始化之后但在任何应用程序任务开始之前。`board_late_initialize()`是进行特定于板的初始化步骤的理想场所，这些步骤需要具有完全初始化的OS，例如内存分配，复杂设备驱动程序的初始化，文件系统的挂载等。
4. `lib_initialize();`初始化C库。这是最后完成的，因为库可能依赖于上述内容。
5. `sched_setupidlefiles();`这是打开`/dev/console`'的逻辑，并为IDLE（空闲）线程创建`stdin`，`stdout`和`stderr`。随后由IDLE线程创建的所有任务将继承这些文件描述符。
6. `nx_bringup();`创建初始任务。这将在下面更详细地描述。
7. 最后进入IDLE循环，完成初始化后，IDLE线程会发生改变，它现在变成了只有在系统中没有其他事情要做时才执行的线程（因此，名称为IDLE线程）。

#### IDLE线程活动

如前所述，IDLE线程是仅在系统中没有其他任何操作时才执行的线程。它在系统中具有最低优先级。它始终具有优先级0。它是唯一允许优先级为0的线程。它永远不会被阻止（否则，会运行什么？）。

因此，IDLE线程始终位于`g_readytorun`列表中，事实上，由于该列表具有优先级，因此可以保证始终是在`g_readytorun`列表尾部的最终条目。

IDLE是一个无限循环。但这并没有使它成为“CPU占用”。由于它是最低优先级，因此无论何时需要其他运行它都可以暂停。

IDLE线程在这个无限循环中完成两件事：

1. 如果工作还未开始（请参阅下面的`nx_bringup()`），则IDLE线程将执行内存清理。延迟内存释放处理需要内存清理。在软件无法访问堆的上下文中释放内存时，必须延迟内存分配，因此无法真正释放内存（例如在中断处理程序中）。在这种情况下，将内存简单地放入释放的内存列表中，最后由IDLE线程清理。*注意*：工作线程的主要功能是扩展设备驱动程序处理的“下半部分”。

   如果工作线程已启动，则它将以比IDLE线程更高的优先级运行。在这种情况下，工作线程将接管清理这些延期分配的任务。

2. `up_idle();`然后循环调用`up_idle()`，`up_idle()`执行的操作是特定于体系结构和板的。通常，这是执行可以CPU的降低功率操作的位置。

#### nx_bringup()

在进入IDLE循环之前，在`nx_start()`的初始化序列的最后调用该函数。该函数位于`nuttx/sched/init/nx_bringup.c`。此函数启动启动系统所需的所有必需线程和任务。此功能执行以下特定操作：

1. 如果配置了按需分页，则此功能将启动页填充任务。这是为了在具有MMU的处理器中以及在启用按需分页的配置中满足页面错误而运行的任务。
2. 如果这样配置，则此函数启动工作线程。工作线程可用于通过`include/nuttx/wqueue.h`中提供的API执行延迟到工作线程的任何处理。工作线程的主要功能是扩展设备驱动程序处理的“下半部分”，但可用于各种目的。
3. 最后，`nx_bringup()`将启动应用程序任务。默认情况下，这是其条目名称为`user_start()`的任务。 `user_start()`由应用程序代码提供，当它运行时，它开始应用程序制定的初始化步骤，如下所述。

注意：默认的`user_start()`入口点可以修改成NSH使用的应用程序名称中的一个。这是一个不常使用的启动选项，这里不再进一步讨论。

#### STM32F4 up_initialize()

所有基于ARM构架的MCU都使用`nuttx/arch/arm/common/up_initialize.c`提供`up_initialize()`。但是，这种相同的ARM初始化执行的操作将调用特定ARM芯片提供的功能。对于STM32F4，这些工具由`nuttx/arch/arm/src/stm32`里的文件提供逻辑。常见的ARM初始化序列是：

1. `up_calibratedelay();`在CPU端口校准期间必须执行的一项操作是定时延迟循环。若定义了`CONFIG_ARCH_CALIBRATION`，`up_initialize()`将执行一些特定的操作来校准延迟循环。但是，这不是正常初始化流程的一部分。 `up_calibratedelay()`在`up_initialize.c`中实现。

2. `up_addregion();`基本堆是在`nx_start()`处理期间建立的。但是，如果电路板支持多个不连续的内存区域，则可以通过此函数将任何添加内存区域添加到堆中。`up_addregion()` 在 `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`文件中实现。

3. `up_irqinitialize();`此函数初始化中断子系统。`up_irqinitialize(`) 在 `nuttx/arch/arm/src/stm32/stm32_irq.c`中实现.

4. `up_pminitialize();`如果定义了`CONFIG_PM`，这个函数必须初始化电源管理子系统。在初始化任何其他设备驱动程序之前，必须在初始化流程中尽早调用此特定于MCU的函数（因为它们可能尝试向电源管理子系统注册）。任何STM32平台都没有实现`up_pminitialize()`。

5. `up_dmainitialize();`初始化DMA子系统，DMA初始化在`nuttx/arch/arm/src/stm32/stm32_dma.c`（包括`nuttx/arch/arm/src/stm32f4xxx_dma.c`）。

6. `up_timerinit();`初始化系统定时器中断。这个函数初始化`ARM Cortex-M SYSTICK`定时器，位于`nuttx/arch/arm/src/stm32/stm32_timerisr.c`。

7. `devnull_register();`注册标准的`/dev/null`设备。

8. 然后此函数初始化控制台设备（如果有）。这意味着会调用

   1. `up_serialinit()`，用于标准串行驱动程序，位于`nuttx/arch/arm/src/stm32/stm32_serial.c`。

   2. `lowconsole_init()`，于低级只写串行控制台，位于`nuttx/drivers/serial/lowconsole_init.c`。或者

      `ramlog_sysloginit()`，用于RAM控制台，位于`nuttx/drivers/ramlog.c`。

9. `up_netinitialize();`初始化网络。对于STM32 F4，此函数位于`nuttx/arch/arm/src/stm32/stm32_eth.c`。

10. `up_usbinitialize();`初始化USB（主机或设备）。对于STM32 F4，此函数位于`nuttx/arch/arm/src/stm32/stm32_otgfsdev.c`。

11. `up_ledon(LED_IRQSENABLED);`最后，`up_initialize()`点亮特定于电路板的LED，表明IRQ现已启用。

#### STM32F4 IDLE thread

默认的STM32 F4 IDLE线程位于`nuttx/arch/arm/src/stm32_idle.c`，这个默认版本做的很少：

1. 他包含一个“骨架”函数例子，该函数说明了如果启用了`CONFIG_PM`，您可以执行的操作（此示例代码未在默认IDLE逻辑中完全实现）。
2. 它执行Cortex-M thumb2指令''wfi''，使CPU休眠直到下一次中断发生。

### 应用程序初始化

在`nx_start()`的OS初始化阶段结束时，通过在入口点`user_start()`创建新任务来启动用户应用程序。在NuttX之上构建的每个应用程序中都必须有一个名为`user_start()`的入口点。在`user_start()`函数中执行的任何其他初始化都完全取决于应用程序。

#### 一个简单的hello world 应用

最简单的用户应用程序是“Hello，World！”例子。请参阅''apps/examples/ hello''。这是整个例子：

~~~c
int user_start(int argc, char *argv[])
{
  printf("Hello, World!!\n");
  return 0;
}
~~~

在这种情况下，不需要额外的应用程序初始化。它只是“打招呼”并退出。

#### 一个Nutt Shell用户应用程序/命令

NuttShell（NSH）是一个简单的shell应用程序，可以与NuttX一起使用。它在这里描述：<http://nuttx.org/Documentation/NuttShell.html> 它支持各种命令，并且（非常）松散地基于bash shell和Unix shell编程中使用的常用实用程序。

NSH是作为一个库实现的，可以在''apps/nshlib''找到。NSH启动流程非常简单。例如，''apps/examples/nsh/nsh_main.c“中的代码说明了如何启动NuttX。它很简单如下：

1. 如果你有C++静态初始值设定项，它将调用你的`up_cxxinitialize()`，然后调用那些静态初始值设定项。对于STM3240G-EVAL板的情况，可以在`nuttx/configs/stm3240g-eval/src/up_cxxinitialize.c`中找到`up_cxxinitialize()`的实现。
2. 然后，此函数调用`nsh_initialize()`来初始化NSH库。 `nsh_initialize()`在下面更详细地描述
3. 如果启用了Telnet控制台，则会调用位于NSH库中的`nsh_telnetstart()`。`nsh_telnetstart()`将启动Telnet守护程序，该守护程序将侦听Telnet连接并启动远程NSH会话。
4. 如果启用了本地控制台（可能在串行端口上），则调用`nsh_consolemain()`。 `nsh_consolemain()`也驻留在NSH库中。 `nsh_consolemain()`不返回，此时完成了整个NSH初始化流程。

#### nsh_initialize()

NSH初始化函数，`nsh_initialize()`位于`apps/nshlib/nsh_init.c`。它只做三件事：

1. `nsh_romfsetc()`;如果这样配置，它将执行NSH启动脚本，该脚本可以在目标文件系统中的`/etc/init.d/rcS`中找到。 `/etc`是`nsh_romfsetc()`挂载只读ROMFS文件系统的位置。ROMFS映像本身就是内置于固件中的。默认情况下，此rcS启动脚本包含以下逻辑：

   ~~~sh
   # Create a RAMDISK and mount it at XXXRDMOUNTPOUNTXXX
   
   mkrd -m XXXMKRDMINORXXX -s XXMKRDSECTORSIZEXXX XXMKRDBLOCKSXXX
   mkfatfs /dev/ramXXXMKRDMINORXXX
   mount -t vfat /dev/ramXXXMKRDMINORXXX XXXRDMOUNTPOUNTXXX
   ~~~

   创建ROMFS镜像时，模板中的`XXXX*XXXX`字符串会被替换：

   - XXXMKRDMINORXXX 将会成为RAM设备次要编号。默认0
   - XXMKRDSECTORSIZEXXX 将会成为RAM设备扇区大小
   - XXMKRDBLOCKSXXX 将会成为设备扇区数目。
   - XXXRDMOUNTPOUNTXXX 将会成为配置的挂载点。默认`/etc`

   这个脚本将创建一个RAMDISK内存盘，在内存盘上格式化一个FAT文件系统，然后将FAT文件系统挂载到已配置的挂载点。这个''rcS''模板文件可以在''apps/nshlib/rcS.template''找到。生成的ROMFS文件系统可以在"apps/nshlib/nsh_romfsimg.h"中找到。

2. `boardctl()`;接下来将执行任何特定于体系结构的NSH初始化（如果有的话）。NSH初始化逻辑所有非标准OS接口`boardctl()`

   `(void)boardctl(BOARDIOC_INIT, 0)`；第一个参数，命令`BOARDIOC_INIT`表示正在请求`boardctl()`执行面向应用程序的初始化。响应此命令，`boardctl()`将调用`board_app_initialize()`的板特定实现。该功能通常不适用于所有配置中的应用程序级代码，但始终可以通过`boardctl()`进行访问。

3. `board_app_initialize()`;对于STM3240G-EVAL，这种体系结构特定的初始化可以“configs/stm3240g-eval/src/stm_nsh.c“找到。它执行以下操作：（1）初始化SPI设备，（2）初始化SDIO，以及（3）挂载可能插入的任何SD卡。

### 关于board_late_initialize()的更多信息

应用程序启动初始化有两种可能性：（1）在应用程序本身中通过`boardctl(BOARDIOC_INIT,0)`或（2）在OS中使用`board_late_initialize()`。每种情况在某些情况下运作良好，但也有一些我不喜欢的事情：

#### boardctl()

你可以在`apps/nshlib`中看到这个应用程序控制的初始化，例如，NSH应用程序；这被视为对`boardctl(BOARDIOC_INIT,0)`的调用。这要求用`CONFIG_LIB_BOARDCTL`启用`boardctl()`接口。

当`CONFIG_LIB_BOARDCTL`=`y`，板级操作系统内部逻辑必须提供`board_app_initilize()`接口，`boardctl(BOARDIOC_INIT,0)`是应用级接口。然后，可以在应用程序的控制下在`board_app_initilize()`中执行所有特定于板的驱动程序级初始化。

#### board_late_initialize()

`board_late_initialize()`是在应用程序启动之前执行的OS/内核级应用程序初始化，它通常运行良好 ，是`boardctl(BOARDIOC_INIT)`的一个很好的选择。为了避免尝试在IDLE线程上执行初始化的问题，会从内部内核线程调用`board_late_initialize()`。IDLE线程有局限性：它不能等待事件，所以它只能用于简单的直线初始化逻辑。在某些情况下，这可能是不够的。因此，`board_late_initialize()`必须在内核线程上运行。

考虑`board_late_initialize()`调用流程：`board_late_initialize()`在启动应用程序任务之前直接从`nuttx/sched/init/nx_bringup.c`文件中的`do_app_start()`调用。 反过来，`nx_create_initthread()`从函数`nx_start_application()`调用。如果未定义`CONFIG_BOARD_LATE_INITIALIZE`，那么这只是一个普通的C函数调用。但是如果定义`CONFIG_BOARD_LATE_INITIALIZE`，则启动一个中间的trampoline内核线程。该内核线程执行`do_app_start()`并将初始化移出IDLE线程。这很好用，但比我想要的要复杂一些。

#### 应用/平台

用户/应用程序初始化也有特殊的地方。这就是`apps/platform`平台目录。该目录应该是`nutx/configs`目录的镜像。对于`nuttx/configs`中的每个电路板，应该在`apps/platform`中有一个电路板目录。

### 自定义NSH初始化

NSH提供了许多方法来自定义其初始化操作。这些在[NSH documentation](http://www.nuttx.org/doku.php?id=documentation:nuttshell)的第4.0节中说明。
