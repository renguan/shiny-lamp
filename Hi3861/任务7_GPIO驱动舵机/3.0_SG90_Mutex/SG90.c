/*
 * 任务7: OpenHarmony系统驱动实验 - GPIO驱动舵机 (学生作业版)
 *
 * 学生要求: 同优先级三个任务通过互斥锁进行多任务联动
 *   任务1: 优先运行, 串口输出1次, 舵机左转45度
 *   任务3: 任务1运行3秒后再运行, 串口输出2次, 舵机右转45度
 *   任务2: 任务3运行后立即运行, 串口输出3次, 舵机居中
 *
 * 时序(100tick=1秒, 三个任务同优先级):
 *   t=0s   任务1 获得互斥锁 -> 输出1次 -> 左转45度 -> 持有互斥锁3秒
 *   t=3s   任务3 获得互斥锁 -> 输出2次 -> 右转45度 -> 持有互斥锁1秒
 *   t=4s   任务2 获得互斥锁 -> 输出3次 -> 居中     -> 持有互斥锁1秒
 *   t=9s   任务1 再次获得互斥锁 -> 循环往复
 */

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

osMutexId_t mutex_id;
#define GPIO2 2

/* 共享资源: 舵机目标角度标志位, 三个任务在互斥锁保护下独占访问 */
#define ANGLE_LEFT   45
#define ANGLE_CENTER 90
#define ANGLE_RIGHT  135
uint8_t angle_flag = ANGLE_CENTER;

/******************* 舵机驱动(硬件层) *******************/
/* 查阅小车原理图可知, SG90舵机通过GPIO2与3861连接
 * SG90舵机需要MCU产生一个周期为20ms的脉冲信号, 以0.5ms~2.5ms的高电平控制舵机角度
 * 输出20000微秒周期脉冲: duty微秒高电平 + (20000-duty)微秒低电平 */
void set_angle(unsigned int duty)
{
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);       //设置GPIO2为输出模式
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);  //GPIO2输出duty微秒高电平
    hi_udelay(duty);
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);  //GPIO2输出(20000-duty)微秒低电平
    hi_udelay(20000 - duty);
}

/* 连续发送10次脉冲, 使舵机稳定转到目标角度 */
void engine_run(unsigned int duty)
{
    for (int i = 0; i < 10; i++) {
        set_angle(duty);
    }
}

/* 角度与脉冲关系: 0.5ms=0度, 1.0ms=45度, 1.5ms=90度, 2.0ms=135度, 2.5ms=180度
 * 参考舵机安装方向: 角度越大越向右转 */
void engine_left_45(void)  { engine_run(1000); }   //45度  -> 左转45度
void engine_center(void)   { engine_run(1500); }   //90度  -> 居中
void engine_right_45(void) { engine_run(2000); }   //135度 -> 右转45度

/******************* 三个同优先级任务: 互斥锁联动 *******************/
/* 任务1: 优先运行, 串口输出1次, 舵机左转45度, 持有互斥锁3秒 */
static void thread1(void)
{
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);   //获取互斥锁, 独占访问共享资源
        angle_flag = ANGLE_LEFT;
        printf("Task1 run: servo left 45.\r\n");   //串口输出1次
        engine_left_45();                          //舵机左转45度
        osDelay(300U);                             //持有互斥锁3秒, 任务3在此等待
        osMutexRelease(mutex_id);                  //释放互斥锁
        osDelay(600U);                             //让出, 等本轮其他任务执行完毕
    }
}

/* 任务3: 任务1运行3秒后再运行, 串口输出2次, 舵机右转45度 */
static void thread3(void)
{
    osDelay(300U);                                 //任务1运行3秒后再运行
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        angle_flag = ANGLE_RIGHT;
        printf("Task3 run: servo right 45.\r\n");  //串口输出2次
        printf("Task3 run: servo right 45.\r\n");
        engine_right_45();                         //舵机右转45度
        osDelay(100U);                             //持有互斥锁1秒
        osMutexRelease(mutex_id);
        osDelay(600U);
    }
}

/* 任务2: 任务3运行后立即运行, 串口输出3次, 舵机居中 */
static void thread2(void)
{
    osDelay(400U);                                 //任务3运行后立即运行
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        angle_flag = ANGLE_CENTER;
        printf("Task2 run: servo center.\r\n");    //串口输出3次
        printf("Task2 run: servo center.\r\n");
        printf("Task2 run: servo center.\r\n");
        engine_center();                           //舵机居中
        osDelay(100U);                             //持有互斥锁1秒
        osMutexRelease(mutex_id);
        osDelay(600U);
    }
}

/******************* 任务创建与启动 *******************/
static void SG90(void)
{
    GpioInit();                                                         //初始化GPIO
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);   //GPIO2复用为普通GPIO
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);         //设置为输出模式

    mutex_id = osMutexNew(NULL);                 //先创建互斥锁
    if (mutex_id == NULL) {
        printf("Falied to create Mutex!\n");
    }

    osThreadAttr_t attr = {0};
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.priority = 24;                          //三个任务同优先级
    attr.name = "thread1";
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Falied to create thread1!\n");
    }
    attr.name = "thread3";
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Falied to create thread3!\n");
    }
    attr.name = "thread2";
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Falied to create thread2!\n");
    }
}

APP_FEATURE_INIT(SG90);
