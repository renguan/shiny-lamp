/*
 * 任务10: 第一阶段综合实验 (OpenHarmony/Hi3861)
 *
 * 要求:
 *   1. 舵机左右旋转测距(舵机GPIO2 + 超声波GPIO7/8)
 *   2. 前15秒红外对管寻线(GPIO13/14 -> UART2指令给STM32控制电机)
 *   3. 15秒后开始蓝牙通信(UART1外接蓝牙模块)
 *   4. 任务3运行, 串口打印消息队列信息(消息队列)
 *   5. 任务1、2交替运行(同优先级时间片轮转)
 *
 * 硬件映射(QST小车板):
 *   舵机GPIO2 | 超声波TRIG=GPIO7 ECHO=GPIO8 | 红外左GPIO13 右GPIO14
 *   蓝牙UART1(GPIO0/1) | 与STM32通信UART2(GPIO11/12)
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"

/* ================= 硬件引脚定义 ================= */
#define PIN_SG90         WIFI_IOT_IO_NAME_GPIO_2      /* 舵机 */
#define PIN_HCSR04_TRIG  7                            /* 超声波TRIG */
#define PIN_HCSR04_ECHO  8                            /* 超声波ECHO */
#define PIN_TRACE_L      WIFI_IOT_IO_NAME_GPIO_13     /* 红外对管 左 */
#define PIN_TRACE_R      WIFI_IOT_IO_NAME_GPIO_14     /* 红外对管 右 */
#define UART_BLE         WIFI_IOT_UART_IDX_1          /* 蓝牙模块 UART1 */
#define UART_STM32       WIFI_IOT_UART_IDX_2          /* 与STM32通信 UART2 */

#define TRACE_TIME_SEC   15                           /* 前15秒红外寻线 */

/* 消息结构: 任务间通过消息队列传递 */
typedef struct {
    uint8_t type;         /* 1=超声波测距 2=蓝牙数据 3=红外寻线 */
    float   dist;         /* 距离(cm) */
    char    str[32];      /* 字符串数据 */
} MsgInfo;

static osMessageQueueId_t g_msgq;

/* ================= 舵机: 20ms周期PWM驱动 ================= */
static void sg90_set_angle(unsigned int duty)
{
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}
static void sg90_left(void)   { for (int i = 0; i < 10; i++) sg90_set_angle(2200); }
static void sg90_middle(void) { for (int i = 0; i < 10; i++) sg90_set_angle(1650); }
static void sg90_right(void)  { for (int i = 0; i < 10; i++) sg90_set_angle(1100); }

/* ================= 超声波测距(返回cm) ================= */
static float hcsr04_get_distance(void)
{
    static unsigned long long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0, cnt = 0;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_GPIO_DIR_OUT);

    /* GPIO7输出触发脉冲 */
    GpioSetOutputVal(PIN_HCSR04_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(PIN_HCSR04_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 测量GPIO8回响高电平时间 */
    while (1) {
        GpioGetInputVal(PIN_HCSR04_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            break;
        }
        if (++cnt > 300000) { time = 0; break; }   /* 超时保护, 防止无回响卡死 */
    }
    distance = time * 0.034f / 2;                  /* 声速340m/s, 往返除2 */
    return distance;
}

/* ================= UART2: 与STM32通信控制电机 ================= */
static uint8_t uart_sendbuf[6];
static void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = (motorA < 0) ? 1 : 0;
    uint8_t B_dir = (motorB < 0) ? 1 : 0;
    if (motorA < 0) motorA = -motorA;
    if (motorB < 0) motorB = -motorB;
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;
    uart_sendbuf[0] = 0xFC;                        /* 帧头 */
    uart_sendbuf[1] = A_dir;                       /* 左轮方向 0正 1反 */
    uart_sendbuf[2] = (uint8_t)motorA;             /* 左轮速度 */
    uart_sendbuf[3] = B_dir;                       /* 右轮方向 */
    uart_sendbuf[4] = (uint8_t)motorB;             /* 右轮速度 */
    uart_sendbuf[5] = 0xFD;                        /* 帧尾 */
    UartWrite(UART_STM32, uart_sendbuf, 6);
}
static void car_forward(void)   { stm32motor_control(100, 100); }
static void car_left_tra(void)  { stm32motor_control(65, 110); }
static void car_right_tra(void) { stm32motor_control(110, 65); }
static void car_stop(void)      { stm32motor_control(0, 0); }

/* ================= UART 初始化 ================= */
static void uart_init(WifiIotUartIdx id)
{
    WifiIotUartAttribute attr;
    memset(&attr, 0, sizeof(attr));
    attr.baudRate = 115200;
    attr.dataBits = 8;
    attr.stopBits = 1;
    attr.parity = 0;
    UartInit(id, &attr, NULL);
}

/* ================= Task1: 舵机左右旋转测距 ================= */
static void *Task1(void *arg)
{
    (void)arg;
    float dist;
    MsgInfo msg;
    while (1) {
        sg90_left();
        dist = hcsr04_get_distance();
        printf("[Task1] 舵机左: %d cm\r\n", (int)dist);
        msg.type = 1; msg.dist = dist;
        osMessageQueuePut(g_msgq, &msg, 0, 0);

        sg90_middle();
        dist = hcsr04_get_distance();
        printf("[Task1] 舵机中: %d cm\r\n", (int)dist);
        msg.type = 1; msg.dist = dist;
        osMessageQueuePut(g_msgq, &msg, 0, 0);

        sg90_right();
        dist = hcsr04_get_distance();
        printf("[Task1] 舵机右: %d cm\r\n", (int)dist);
        msg.type = 1; msg.dist = dist;
        osMessageQueuePut(g_msgq, &msg, 0, 0);
    }
    return NULL;
}

/* ================= Task2: 前15秒红外寻线, 15秒后蓝牙通信 ================= */
static void *Task2(void *arg)
{
    (void)arg;
    WifiIotGpioValue l, r;
    uint8_t ble_buf[64];
    uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t start = osKernelGetTickCount();
    MsgInfo msg;

    /* 阶段1: 前15秒 红外对管寻线 */
    while ((osKernelGetTickCount() - start) < TRACE_TIME_SEC * tick_freq) {
        GpioGetInputVal(PIN_TRACE_L, &l);
        GpioGetInputVal(PIN_TRACE_R, &r);
        if (l == WIFI_IOT_GPIO_VALUE0 && r == WIFI_IOT_GPIO_VALUE0) {
            car_forward();                    /* 都在黑线上: 前进 */
        } else if (l == WIFI_IOT_GPIO_VALUE1 && r == WIFI_IOT_GPIO_VALUE0) {
            car_right_tra();                  /* 左偏出线: 右转修正 */
        } else if (l == WIFI_IOT_GPIO_VALUE0 && r == WIFI_IOT_GPIO_VALUE1) {
            car_left_tra();                   /* 右偏出线: 左转修正 */
        } else {
            car_stop();                       /* 都出线: 停止 */
        }
        msg.type = 3;
        snprintf(msg.str, sizeof(msg.str), "L:%d R:%d", l, r);
        osMessageQueuePut(g_msgq, &msg, 0, 0);
        printf("[Task2] 寻线 L:%d R:%d\r\n", l, r);
        usleep(30000);                        /* 30ms 一次 */
    }
    car_stop();
    printf("[Task2] 15秒到, 切换蓝牙通信!\r\n");

    /* 阶段2: 15秒后 蓝牙通信(UART1读蓝牙模块数据) */
    while (1) {
        int n = UartRead(UART_BLE, ble_buf, sizeof(ble_buf) - 1);
        if (n > 0) {
            ble_buf[n] = '\0';
            printf("[Task2] 蓝牙收到: %s\r\n", ble_buf);
            msg.type = 2;
            snprintf(msg.str, sizeof(msg.str), "%s", ble_buf);
            osMessageQueuePut(g_msgq, &msg, 0, 0);
        }
        usleep(20000);                        /* 20ms 轮询一次 */
    }
    return NULL;
}

/* ================= Task3: 消息队列接收并串口打印 ================= */
static void *Task3(void *arg)
{
    (void)arg;
    MsgInfo msg;
    while (1) {
        if (osMessageQueueGet(g_msgq, &msg, NULL, osWaitForever) == osOK) {
            if (msg.type == 1) {
                printf("[Task3] 消息队列-测距: %d cm\r\n", (int)msg.dist);
            } else if (msg.type == 2) {
                printf("[Task3] 消息队列-蓝牙: %s\r\n", msg.str);
            } else if (msg.type == 3) {
                printf("[Task3] 消息队列-寻线: %s\r\n", msg.str);
            }
        }
    }
    return NULL;
}

/* ================= 入口 ================= */
static void sum_experiment_first_demo(void)
{
    osThreadAttr_t attr;

    GpioInit();

    /* 红外对管 GPIO13/14 输入模式 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(PIN_TRACE_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(PIN_TRACE_R, WIFI_IOT_GPIO_DIR_IN);

    /* 蓝牙 UART1(GPIO0/1) + STM32通信 UART2(GPIO11/12) */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    uart_init(UART_BLE);
    uart_init(UART_STM32);

    /* 消息队列: 8条消息 */
    g_msgq = osMessageQueueNew(8, sizeof(MsgInfo), NULL);

    /* 创建任务1/2/3: 同优先级, 时间片交替运行 */
    memset(&attr, 0, sizeof(attr));
    attr.name = "Task1";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)Task1, NULL, &attr);

    attr.name = "Task2";
    osThreadNew((osThreadFunc_t)Task2, NULL, &attr);

    attr.name = "Task3";
    osThreadNew((osThreadFunc_t)Task3, NULL, &attr);

    printf("第一阶段综合实验启动!\r\n");
}

APP_FEATURE_INIT(sum_experiment_first_demo);