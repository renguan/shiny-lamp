/*
 * Motor_Diag v2: 持续重发式电机方向自测
 *
 * 背景: 用户"只向前"(持续发前进帧)能动车, 但 MotorDiag v1(每个动作单发一帧停1.5s)全不动。
 * 本版模仿"只向前"的写法: 每个动作**每100ms持续重发**指令帧, 并打印 UartWrite 返回值。
 *
 * 流程(循环):
 *   [FWD-LONG] 前进 10s   <- 先复现"只向前", 若这步动 -> 帧/STM32 链路 OK
 *   [BACK]     后退  5s
 *   [LEFT]     原地左转 5s
 *   [RIGHT]    原地右转 5s
 *   步间停 1s。每步打印发送帧与 ret。
 *
 * 观察结论:
 *   - 前进10s动、后面都不动  -> STM32 收到前进但不认反转/转向 -> 换 STM32 底盘固件
 *   - 全都不动(ret=0正常)     -> STM32 未解析帧/电机供电问题
 *                                     -> 用串口直连 STM32 测试(开关拨STM32端, 115200,
 *                                        手动HEX发 FC 00 64 00 64 FD 等看车动不动)
 *   - ret=-1                 -> Hi3861 UART2 发送失败(初始化/引脚问题)
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

#define UART_STM32       WIFI_IOT_UART_IDX_2
#define TEST_SPEED       90        /* 自测速度 */

/* ================= 电机协议 ================= */
static uint8_t uart_sendbuf[6];
static void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = (motorA < 0) ? 1 : 0;
    uint8_t B_dir = (motorB < 0) ? 1 : 0;
    if (motorA < 0) motorA = -motorA;
    if (motorB < 0) motorB = -motorB;
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;
    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = (uint8_t)motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = (uint8_t)motorB;
    uart_sendbuf[5] = 0xFD;
    printf("[TX] %02X %02X %02X %02X %02X %02X ret=%d\r\n",
           uart_sendbuf[0], uart_sendbuf[1], uart_sendbuf[2],
           uart_sendbuf[3], uart_sendbuf[4], uart_sendbuf[5],
           UartWrite(UART_STM32, uart_sendbuf, 6));
}

/* 持续执行某动作 ms 毫秒, 每100ms重发一次指令 */
static void phase(const char *name, int motorA, int motorB, int ms)
{
    int n = ms / 100;
    int i;
    printf("[TEST] %s (%d ms)\r\n", name, ms);
    for (i = 0; i < n; i++) {
        stm32motor_control(motorA, motorB);
        usleep(100000);
    }
    stm32motor_control(0, 0);
    printf("[TEST] STOP\r\n");
    usleep(1000000);       /* 停1s */
}

static void *DiagTask(void *arg)
{
    (void)arg;
    printf("[TEST] Motor diag v2 start!\r\n");
    while (1) {
        phase("FWD-LONG forward 10s (mimic forward-only)",  TEST_SPEED,  TEST_SPEED, 10000);
        phase("BACKWARD",                                  -TEST_SPEED, -TEST_SPEED,  5000);
        phase("LEFT",                                      -TEST_SPEED,  TEST_SPEED,  5000);
        phase("RIGHT",                                      TEST_SPEED, -TEST_SPEED,  5000);
    }
    return NULL;
}

/* ================= 入口 ================= */
static void motor_diag_demo(void)
{
    osThreadAttr_t attr;
    WifiIotUartAttribute uattr;

    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    printf("UART2 init ret=%u\r\n", UartInit(UART_STM32, &uattr, NULL));

    memset(&attr, 0, sizeof(attr));
    attr.name = "DiagTask";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)DiagTask, NULL, &attr);

    printf("Motor diag v2 ready!\r\n");
}

APP_FEATURE_INIT(motor_diag_demo);
