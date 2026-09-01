/*
 * 悬崖避障 (Cliff Detection)
 *
 * 功能: 小车在台面上自主运动, 车前端底盘下的红外对管实时检测台面边缘(悬崖)
 *       到达边缘 -> 停止 -> 后退一小段距离 -> 转向 -> 继续前进, 循环往复
 *
 * 硬件映射(QST小车板):
 *   红外对管: GPIO13(左) / GPIO14(右), 朝下安装, 检测台面反射
 *   电机控制: Hi3861 经 UART2(GPIO11/12) 发送协议帧给 STM32 驱动电机
 *
 * 对管电平逻辑(TCRT5000): 有反射(台面上)=0, 无反射(边缘悬空/黑线)=1
 *   任一对管为1 => 车头已探出台面边缘 => 停止后退
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

#define PIN_TRACE_L      WIFI_IOT_IO_NAME_GPIO_13   /* 红外对管 左 */
#define PIN_TRACE_R      WIFI_IOT_IO_NAME_GPIO_14   /* 红外对管 右 */
#define UART_STM32       WIFI_IOT_UART_IDX_2        /* 与STM32通信 */

/* 运动参数(ms), 按实车微调 */
#define BACK_MS          2000                       /* 后退时长 */
#define TURN_MS          1500                       /* 转向时长 */
#define REST_MS          300                        /* 转向后停顿 */

/* ============ UART2 电机协议(0xFC帧头+方向+速度+0xFD帧尾) ============ */
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
    UartWrite(UART_STM32, uart_sendbuf, 6);
}
static void car_forward(void)  { stm32motor_control(150, 150); }
static void car_backward(void) { stm32motor_control(-150, -150); }
static void car_left(void)     { stm32motor_control(-100, 150); }
static void car_right(void)    { stm32motor_control(150, -100); }
static void car_stop(void)     { stm32motor_control(0, 0); }

/* ============ 悬崖检测: 任一红外对管无反射(悬空)即为边缘 ============ */
static int at_cliff(void)
{
    WifiIotGpioValue l, r;
    GpioGetInputVal(PIN_TRACE_L, &l);
    GpioGetInputVal(PIN_TRACE_R, &r);
    return (l == WIFI_IOT_GPIO_VALUE1 || r == WIFI_IOT_GPIO_VALUE1);
}

/* ============ 主任务: 前进 -> 遇边缘 -> 停 -> 后退 -> 转向 -> 继续 ============ */
static void *CliffTask(void *arg)
{
    (void)arg;
    int turn_dir = 0;   /* 交替左/右转 */

    while (1) {
        car_forward();
        printf("[Cliff] 前进...\r\n");

        /* 持续前进直到检测到边缘 */
        while (!at_cliff()) {
            usleep(30000);   /* 30ms 轮询一次 */
        }

        /* 到达边缘: 停止 */
        car_stop();
        printf("[Cliff] 检测到边缘! 停止\r\n");
        usleep(200000);

        /* 后退一小段距离避开悬崖 */
        car_backward();
        printf("[Cliff] 后退避让...\r\n");
        usleep(BACK_MS * 1000);
        car_stop();

        /* 转向换个方向 */
        if (turn_dir) {
            car_left();
            printf("[Cliff] 左转避开\r\n");
        } else {
            car_right();
            printf("[Cliff] 右转避开\r\n");
        }
        usleep(TURN_MS * 1000);
        car_stop();
        turn_dir = !turn_dir;

        usleep(REST_MS * 1000);
    }
    return NULL;
}

/* ============ 入口 ============ */
static void cliff_detection_demo(void)
{
    osThreadAttr_t attr;
    WifiIotUartAttribute uattr;

    GpioInit();

    /* 红外对管 GPIO13/14 输入模式 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(PIN_TRACE_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(PIN_TRACE_R, WIFI_IOT_GPIO_DIR_IN);

    /* UART2 与STM32通信 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    UartInit(UART_STM32, &uattr, NULL);

    /* 创建悬崖避障任务 */
    memset(&attr, 0, sizeof(attr));
    attr.name = "CliffTask";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)CliffTask, NULL, &attr);

    printf("悬崖避障启动!\r\n");
}

APP_FEATURE_INIT(cliff_detection_demo);