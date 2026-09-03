/*
 * BLE_Car_Control: 手机蓝牙控制小车前进/后退/转弯
 *
 * 硬件(QST小车板):
 *   蓝牙模块: UART1 (GPIO0 TXD / GPIO1 RXD) -> 手机蓝牙串口助手连接
 *   电机控制: Hi3861 经 UART2 (GPIO11/12) 协议帧指挥 STM32
 *
 * 指令协议(手机蓝牙串口助手发送单个字符):
 *   F/f = 前进   B/b = 后退   L/l = 左转   R/r = 右转   S/s = 停止
 *   支持按住连续发送(如按住F持续前进), 松开后3秒无指令自动停车
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

#define UART_BLE         WIFI_IOT_UART_IDX_1        /* 蓝牙模块 UART1 */
#define UART_STM32       WIFI_IOT_UART_IDX_2        /* 与STM32通信 UART2 */

#define BLE_BAUD         9600                       /* 蓝牙模块波特率(BT04默认9600) */
#define STOP_TIMEOUT_MS  3000                       /* 无指令自动停车时间 */

/* 运动速度(0-150) */
#define SPEED_FWD        75
#define SPEED_BACK       75
#define SPEED_TURN       100

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
    {
        int ret = UartWrite(UART_STM32, uart_sendbuf, 6);
        /* 诊断: 打印发给STM32的协议帧 */
        printf("[U2] %02X %02X %02X %02X %02X %02X ret=%d\r\n",
               uart_sendbuf[0], uart_sendbuf[1], uart_sendbuf[2],
               uart_sendbuf[3], uart_sendbuf[4], uart_sendbuf[5], ret);
    }
}
static void car_forward(void)  { stm32motor_control(SPEED_FWD, SPEED_FWD); }
static void car_backward(void) { stm32motor_control(-SPEED_BACK, -SPEED_BACK); }
static void car_left(void)     { stm32motor_control(-SPEED_TURN, SPEED_TURN); }
static void car_right(void)    { stm32motor_control(SPEED_TURN, -SPEED_TURN); }
static void car_stop(void)     { stm32motor_control(0, 0); }

/* ============ UART 初始化(返回结果) ============ */
static unsigned int uart_init(WifiIotUartIdx id, unsigned int baud)
{
    WifiIotUartAttribute attr;
    memset(&attr, 0, sizeof(attr));
    attr.baudRate = baud;
    attr.dataBits = 8;
    attr.stopBits = 1;
    attr.parity = 0;
    return UartInit(id, &attr, NULL);
}

/* ============ 蓝牙遥控任务 ============ */
static void *BleControlTask(void *arg)
{
    (void)arg;
    uint8_t buf[32];
    int i, n;
    uint64_t last_cmd_us = 0;

    while (1) {
        n = UartRead(UART_BLE, buf, sizeof(buf));
        if (n > 0) {
            /* 诊断: 打印收到的原始字节(hex + 字符), 核对波特率/链路 */
            printf("[BLE] RX(%d):", n);
            for (i = 0; i < n; i++) {
                printf(" %02X", buf[i]);
            }
            printf("\r\n");
            for (i = 0; i < n; i++) {
                switch (buf[i]) {
                    case 'F': case 'f':
                        car_forward();
                        printf("[BLE] FORWARD\r\n");
                        break;
                    case 'B': case 'b':
                        car_backward();
                        printf("[BLE] BACKWARD\r\n");
                        break;
                    case 'L': case 'l':
                        car_left();
                        printf("[BLE] LEFT\r\n");
                        break;
                    case 'R': case 'r':
                        car_right();
                        printf("[BLE] RIGHT\r\n");
                        break;
                    case 'S': case 's': case ' ':
                        car_stop();
                        printf("[BLE] STOP\r\n");
                        break;
                    default:
                        break;
                }
            }
            last_cmd_us = hi_get_us();   /* 记录最后指令时间 */
        } else {
            /* 超时保护: 超过STOP_TIMEOUT_MS没收到指令则停车 */
            if (last_cmd_us != 0 && (hi_get_us() - last_cmd_us) > (uint64_t)STOP_TIMEOUT_MS * 1000) {
                car_stop();
                last_cmd_us = 0;
                printf("[BLE] TIMEOUT AUTO-STOP\r\n");
            }
        }
        usleep(10000);   /* 10ms 轮询 */
    }
    return NULL;
}

/* ============ 入口 ============ */
static void ble_car_control_demo(void)
{
    osThreadAttr_t attr;

    GpioInit();

    /* UART2(与STM32通信, 115200) 先初始化 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    printf("UART2 init ret=%u\r\n", uart_init(UART_STM32, 115200));

    /* UART1(蓝牙, 9600): 系统已释放该口(见app_main.c修改), 由应用初始化 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    printf("UART1 init ret=%u\r\n", uart_init(UART_BLE, BLE_BAUD));

    /* 遥控任务 */
    memset(&attr, 0, sizeof(attr));
    attr.name = "BleControl";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)BleControlTask, NULL, &attr);

    printf("BLE Car Ready! (F=fwd B=back L=left R=right S=stop)\r\n");
}

APP_FEATURE_INIT(ble_car_control_demo);