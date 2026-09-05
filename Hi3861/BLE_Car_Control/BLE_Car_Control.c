/*
 * BLE_Car_Control: 手机蓝牙控制小车 (任务26 蓝牙遥控)
 *
 * 硬件(QST小车板):
 *   蓝牙模块 JDY-16: UART1 (GPIO0 TXD / GPIO1 RXD), 波特率 9600
 *   电机: Hi3861 经 UART2 (GPIO11/12) 协议帧指挥 STM32, 115200
 *
 * 指令协议(手机蓝牙串口助手发送单个字符, 老师规范):
 *   0 = 停止                 W/w = 前进(100,100)
 *   A/a = 左转(-50,150)      D/d = 右转(150,-50)
 *   S/s = 后退(-150,-150)
 *   I/i = 速度(100,100)      K/k = 速度(150,150)
 *   E/e = 一键演示: 自动执行一套动作序列(前进->左转->前进->右转->后退->停)
 *   3 秒无指令自动停车(防手机断开乱跑)
 *
 * 学生扩展(老师文档"一键系列动作"):
 *   按单个按键 E 触发整套动作序列, 用"步进表 + 状态机"实现,
 *   每步 = 动作函数 + 持续时长(ms), 增删/改顺序/改时长只需改 demo_seq[] 表,
 *   序列执行期间按键 0 或任意方向键可立即打断。
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

#define BLE_BAUD         9600                       /* 蓝牙模块波特率(JDY-16默认9600) */
#define STOP_TIMEOUT_MS  3000                       /* 无指令自动停车时间 */

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

/* 老师规范动作(速度参考 supportPack/5_QST_car/src/robot_l9110s.c) */
static void car_forward(void)  { stm32motor_control(100, 100); }   /* W 前进 */
static void car_backward(void) { stm32motor_control(-150, -150); } /* S 后退 */
static void car_left(void)     { stm32motor_control(-50, 150); }   /* A 左转 */
static void car_right(void)    { stm32motor_control(150, -50); }   /* D 右转 */
static void car_stop(void)     { stm32motor_control(0, 0); }       /* 0 停止 */
static void car_speed_I(void)  { stm32motor_control(100, 100); }   /* I (100,100) */
static void car_speed_K(void)  { stm32motor_control(150, 150); }   /* K (150,150) */

/* ============ 一键系列动作(按键 E 触发) ============ */
typedef void (*action_fn)(void);
typedef struct {
    action_fn fn;     /* 该步执行的动作函数 */
    uint32_t  ms;     /* 该步持续时长(ms) */
} seq_step_t;

/* 演示序列(步进表, 可按需增删/改顺序/改时长):
 *   前进1s -> 左转0.8s -> 前进1s -> 右转0.8s -> 后退1s -> 停 */
static const seq_step_t demo_seq[] = {
    { car_forward,  1000 },
    { car_left,      800 },
    { car_forward,  1000 },
    { car_right,     800 },
    { car_backward, 1000 },
};
#define DEMO_SEQ_STEPS (sizeof(demo_seq) / sizeof(demo_seq[0]))

static int      seq_active = 0;          /* 1=正在执行动作序列 */
static uint32_t seq_index  = 0;          /* 当前执行到第几步 */
static uint64_t seq_step_start_us = 0;   /* 当前步开始时间戳(us) */
static uint64_t g_last_cmd_us = 0;       /* 最后收到指令时间戳(us) */

static void seq_start(void)
{
    seq_index = 0;
    seq_active = 1;
    seq_step_start_us = hi_get_us();
    demo_seq[0].fn();
    printf("[SEQ] START %u steps\r\n", (unsigned)DEMO_SEQ_STEPS);
}

/* 序列推进: 每步到时自动切下一步, 全部完成后自动停车 */
static void seq_tick(void)
{
    uint64_t now;
    if (!seq_active) {
        return;
    }
    now = hi_get_us();
    if ((now - seq_step_start_us) >= (uint64_t)demo_seq[seq_index].ms * 1000) {
        seq_index++;
        if (seq_index >= DEMO_SEQ_STEPS) {
            car_stop();
            seq_active = 0;
            g_last_cmd_us = 0;   /* 序列结束, 清空指令时间戳, 避免误触发超时停车 */
            printf("[SEQ] DONE\r\n");
        } else {
            seq_step_start_us = now;
            demo_seq[seq_index].fn();
            printf("[SEQ] step %u\r\n", (unsigned)seq_index);
        }
    }
}

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

    while (1) {
        n = UartRead(UART_BLE, buf, sizeof(buf));
        if (n > 0) {
            /* 诊断: 打印收到的原始字节(hex), 核对波特率/链路 */
            printf("[BLE] RX(%d):", n);
            for (i = 0; i < n; i++) {
                printf(" %02X", buf[i]);
            }
            printf("\r\n");
            for (i = 0; i < n; i++) {
                switch (buf[i]) {
                    case '0':
                        seq_active = 0;
                        car_stop();
                        printf("[BLE] STOP\r\n");
                        break;
                    case 'W': case 'w':
                        seq_active = 0;
                        car_forward();
                        printf("[BLE] FORWARD\r\n");
                        break;
                    case 'A': case 'a':
                        seq_active = 0;
                        car_left();
                        printf("[BLE] LEFT\r\n");
                        break;
                    case 'D': case 'd':
                        seq_active = 0;
                        car_right();
                        printf("[BLE] RIGHT\r\n");
                        break;
                    case 'S': case 's':
                        seq_active = 0;
                        car_backward();
                        printf("[BLE] BACKWARD\r\n");
                        break;
                    case 'I': case 'i':
                        seq_active = 0;
                        car_speed_I();
                        printf("[BLE] SPEED(100,100)\r\n");
                        break;
                    case 'K': case 'k':
                        seq_active = 0;
                        car_speed_K();
                        printf("[BLE] SPEED(150,150)\r\n");
                        break;
                    case 'E': case 'e':
                        seq_start();
                        break;
                    default:
                        break;
                }
            }
            g_last_cmd_us = hi_get_us();   /* 记录最后指令时间 */
        } else {
            /* 超时保护: 超过STOP_TIMEOUT_MS没收到指令且无序列运行则停车 */
            if (!seq_active && g_last_cmd_us != 0 &&
                (hi_get_us() - g_last_cmd_us) > (uint64_t)STOP_TIMEOUT_MS * 1000) {
                car_stop();
                g_last_cmd_us = 0;
                printf("[BLE] TIMEOUT AUTO-STOP\r\n");
            }
        }
        seq_tick();      /* 动作序列推进 */
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

    printf("BLE Car Ready! (0=stop W=fwd A=left D=right S=back I=100,100 K=150,150 E=demo)\r\n");
}

APP_FEATURE_INIT(ble_car_control_demo);
