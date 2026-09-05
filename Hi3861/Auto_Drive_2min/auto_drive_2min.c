/*
 * Auto_Drive_2min: 教室自主行驶2分钟 - 超声波避障
 *
 * 需求: 小车放在教室里, 自动(非遥控)行驶约 2 分钟, 途中不撞到任何东西。
 *
 * 方案(QST小车板硬件):
 *   超声波 HC-SR04 装在 GPIO2 舵机云台上, 可左/中/右转向测距
 *     TRIG = GPIO7, ECHO = GPIO8
 *   电机: Hi3861 经 UART2(GPIO11/12) 发送协议帧(0xFC+左向/左速+右向/右速+0xFD)
 *         指挥 STM32 驱动, 115200
 *
 * 行为流程(阻塞式状态机, 类似悬崖避障):
 *   1. 前进阶段: 云台居中, 持续前进, 每 ~60ms 测一次前方距离
 *      - 前方 < STOP_TRIGGER_CM(30cm) 连续 2 次  -> 判定有障碍
 *      - 前方 < EMERGENCY_CM(12cm) 单次立即急停    -> 判定障碍很近
 *      - 全程检查 2 分钟时间到, 到则停车结束
 *   2. 避障阶段: 强力制动 -> 后退 BACKUP_MS -> 停车
 *      - 云台扫左右两侧距离 dl/dr
 *      - 两侧都堵(< SIDE_MIN_CM)  -> 原地调头 180 度, 沿来路返回
 *      - 否则向较空一侧转向 TURN_90_MS 继续前进
 *   3. 循环直到 120 秒 -> 自动停车, 打印完成
 *
 * 实车调参(都在本文件顶部宏):
 *   - 前进/后退/转向速度、制动/后退/转向时长按实车手感微调
 *   - 若小车实际转向与预期相反(如该左转实际右转), 把避障里两处
 *     "转向较空侧" 的 car_left/car_right 对调(或互换 car_left/car_right 定义)
 *   - 若超声波左右读数与车头朝向相反, 把扫描 dl/dr 的赋值对调
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

/* ================= 引脚 ================= */
#define PIN_SG90         WIFI_IOT_IO_NAME_GPIO_2   /* 舵机云台 */
#define PIN_TRIG         7                         /* 超声波 TRIG */
#define PIN_ECHO         8                         /* 超声波 ECHO */
#define UART_STM32       WIFI_IOT_UART_IDX_2       /* 与STM32通信 */

/* ================= 舵机角度脉宽 us(任务10实测): 左/中/右 ================= */
#define SG90_LEFT        2200
#define SG90_MID         1650
#define SG90_RIGHT       1100

/* ================= 运动参数(按实车微调) ================= */
#define SPEED_FWD        70                        /* 前进速度: 适中, 给超声波留反应距离 */
#define SPEED_BRAKE      100                       /* 反转制动力: 大力矩瞬间刹住 */
#define SPEED_BACK       80                        /* 后退速度 */
#define SPEED_TURN       100                       /* 转向速度(原地差速旋转) */
#define BRAKE_MS         250                       /* 强力制动时长 */
#define BACKUP_MS        700                       /* 遇障后后退时长(拉开与障碍距离) */
#define TURN_90_MS       1500                      /* 转向较空侧时长(约90度, 按实车标定) */
#define TURN_180_MS      3000                      /* 两侧都堵时调头180度时长 */
#define REST_MS          200                       /* 转向后停顿 */

/* ================= 避障判定阈值 ================= */
#define STOP_TRIGGER_CM  30                        /* 前方低于此值且连续2次 -> 避障 */
#define EMERGENCY_CM     12                        /* 前方低于此值 -> 单次立即急停 */
#define SIDE_MIN_CM      35                        /* 转向侧最小可用距离: 低于视为不通
                                                       (>= 避障触发距离+余量, 避免转向后立刻又判障) */
#define FAR_CM           300                       /* 超过此距离视为畅通(无回波/超量程) */

/* ================= 时序参数 ================= */
#define RUN_TOTAL_MS     120000                    /* 总运行时长: 2分钟 */
#define SAMPLE_PAD_US    40000                     /* 测距间隔填充(HC-SR04触发周期需>=60ms) */
#define SCAN_PULSES      30                        /* 扫描时舵机脉冲数(30*20ms=0.6s) */
#define SCAN_SETTLE_MS   200                       /* 扫描后等舵机到位再测距 */

/* ================= 舵机驱动(软件PWM, 20ms周期) ================= */
static void sg90_pulse(unsigned int duty)
{
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}
/* 舵机转到 duty(us) 位置, 发 pulses 个脉冲让它转到位 */
static void sg90_goto(unsigned int duty, int pulses)
{
    int i;
    for (i = 0; i < pulses; i++) {
        sg90_pulse(duty);
    }
}

/* ================= 超声波测距(HC-SR04, 返回cm) ================= */
static float hcsr04_get_distance(void)
{
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    uint64_t t0, t1;
    float cm;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_IN);

    /* 触发脉冲 >=10us */
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 等回波高电平, 30ms 无回波 -> 无物体, 返回 FAR_CM(畅通) */
    t0 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            break;
        }
        if ((hi_get_us() - t0) > 30000) {
            return (float)FAR_CM;
        }
    }
    /* 测量高电平持续时长 */
    t1 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE0) {
            break;
        }
        if ((hi_get_us() - t1) > 40000) {   /* 回波超长(无物体时模块可能拉高~38ms) */
            return (float)FAR_CM;
        }
    }
    t1 = hi_get_us() - t1;                  /* 高电平时长 us */
    cm = (float)t1 * 0.034f / 2.0f;         /* 声速340m/s, 往返除2 */
    if (cm > FAR_CM) {                      /* 超量程按畅通处理 */
        cm = (float)FAR_CM;
    }
    return cm;
}

/* ================= UART2 电机协议(0xFC帧头+方向+速度+0xFD帧尾) ================= */
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
static void car_forward(void)  { stm32motor_control(SPEED_FWD, SPEED_FWD); }
static void car_brake(void)    { stm32motor_control(-SPEED_BRAKE, -SPEED_BRAKE); } /* 反转制动 */
static void car_backward(void) { stm32motor_control(-SPEED_BACK, -SPEED_BACK); }
static void car_left(void)     { stm32motor_control(-SPEED_TURN, SPEED_TURN); }    /* 原地左转 */
static void car_right(void)    { stm32motor_control(SPEED_TURN, -SPEED_TURN); }    /* 原地右转 */
static void car_stop(void)     { stm32motor_control(0, 0); }

/* ================= 时间判断: 运行是否满2分钟 ================= */
static int time_up(uint64_t t_start)
{
    return ((hi_get_us() - t_start) / 1000) >= RUN_TOTAL_MS;
}

/* ================= 主任务: 前进 -> 遇障 -> 制动后退 -> 扫左右选路 -> 转向 -> 循环 ================= */
static void *AutoDriveTask(void *arg)
{
    (void)arg;
    float d = 0, dl = 0, dr = 0;
    int trig_cnt = 0;
    uint32_t diag = 0;
    uint64_t t_start;

    sg90_goto(SG90_MID, 50);        /* 云台先居中 */
    usleep(300000);

    t_start = hi_get_us();
    printf("[Auto] 2min auto drive start!\r\n");

    while (1) {
        /* ============ 前进阶段 ============ */
        car_forward();
        trig_cnt = 0;
        printf("[Auto] forward...\r\n");

        while (1) {
            /* 行驶中持续给云台发居中脉冲, 防止震动偏头 */
            sg90_pulse(SG90_MID);

            d = hcsr04_get_distance();      /* 测前方距离 */

            if (d < EMERGENCY_CM) {         /* 很近: 单次立即判障 */
                trig_cnt = 2;
            } else if (d < STOP_TRIGGER_CM) {
                if (++trig_cnt >= 2) {      /* 连续2次确认, 防误判 */
                    break;
                }
            } else {
                trig_cnt = 0;
            }

            /* 诊断: 每约1秒打印一次距离 */
            if (++diag % 16 == 0) {
                printf("[Auto] dist=%d cm\r\n", (int)d);
            }

            if (time_up(t_start)) {         /* 2分钟到, 结束 */
                break;
            }
            usleep(SAMPLE_PAD_US);          /* 填充测距周期 */
        }
        if (time_up(t_start)) {
            break;
        }

        /* ============ 遇障: 强力制动 -> 后退 ============ */
        printf("[Auto] blocked dist=%d cm\r\n", (int)d);
        car_brake();
        usleep(10000);
        car_brake();
        usleep(10000);
        car_brake();
        usleep(BRAKE_MS * 1000);

        printf("[Auto] backup...\r\n");
        car_backward();
        usleep(BACKUP_MS * 1000);
        car_stop();
        usleep(100000);

        /* ============ 云台扫左右, 选较空一侧 ============ */
        sg90_goto(SG90_LEFT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dl = hcsr04_get_distance();
        printf("[Auto] left=%d cm\r\n", (int)dl);

        sg90_goto(SG90_RIGHT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dr = hcsr04_get_distance();
        printf("[Auto] right=%d cm\r\n", (int)dr);

        sg90_goto(SG90_MID, SCAN_PULSES);   /* 云台回中 */

        if (dl < SIDE_MIN_CM && dr < SIDE_MIN_CM) {
            /* 两侧都堵: 原地调头180度, 沿来路返回 */
            printf("[Auto] both blocked, turn around 180!\r\n");
            car_left();
            usleep(TURN_180_MS * 1000);
        } else if (dl >= dr) {
            printf("[Auto] turn LEFT (L=%d R=%d)\r\n", (int)dl, (int)dr);
            car_left();
            usleep(TURN_90_MS * 1000);
        } else {
            printf("[Auto] turn RIGHT (L=%d R=%d)\r\n", (int)dl, (int)dr);
            car_right();
            usleep(TURN_90_MS * 1000);
        }
        car_stop();
        usleep(REST_MS * 1000);
    }

    /* 2分钟到: 停车帧连发3次, 防止单帧丢失导致车不停 */
    car_stop();
    usleep(10000);
    car_stop();
    usleep(10000);
    car_stop();
    printf("[Auto] 2 MIN DONE, car stopped!\r\n");
    return NULL;
}

/* ================= 入口 ================= */
static void auto_drive_demo(void)
{
    osThreadAttr_t attr;
    WifiIotUartAttribute uattr;

    GpioInit();

    /* 舵机 GPIO2 输出 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);

    /* 超声波 GPIO7/8 (hcsr04_get_distance 内部也会设置) */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);

    /* UART2 与 STM32 通信 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    UartInit(UART_STM32, &uattr, NULL);

    /* 创建自主行驶任务 */
    memset(&attr, 0, sizeof(attr));
    attr.name = "AutoDrive";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)AutoDriveTask, NULL, &attr);

    printf("Auto Drive 2min ready!\r\n");
}

APP_FEATURE_INIT(auto_drive_demo);
