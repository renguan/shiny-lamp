/*
 * ============================================================================
 * Ultrasonic_2min_Drive: 教室自主行驶 2 分钟 —— 超声波避障(全新最终版)
 * ----------------------------------------------------------------------------
 * 需求: 小车放在教室里, 自动(非遥控)行驶约 2 分钟, 全程不撞到任何东西,
 *       利用超声波测距完成; 到点自动停车。
 *
 * 硬件(QST先锋号小车, Hi3861 侧):
 *   SG90 舵机云台 : GPIO2 (软件 PWM, 20ms 周期; 云台上装 HC-SR04, 可左/中/右转)
 *   超声波 HC-SR04: TRIG = GPIO7, ECHO = GPIO8
 *   电机          : Hi3861 经 UART2(GPIO11/12, 115200) 发协议帧
 *                   0xFC + 左轮方向 + 左轮速度 + 右轮方向 + 右轮速度 + 0xFD
 *                   指挥 STM32 底盘固件驱动电机(方向 0=正转 1=反转, 速度 0~150)
 *
 * 行为(阻塞式状态机, 前进为主, 心跳式发帧):
 *   1) 上电云台回中, 等 START_DELAY_MS(3s) 再起步(留时间放车/退后);
 *   2) 前进阶段: 云台居中(每周期发脉冲防震动偏头), 持续前进;
 *      每 ~60-90ms 测一次正前方距离:
 *        - 距离 < EMERGENCY_CM(14)          -> 单次立即判障(急停兜底)
 *        - 距离 < EVADE_CM(30) 且连续 2 次   -> 判障(消抖防误判)
 *      前进帧每周期重发(心跳式), 避免单帧丢失/STM32 需连续指令时车不动;
 *   3) 判障 -> 反转制动(250ms) -> 后退(500ms)拉开距离 -> 停车;
 *   4) 云台扫左/右测距 dl/dr(各连测 2 次取小, 防浅角弱回波误判成"无物"):
 *        - 两侧都堵(< SIDE_MIN_CM) -> 原地调头 180° 沿来路返回
 *        - 否则向较空一侧转 ~90°(分小步转, 每步停车看正前, 打通即停, 防转多撞墙)
 *   5) 循环 2)~4) 直到 120s -> 自动停车(停车帧连发 3 次), 打印完成。
 *
 * 实车调参/方向核对(烧录前必读, 详见同目录 使用说明.md):
 *   a) 若避障时车往"障碍"那边转(左右反): 交换 car_left/car_right 两行实现;
 *   b) 若云台扫到的"左"其实是车右侧(云台装反): 交换左/右脉宽宏
 *      (SG90_LEFT/SG90_RIGHT) 或交换 dl/dr 赋值;
 *   c) 直行跑偏: 调 SPEED_FWD_L/SPEED_FWD_R 差值修正(反偏则两宏对调);
 *      刹不住: 调大 EVADE_CM 或 BRAKE_MS; 转向角度: 调 TURN_STEP_MS/步数/SPEED_TURN;
 *   d) 电机链路前提: STM32 必须烧"解析 0xFC 帧"的底盘固件(支持包)。
 *      用串口助手 115200 手动 HEX 发 FC 00 64 00 64 FD(前进)/FC 01 50 01 50 FD
 *      (后退)可直连验证; 若只前进能动而后退/转向不动 -> 先重烧 STM32 底盘固件。
 * ============================================================================
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
#define PIN_SG90        WIFI_IOT_IO_NAME_GPIO_2   /* 舵机云台 */
#define PIN_TRIG        7                         /* 超声波 TRIG  GPIO7 */
#define PIN_ECHO        8                         /* 超声波 ECHO  GPIO8 */
#define UART_STM32      WIFI_IOT_UART_IDX_2       /* 与STM32通信 */

/* ================= 舵机角度脉宽 us(任务10实车标定): 左/中/右 ================= */
#define SG90_LEFT       2200
#define SG90_MID        1650
#define SG90_RIGHT      1100

/* ================= 运动参数(按实车微调) ================= */
#define SPEED_FWD_L     70                        /* 前进速度(0~150), 左右可分开调差速 */
#define SPEED_FWD_R     70
#define SPEED_BRAKE     110                       /* 反转制动力: 大力矩瞬间刹住 */
#define SPEED_BACK      70                        /* 遇障后退速度 */
#define SPEED_TURN      95                        /* 原地转向速度 */
#define BRAKE_MS        250                       /* 强力制动时长 */
#define BACKUP_MS       500                       /* 判障后退时长(拉开与障碍距离) */
#define TURN_STEP_MS    400                       /* 转向一小步时长(约20~30度) */
#define TURN_90_STEPS   4                         /* 90° 转向步数 */
#define TURN_180_STEPS  7                         /* 两侧都堵时调头 180° 步数 */
#define REST_MS         200                       /* 转向后停顿 */
#define SCAN_PULSES     50                        /* 扫描时舵机脉冲数(50*20ms=1s 转到位) */
#define SCAN_SETTLE_MS  150                       /* 扫描后等舵机稳定再测距 */

/* ================= 避障判定阈值(单位 cm) ================= */
#define EMERGENCY_CM    14                        /* 前方低于此值 -> 单次立即判障 */
#define EVADE_CM        30                        /* 前方低于此值且连续2次 -> 判障 */
#define SIDE_MIN_CM     35                        /* 转向侧最小可用距离, 低于视为不通 */
#define GO_CM           45                        /* 转向中正前高于此值视为已打通, 停转 */
#define FAR_CM          999                       /* 无回波/超量程按畅通处理 */

/* ================= 时序参数 ================= */
#define START_DELAY_MS  3000                      /* 上电后等待再起步(ms) */
#define RUN_TOTAL_MS    120000                    /* 总运行时长: 2分钟 */
#define MIN_AVOID_MS    3000                      /* 剩余时间小于它, 不再启动完整避障 */

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

/* ================= 超声波测距(HC-SR04, 返回 cm) ================= */
static int hcsr04_get_distance(void)
{
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int t0, t1;
    int cm;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_IN);

    /* 触发脉冲 >=10us */
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 等回波高电平; 30ms 无回波 -> 前方无物体(空旷/超量程), 按畅通处理 */
    t0 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            break;
        }
        if ((hi_get_us() - t0) > 30000) {
            return FAR_CM;
        }
    }

    /* 测量回波高电平持续时间 */
    t1 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE0) {
            break;
        }
        if ((hi_get_us() - t1) > 40000) {   /* 回波超长(空旷时模块可能拉高~38ms) */
            return FAR_CM;
        }
    }
    t1 = hi_get_us() - t1;                  /* 高电平时长 us */
    cm = (int)(t1 / 58);                    /* 声速340m/s: 距离cm = 时间us / 58 */
    if (cm > FAR_CM) {
        cm = FAR_CM;
    }
    return cm;
}

/* 连测 2 次取小值: 浅角度打墙回波弱会偶发"无回波", 取小更保险(决策用) */
static int hcsr04_min(void)
{
    int a, b;
    a = hcsr04_get_distance();
    usleep(60000);                          /* 两次触发间隔 >=60ms */
    b = hcsr04_get_distance();
    return (a < b) ? a : b;
}

/* ================= UART2 电机协议(0xFC帧头+方向+速度+0xFD帧尾) ================= */
static uint8_t uart_sendbuf[6];
static void motor_send(int motorA, int motorB)
{
    uint8_t a_dir = (motorA < 0) ? 1 : 0;
    uint8_t b_dir = (motorB < 0) ? 1 : 0;
    if (motorA < 0) motorA = -motorA;
    if (motorB < 0) motorB = -motorB;
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;
    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = a_dir;
    uart_sendbuf[2] = (uint8_t)motorA;
    uart_sendbuf[3] = b_dir;
    uart_sendbuf[4] = (uint8_t)motorB;
    uart_sendbuf[5] = 0xFD;
    UartWrite(UART_STM32, uart_sendbuf, 6);   /* 失败不处理: 心跳下个周期自动重发 */
}

static void car_forward(void)  { motor_send(SPEED_FWD_L, SPEED_FWD_R); }
static void car_brake(void)    { motor_send(-SPEED_BRAKE, -SPEED_BRAKE); }  /* 反转制动 */
static void car_backward(void) { motor_send(-SPEED_BACK, -SPEED_BACK); }
static void car_left(void)     { motor_send(-SPEED_TURN, SPEED_TURN); }     /* 原地左转 */
static void car_right(void)    { motor_send(SPEED_TURN, -SPEED_TURN); }     /* 原地右转 */
static void car_stop(void)     { motor_send(0, 0); }

/* 停车帧连发 3 次(心跳), 保证 STM32 侧一定执行停车 */
static void car_stop_x3(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        car_stop();
        usleep(30000);
    }
}

/* 心跳式执行动作 ms 毫秒: 每 30ms 重发一次指令帧(STM32 需要连续指令也能动) */
static void motor_hold(void (*action)(void), unsigned int ms)
{
    unsigned int elapsed = 0;
    while (elapsed < ms) {
        action();
        usleep(30000);
        elapsed += 30;
    }
}

/* ================= 时间判断: 运行是否满 2 分钟 ================= */
static int time_up(unsigned int t0)
{
    return (hi_get_us() - t0) >= (RUN_TOTAL_MS * 1000u);
}

/* ================= 转向: 分小步转, 每步停车看正前, 打通即停(防转多撞墙) ================= */
static void turn_steps(int right, int steps, unsigned int t0)
{
    int i;
    int d = FAR_CM;

    for (i = 0; i < steps; i++) {
        unsigned int k;
        if (time_up(t0)) {
            break;
        }
        /* 一小步: 原地旋转, 心跳发转向帧, 同时云台保持居中 */
        for (k = 0; k < (TURN_STEP_MS / 50); k++) {
            if (right) {
                car_right();
            } else {
                car_left();
            }
            sg90_pulse(SG90_MID);
            usleep(30000);
        }
        car_stop();
        usleep(100000);

        /* 90° 转向: 每步后看正前是否已打通; 180° 调头不检查(完整转过去) */
        if (steps <= TURN_90_STEPS) {
            d = hcsr04_get_distance();
            printf("[2min] turn %s step %d/%d, front=%d cm\r\n",
                   right ? "R" : "L", i + 1, steps, d);
            if (d >= GO_CM) {
                break;
            }
        }
    }
    car_stop();
}

/* ================= 遇障处理: 后退拉开 -> 扫左右 -> 选向转向 ================= */
static void avoid_phase(unsigned int t0)
{
    int dl = FAR_CM;
    int dr = FAR_CM;

    printf("[2min] avoid: backup then scan sides\r\n");
    motor_hold(car_backward, BACKUP_MS);   /* 后退拉开距离 */
    car_stop_x3();
    usleep(150000);

    /* 云台左扫: 到位稳定后连测 2 次取小 */
    sg90_goto(SG90_LEFT, SCAN_PULSES);
    usleep(SCAN_SETTLE_MS * 1000);
    dl = hcsr04_min();

    /* 云台右扫 */
    sg90_goto(SG90_RIGHT, SCAN_PULSES);
    usleep(SCAN_SETTLE_MS * 1000);
    dr = hcsr04_min();

    /* 云台回中 */
    sg90_goto(SG90_MID, SCAN_PULSES);
    usleep(SCAN_SETTLE_MS * 1000);

    printf("[2min] scan: L=%d cm, R=%d cm\r\n", dl, dr);

    if (dl < SIDE_MIN_CM && dr < SIDE_MIN_CM) {
        /* 两侧都堵: 原地调头 180°, 沿来路返回 */
        printf("[2min] both sides tight -> turn around 180\r\n");
        turn_steps(1, TURN_180_STEPS, t0);
    } else if (dl >= dr) {
        /* 左边较空: 左转 */
        printf("[2min] steer LEFT (open side)\r\n");
        turn_steps(0, TURN_90_STEPS, t0);
    } else {
        /* 右边较空: 右转 */
        printf("[2min] steer RIGHT (open side)\r\n");
        turn_steps(1, TURN_90_STEPS, t0);
    }
    car_stop_x3();
    usleep(REST_MS * 1000);
}

/* ================= 前进阶段: 盯正前, 判障即停并制动; 返回 1=时间到 ================= */
static int forward_phase(unsigned int t0)
{
    int cnt = 0;
    int d = FAR_CM;
    unsigned int diag = 0;

    printf("[2min] forward...\r\n");
    while (1) {
        car_forward();              /* 心跳: 每周期重发前进帧 */
        sg90_pulse(SG90_MID);       /* 云台保持居中(防震动偏头) */

        d = hcsr04_get_distance();  /* 测正前方距离 */

        if (d < EMERGENCY_CM) {     /* 很近: 单次立即判障(急停兜底) */
            cnt = 2;
        } else if (d < EVADE_CM) {  /* 较近: 连续 2 次确认, 防误判 */
            if (++cnt >= 2) {
                break;
            }
        } else {
            cnt = 0;
        }

        /* 诊断: 约每 1s 打印一次距离与运行秒数 */
        if ((++diag % 10) == 0) {
            printf("[2min] t=%u s, dist=%d cm\r\n",
                   (unsigned int)((hi_get_us() - t0) / 1000000), d);
        }

        if (time_up(t0)) {
            car_stop_x3();
            return 1;               /* 已满 2 分钟 */
        }
        usleep(30000);
    }

    /* ---- 判障: 反转制动(心跳)刹住惯性 ---- */
    printf("[2min] OBSTACLE dist=%d cm -> brake\r\n", d);
    motor_hold(car_brake, BRAKE_MS);
    car_stop_x3();
    usleep(120000);
    return 0;
}

/* ================= 主任务 ================= */
static void *DriveTask(void *arg)
{
    (void)arg;
    unsigned int t0;

    sg90_goto(SG90_MID, 50);        /* 云台先居中 */
    usleep(300000);

    printf("[2min] === classroom auto-drive %u ms (ultrasonic) ===\r\n",
           (unsigned int)RUN_TOTAL_MS);
    printf("[2min] starting in %u s, stand back!\r\n",
           (unsigned int)(START_DELAY_MS / 1000));
    usleep(START_DELAY_MS * 1000);

    t0 = hi_get_us();
    printf("[2min] START!\r\n");

    while (1) {
        if (forward_phase(t0)) {
            break;                  /* 已满 2 分钟 */
        }
        if (time_up(t0)) {
            break;
        }
        /* 剩余时间不足完成一次完整避障 -> 提前收车, 不再启动新避障 */
        if ((hi_get_us() - t0) >= (RUN_TOTAL_MS - MIN_AVOID_MS) * 1000u) {
            printf("[2min] < %u s left, stop ahead of time\r\n",
                   (unsigned int)(MIN_AVOID_MS / 1000));
            break;
        }
        avoid_phase(t0);
        if (time_up(t0)) {
            break;
        }
    }

    /* 到点: 云台回中 + 停车帧连发, 保证 STM32 侧一定停车 */
    sg90_goto(SG90_MID, 20);
    car_stop_x3();
    usleep(50000);
    car_stop_x3();
    printf("[2min] === 2 MIN DONE, car stopped ===\r\n");
    return NULL;
}

/* ================= 入口 ================= */
static void drive_2min_demo(void)
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
    attr.name = "DriveTask";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)DriveTask, NULL, &attr);

    printf("[2min] ready (press reset to restart)\r\n");
}

APP_FEATURE_INIT(drive_2min_demo);
