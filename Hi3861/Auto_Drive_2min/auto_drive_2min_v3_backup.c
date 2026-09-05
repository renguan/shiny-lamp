/*
 * Auto_Drive_2min v3: 教室自主行驶2分钟 - 超声波避障
 *
 * 需求: 小车放在教室里, 自动(非遥控)行驶约 2 分钟, 途中不撞到任何东西。
 *
 * v2 实车反馈: 几乎不走(前进占空比太低: 每段只走0.9s就停车扫描1.5s+,
 *             且转向要求"转到正前>=70cm", 常空转到步数上限).
 * v3 修改:
 *   1. 每周期 DRIVE_MS=1.5s 连续前进(可调), 前进中每 ~60ms 盯正前,
 *      只有正前真近(<32cm×2 或 <15cm×1)才提前停 -> 保证"会往前走";
 *   2. 扫描改在前进段之间做(左/右两向, ~1s), 正前由前进段自己盯着,
 *      侧墙逼近(<40cm)转 1 小步修正, 防斜向撞墙;
 *   3. 转向按测量定步数: 判障/正前近 -> 向较空侧转 ~3 步(约100度)或调头 5 步,
 *      小步(350ms)+每步停, 不再"转到70cm才罢休"; 转向后前进段会立刻复核正前,
 *      不盲冲;
 *   4. 差速直行保留(SPEED_FWD_L>R 抵消偏左); 前进中周期性重发前进帧防丢帧。
 *
 * 实车调参/方向核对:
 *   - 直行仍偏左: 加大 SPEED_FWD_L 或减小 SPEED_FWD_R(反偏右则两宏对调);
 *   - "该左转实际右转": 对调 car_left/car_right 实现; 云台左右与车头相反:
 *     对调 dl/dr 赋值;
 *   - 走得太碎: 加大 DRIVE_MS(如2500); 转弯太猛: 减小 SPEED_TURN/TURN_STEP_MS
 *   - 判障距离/刹车手感: STOP_CM / SPEED_FWD_L/R / BRAKE_MS 微调
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
#define SPEED_FWD_L      78                        /* 左轮前进: 略大于右轮, 抵消实测偏左 */
#define SPEED_FWD_R      70                        /* 右轮前进 */
#define SPEED_BRAKE      100                       /* 反转制动力 */
#define SPEED_BACK       80                        /* 后退速度 */
#define SPEED_TURN       85                        /* 转向速度(低一点, 减小每步位移) */
#define BRAKE_MS         200                       /* 制动时长 */
#define BACKUP_MS        700                       /* 遇障后退时长(给转向留空间) */

/* ================= 避障/导航阈值 ================= */
#define STOP_CM          32                        /* 前进中正前<此值且连续2次 -> 停 */
#define EMERGENCY_CM     15                        /* 前进中正前<此值 -> 单次急停 */
#define AVOID_CM         45                        /* 行驶结束正前仍<此值 -> 需转向(不直冲) */
#define SIDE_WARN_CM     40                        /* 扫描时侧向<此值 -> 反向修正1小步 */
#define SIDE_OPEN_CM     45                        /* 侧向>=此值才作为"转向目标侧" */
#define FAR_CM           300                       /* 超过此距离视为畅通(无回波/超量程) */

/* ================= 时序参数 ================= */
#define RUN_TOTAL_MS     120000                    /* 总运行时长: 2分钟 */
#define DRIVE_MS         1500                      /* 每段前进时长(两次扫描之间的行驶时间) */
#define SAMPLE_PAD_US    30000                     /* 前进中测距间隔填充(HC-SR04周期>=60ms) */
#define FWD_RETRY_ITERS  5                         /* 前进中每5个采样周期重发一次前进帧(防丢帧) */
#define TURN_STEP_MS     350                       /* 转向每步时长(小步+每步停) */
#define TURN_STEPS_NORM  3                         /* 常规避障转向步数(约100度) */
#define TURN_STEPS_BACK  5                         /* 两侧都堵调头步数(约180度) */
#define STEER_STEPS      1                         /* 侧向修正步数 */
#define SCAN_PULSES      12                        /* 扫描舵机脉冲数(12*20ms=0.24s) */
#define SCAN_SETTLE_MS   100                       /* 扫描后等舵机到位再测距 */
#define REST_MS          100                       /* 转向每步后停顿 */

/* ================= 舵机驱动(软件PWM, 20ms周期) ================= */
static void sg90_pulse(unsigned int duty)
{
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}
/* 舵机转到 duty(us) 位置, 发 pulses 个脉冲 */
static void sg90_goto(unsigned int duty, int pulses)
{
    int i;
    for (i = 0; i < pulses; i++) {
        sg90_pulse(duty);
    }
}

/* ================= 超声波测距(HC-SR04, 返回cm; FAR_CM=无回波/超量程) ================= */
static float hcsr04_get_distance(void)
{
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    uint64_t t0, t1;
    float cm;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_IN);

    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE0);

    /* 等回波高电平, 30ms 无回波 -> 畅通 */
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
        if ((hi_get_us() - t1) > 40000) {
            return (float)FAR_CM;
        }
    }
    t1 = hi_get_us() - t1;
    cm = (float)t1 * 0.034f / 2.0f;
    if (cm > FAR_CM) {
        cm = (float)FAR_CM;
    }
    return cm;
}
/* 决策用: 连测2次(间隔>=60ms)取较小值, 防偶发漏回波把"有障碍"读成"畅通" */
static float hcsr04_measure_twice(void)
{
    float d1, d2;
    d1 = hcsr04_get_distance();
    usleep(60000);
    d2 = hcsr04_get_distance();
    return (d1 < d2) ? d1 : d2;
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
static void car_forward(void)  { stm32motor_control(SPEED_FWD_L, SPEED_FWD_R); } /* 左轮略快, 抵消偏左 */
static void car_brake(void)    { stm32motor_control(-SPEED_BRAKE, -SPEED_BRAKE); }
static void car_backward(void) { stm32motor_control(-SPEED_BACK, -SPEED_BACK); }
static void car_left(void)     { stm32motor_control(-SPEED_TURN, SPEED_TURN); }   /* 原地左转 */
static void car_right(void)    { stm32motor_control(SPEED_TURN, -SPEED_TURN); }   /* 原地右转 */
static void car_stop(void)     { stm32motor_control(0, 0); }

/* ================= 时间判断 ================= */
static int time_up(uint64_t t_start)
{
    return ((hi_get_us() - t_start) / 1000) >= RUN_TOTAL_MS;
}

/* ================= 小步转向: steps>0 左转, steps<0 右转 ================= */
static void spin_steps(int steps)
{
    int i, n = (steps > 0) ? steps : -steps;
    for (i = 0; i < n; i++) {
        if (steps > 0) {
            car_left();
        } else {
            car_right();
        }
        usleep(TURN_STEP_MS * 1000);
        car_stop();
        usleep(REST_MS * 1000);
    }
}

/* ================= 主任务 ================= */
static void *AutoDriveTask(void *arg)
{
    (void)arg;
    float d = FAR_CM, dc_last = FAR_CM, dl = FAR_CM, dr = FAR_CM;
    int trig_cnt, blocked, avoid;
    int steps, iters;
    uint32_t cycle = 0;
    uint64_t t_start, drive_start;

    sg90_goto(SG90_MID, 50);        /* 云台先居中 */
    usleep(200000);

    t_start = hi_get_us();
    printf("[Auto] 2min auto drive v3 start!\r\n");

    while (1) {
        if (time_up(t_start)) {
            break;
        }
        cycle++;

        /* ============ DRIVE: 前进 DRIVE_MS, 期间盯正前 ============ */
        blocked = 0;
        avoid = 0;
        trig_cnt = 0;
        iters = 0;
        car_forward();
        drive_start = hi_get_us();
        while ((hi_get_us() - drive_start) < (uint64_t)DRIVE_MS * 1000) {
            sg90_pulse(SG90_MID);                  /* 防云台震动偏头 */
            d = hcsr04_get_distance();             /* 正前距离 */

            if (d < EMERGENCY_CM) {                /* 很近: 单次急停 */
                blocked = 1;
                break;
            } else if (d < STOP_CM) {
                if (++trig_cnt >= 2) {             /* 连续2次确认 */
                    blocked = 1;
                    break;
                }
            } else {
                trig_cnt = 0;
            }

            if (time_up(t_start)) {
                break;
            }
            usleep(SAMPLE_PAD_US);
            if (++iters % FWD_RETRY_ITERS == 0) {
                car_forward();                     /* 周期性重发, 防单帧丢失 */
            }
        }
        dc_last = d;                               /* 记录本次行驶末次正前距离 */
        if (time_up(t_start)) {
            break;
        }
        car_stop();

        if (blocked) {
            /* 遇障: 制动 -> 后退, 下段必须转向 */
            printf("[Auto] #%u BLOCKED dist=%d cm, brake+backup\r\n", cycle, (int)d);
            car_brake();
            usleep(10000);
            car_brake();
            usleep(10000);
            car_brake();
            usleep(BRAKE_MS * 1000);
            car_backward();
            usleep(BACKUP_MS * 1000);
            car_stop();
            usleep(100000);
            avoid = 1;
        } else if (dc_last < AVOID_CM) {
            /* 正前已比较近(再走一点就会判障): 下段转向, 不直冲 */
            avoid = 1;
            printf("[Auto] #%u front close %d cm\r\n", cycle, (int)dc_last);
        }

        /* ============ SCAN: 扫左右, 定转向步数 ============ */
        sg90_goto(SG90_LEFT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dl = hcsr04_measure_twice();               /* 左 */
        sg90_goto(SG90_RIGHT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dr = hcsr04_measure_twice();               /* 右 */
        sg90_goto(SG90_MID, SCAN_PULSES);          /* 回中 */
        usleep(SCAN_SETTLE_MS * 1000);

        steps = 0;
        if (avoid) {
            /* 刚避障/正前近: 必须转向(>0左转 <0右转), 或两侧都堵则调头 */
            if (dl >= SIDE_OPEN_CM || dr >= SIDE_OPEN_CM) {
                if (dl >= dr) {
                    steps = TURN_STEPS_NORM;       /* 左侧更空: 左转 */
                } else {
                    steps = -TURN_STEPS_NORM;      /* 右侧更空: 右转 */
                }
            } else {
                /* 两侧都堵: 调头(左右交替, 防总往一边转) */
                steps = (cycle & 1) ? TURN_STEPS_BACK : -TURN_STEPS_BACK;
            }
        } else if (dl < SIDE_WARN_CM && dl <= dr) {
            steps = -STEER_STEPS;                  /* 左墙近: 右偏一点 */
        } else if (dr < SIDE_WARN_CM && dr < dl) {
            steps = STEER_STEPS;                   /* 右墙近: 左偏一点 */
        }
        printf("[Auto] #%u scan L=%d R=%d dclast=%d, turn=%d\r\n",
               cycle, (int)dl, (int)dr, (int)dc_last, steps);

        if (steps != 0) {
            spin_steps(steps);                     /* 小步转向 */
        }
        /* 回到顶部: 下一段前进(前进段会立刻复核正前) */
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

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    UartInit(UART_STM32, &uattr, NULL);

    memset(&attr, 0, sizeof(attr));
    attr.name = "AutoDrive";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)AutoDriveTask, NULL, &attr);

    printf("Auto Drive 2min v3 ready!\r\n");
}

APP_FEATURE_INIT(auto_drive_demo);
