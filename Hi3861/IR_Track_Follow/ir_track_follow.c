/*
 * IR_Track_Follow v8: 短促有力耸动 + 全程连续采样 循迹/岔路寻路
 *
 * 实车特性(用户反馈): 低速(≈50)小车卡住不走(要助推), 说明电机有死区;
 *   连续高速又来不及反应。=> 耸动 = 高速短爆发 + 短暂停顿(用"有力但时间短"控距),
 *   传感器全程每 10ms 采样(不停下来才识别), 行驶中压黑提前停。
 *
 * 每耸: 高速(≈110)持续 BURST_MS(默认160ms) -> 停 SETTLE_MS(80ms) -> 读传感器。
 *   耸动中每10ms采样: 进入黑区(连续2次双1)立即提前停 -> 转测绘。
 *   直线段耸动方向按"上一耸停点读数": 0,0直耸; 1,0向左弧耸; 0,1向右弧耸。
 *
 * 黑区测绘/岔路寻路逻辑同 v7: 压黑 -> 逐耸过黑计数区段(单/双条) ->
 *   黑后连续地面(FLOOR_CLEAR_HOPS 耸) = 到点 -> 探臂(先左后右):
 *   探到=岔口上臂(记栈), 双条无臂=终点停, 单区无臂=死路(倒车掉头折返, 回岔口走反臂)。
 *
 * 上车标定: BURST_MS 使每耸 ≈1.5~2.5cm(≈1/8轮); 若一耸太远减 BURST_MS, 不动则加。
 * 宏: TRACE_FLIP / PREFER_LEFT / BURST_MS / ZONE_MAX_STEPS / FLOOR_CLEAR_HOPS。
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

#define PIN_IR_L         WIFI_IOT_IO_NAME_GPIO_13
#define PIN_IR_R         WIFI_IOT_IO_NAME_GPIO_14
#define UART_STM32       WIFI_IOT_UART_IDX_2

/* ===== 运动参数(实车: 低速有死区; 耸动停止要可靠, 防突冲) ===== */
#define SPEED_BURST      150        /* 耸动主速(顶格) */
#define SPEED_ARC_HI     150        /* 弧耸快轮 */
#define SPEED_ARC_LO     120        /* 弧耸慢轮 */
#define SPEED_BACK       100
#define SPEED_TURN       150        /* 原地转 */
#define SPIN_MIN_MS      150        /* 纠偏: 至少转够的角度(避免一次回不了中反复小转) */
#define SPIN_MAX_MS      800
#define RECOVER_STUCK_MS 4000       /* 同一侧连续纠偏失败>=此ms => 疑似轮胎卡死 */
#define RECOVER_MS       600        /* 解脱: 反向倒车时长 */
#define SPEED_REC_FAST   90         /* 反向倒车快侧 */
#define SPEED_REC_SLOW   50         /* 反向倒车慢侧(若倒的方向反了, 把快慢对调) */
#define BURST_MS         300        /* 每耸时长(拉长, 每耸≈2-3cm) */
#define SETTLE_MS        60         /* 耸后停稳(压缩, 提高决策频率) */
#define TURN_PROBE_MS    550        /* (旧)探臂转角 */
#define TURN_ARM_MS      750        /* 探到臂后"转上岔臂"转角时长(调大, 真正上臂) */
#define SCAN_PROBE_MS    1600       /* 探臂连续扫描最大时长(角度加大) */
#define BACK_TO_JUNC_MS  400        /* 探臂前倒车回靠岔口(每段时长) */
#define TURN_180_MS      1600
#define BACK_CLEAR_MS    400
#define RETRACE_MAX_S    30
#define BOOT_BACK_MS     300
#define BOOT_BACK_TRIES  6

/* ===== 判定阈值 ===== */
#define TRACE_FLIP       0
#define PREFER_LEFT      1
#define SAMPLE_US        10000
#define BLACK_SAMP_ABORT 2          /* 耸动中连续双1>=此 => 提前停 */
#define ZONE_MAX_STEPS   8          /* 压黑连续耸上限 */
#define ZONE_GAP_HOPS    4          /* 两段黑间隔<=此耸数 => 双条(每耸变长, 同步换算) */
#define FLOOR_CLEAR_HOPS 3          /* 黑后连续地面耸数>=此 => 到点(缩小, 别溜进岔口腹地) */
#define MAX_FORKS        4

enum { S_DRIVE = 0, S_BLACK, S_AFTER, S_DEAD, S_GOAL, S_BOOTBK, S_HALT, S_SEARCH };

/* ===== 电机 ===== */
static uint8_t tx[6];
static void motor(int a, int b)
{
    uint8_t ad = (a < 0) ? 1 : 0, bd = (b < 0) ? 1 : 0;
    if (a < 0) a = -a;
    if (b < 0) b = -b;
    if (a > 150) a = 150;
    if (b > 150) b = 150;
    tx[0] = 0xFC; tx[1] = ad; tx[2] = (uint8_t)a;
    tx[3] = bd;  tx[4] = (uint8_t)b; tx[5] = 0xFD;
    UartWrite(UART_STM32, tx, 6);
}
/* 停车: 连发停帧(短促但多帧), 防单帧丢失 */
static void stop(void)
{
    int i;
    for (i = 0; i < 4; i++) {
        motor(0, 0);
        usleep(15000);
    }
}

/* ===== 红外 ===== */
static void rd(int *l, int *r)
{
    WifiIotGpioValue a, b;
    GpioGetInputVal(PIN_IR_L, &a);
    GpioGetInputVal(PIN_IR_R, &b);
    *l = (a == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
    *r = (b == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
}

/* 一次耸动(每10ms采样, 采样流全程参与判定)。
 * kind: 0直 1左弧 -1右弧; abort: 1=行驶中遇"双黑/持续单侧偏线"提前停。
 * 返回: 2=持续双黑; 1=途中碰过线(任一单1); 0=纯地面;
 *      -1=abort模式下持续单侧偏线提前停(仅abort=1时返回) */
static int burst(int kind, int abort)
{
    int i, l = 0, r = 0, s11 = 0, s1s = 0, saw = 0;
    int n = BURST_MS * 1000 / SAMPLE_US;

    stop();                      /* 先确保从静止开始(清残留运动) */
    for (i = 0; i < n; i++) {
        if (kind == 0)      motor(SPEED_BURST, SPEED_BURST);
        else if (kind > 0)  motor(SPEED_ARC_LO, SPEED_ARC_HI);
        else                motor(SPEED_ARC_HI, SPEED_ARC_LO);
        usleep(SAMPLE_US);
        rd(&l, &r);
        if (l == 1 && r == 1) {
            s1s = 0;
            if (++s11 >= BLACK_SAMP_ABORT) saw = 2;
        } else if (l == 1 || r == 1) {
            s11 = 0;
            if (abort && kind == 0) {
                /* 仅"直耸"时持续单侧偏线才提前停(直线段即时发现偏出);
                 * 弧耸本就在纠偏, 单侧是常态, 不提前停, 否则一转弯就停 */
                if (++s1s >= BLACK_SAMP_ABORT) {
                    stop();
                    usleep(SETTLE_MS * 1000);
                    return -1;
                }
            }
            if (saw < 2) saw = 1;
        } else {
            s11 = 0;
            s1s = 0;
        }
        if (abort && saw == 2) {
            stop();
            usleep(SETTLE_MS * 1000);
            return 2;
        }
    }
    stop();
    usleep(SETTLE_MS * 1000);
    return saw;
}

/* 原地转纠偏(左右轮正负对转): 先转足 SPIN_MIN_MS 角度(一步到位), 之后才认"回中";
 * 全程每10ms采样, 压黑即停返回2; 回中(连续2次0,0)返回0; 超时返回-1 */
static int spin_until_center(int dir, int max_ms)
{
    int i, l = 0, r = 0, ctr = 0;
    int n = max_ms * 1000 / SAMPLE_US;
    int nmin = SPIN_MIN_MS * 1000 / SAMPLE_US;

    for (i = 0; i < n; i++) {
        if (dir > 0) motor(-SPEED_TURN, SPEED_TURN);   /* 原地左转 */
        else         motor(SPEED_TURN, -SPEED_TURN);   /* 原地右转 */
        usleep(SAMPLE_US);
        rd(&l, &r);
        if (l == 1 && r == 1) {
            stop();
            usleep(80000);
            return 2;                 /* 压黑 */
        }
        if (i >= nmin && l == 0 && r == 0) {
            if (++ctr >= 2) {
                stop();
                usleep(80000);
                return 0;             /* 转足最小角度后才认回中 */
            }
        } else {
            ctr = 0;
        }
    }
    stop();
    usleep(80000);
    return -1;                        /* 超时未回中 */
}

/* 轮胎/机械解脱: 长时间单侧回不了中时, 反向倒车一小段(left_bias=1 左倒, 0 右倒) */
static void recover_reverse(int left_bias)
{
    stop();
    usleep(150000);
    printf("[T] RECOVER reverse %s\r\n", left_bias ? "LEFT" : "RIGHT");
    if (left_bias) motor(-SPEED_REC_FAST, -SPEED_REC_SLOW);
    else           motor(-SPEED_REC_SLOW, -SPEED_REC_FAST);
    usleep(RECOVER_MS * 1000);
    stop();
    usleep(150000);
}

/* 探臂: 连续旋转扫描(角度逐档加大), 任一瞬时见带即返回1;
 * side>0 向左扫 -1 向右扫; 最多扫 SCAN_PROBE_MS */
static int probe_side(int side, int max_ms)
{
    int i, l = 0, r = 0;
    int n = max_ms * 1000 / SAMPLE_US;
    for (i = 0; i < n; i++) {
        if (side > 0) motor(-SPEED_TURN, SPEED_TURN);
        else          motor(SPEED_TURN, -SPEED_TURN);
        usleep(SAMPLE_US);
        rd(&l, &r);
        if (l == 1 || r == 1) {
            stop();
            usleep(80000);
            return 1;
        }
    }
    stop();
    usleep(80000);
    return 0;
}

/* 探臂前先倒车回靠岔口边缘(确保没溜进岔口腹地) */
static void back_to_junction(void)
{
    int t, l = 0, r = 0;
    stop();
    usleep(100000);
    for (t = 0; t < 3; t++) {
        motor(-SPEED_BACK, -SPEED_BACK);
        usleep(BACK_TO_JUNC_MS * 1000);
        stop();
        usleep(80000);
        rd(&l, &r);
        if (l == 0 && r == 0) break;   /* 已到地面, 停 */
    }
}

/* ===== 主任务 ===== */
static void *TrackTask(void *arg)
{
    (void)arg;
    int l = 0, r = 0;
    int state = S_DRIVE;
    int ret = 0, nfork = 0;
    int stack[MAX_FORKS];
    int zones = 0, black_n = 0, gap_n = 0, floor_n = 0;
    int seen_line = 0, bk_cnt = 0;
    int line_seen = 0;       /* 1=已沿主线跑过(终点判定前提) */
    int drive_ok = 0;        /* 连续正常循迹耸数 */
    int stuck_ms = 0;        /* 同一侧纠偏失败累计ms(轮胎解脱用) */
    int stuck_dir = 0;       /* 当前卡死侧方向 */
    int srch = 0;            /* 找线尝试计数 */
    uint64_t t0 = hi_get_us();
    uint32_t hopcnt = 0;

    printf("[T] IR track v8 start\r\n");

    while (1) {
        rd(&l, &r);

        switch (state) {
        case S_DRIVE:
            if (l == 1 && r == 1) {
                if (!seen_line) { state = S_BOOTBK; bk_cnt = 0; break; }
                zones = 1; black_n = 0; gap_n = 0; floor_n = 0;
                printf("[T] on black -> map\r\n");
                state = S_BLACK;
                break;
            }
            seen_line = 1;
            if (l == 0 && r == 0) {
                /* 居中: 直耸; 耸中压黑/持续偏线会提前停, 下次循环按新读数处理 */
                if (burst(0, 1) == 2) {
                    zones = 1; black_n = 0; gap_n = 0; floor_n = 0;
                    drive_ok = 0;
                    printf("[T] burst hit black -> map\r\n");
                    state = S_BLACK;
                    break;
                }
            } else {
                /* 单侧偏线: 原地转(左右正负对转)直到回中/压黑 */
                int dir = (l == 1) ? 1 : -1;
                int c;
                if (stuck_dir != dir) { stuck_dir = dir; stuck_ms = 0; }
                c = spin_until_center(dir, SPIN_MAX_MS);
                if (c == 2) {
                    zones = 1; black_n = 0; gap_n = 0; floor_n = 0;
                    drive_ok = 0;
                    stuck_ms = 0; stuck_dir = 0;
                    printf("[T] spin hit black -> map\r\n");
                    state = S_BLACK;
                    break;
                }
                if (c == 0) {
                    stuck_ms = 0; stuck_dir = 0;   /* 回中成功 */
                } else {
                    /* 一次没转回中: 累计; 同一侧累计>=4s => 轮胎卡死, 反向倒车解脱 */
                    stuck_ms += SPIN_MAX_MS;
                    if (stuck_ms >= RECOVER_STUCK_MS) {
                        printf("[T] stuck side %s for %d ms -> recover\r\n",
                               (l == 1) ? "(1,0)" : "(0,1)", stuck_ms);
                        /* (0,1) 向左倒; (1,0) 向右倒(用户要求) */
                        recover_reverse(l == 1 ? 0 : 1);
                        stuck_ms = 0; stuck_dir = 0;
                    }
                }
                /* c==0 已回中 / 尚未触发解脱: 下轮按新读数处理 */
            }
            if (++drive_ok >= 5) line_seen = 1;   /* 连续沿主线走几耸 => 已跑过主线 */
            hopcnt++;
            if (hopcnt % 5 == 0) printf("[D] L=%d R=%d\r\n", l, r);
            break;

        case S_BLACK:
            /* 压黑中: 耸(不提前停), 耸完读: 还黑=继续; 出黑=一段结束 */
            burst(0, 0);
            rd(&l, &r);
            printf("[B] L=%d R=%d\r\n", l, r);
            if (l == 1 && r == 1) {
                if (++black_n > ZONE_MAX_STEPS) {
                    /* 长黑异常: 退回地面, 当岔口处理 */
                    printf("[T] long black, back off\r\n");
                    motor(-SPEED_BACK, -SPEED_BACK);
                    usleep(3 * BURST_MS * 1000);
                    stop();
                    zones = 1;
                    floor_n = FLOOR_CLEAR_HOPS;   /* 直接判点 */
                    state = S_AFTER;
                    break;
                }
                break;               /* 继续压黑 */
            }
            black_n = 0;
            gap_n = 0;
            floor_n = 0;
            printf("[T] zone%d end, zones=%d\r\n", 0, zones);
            state = S_AFTER;
            break;

        case S_AFTER:
            /* 黑后观察: 用耸动全程采样流判定(压黑/碰线/纯地面) */
            {
                int code = burst(0, 0);
                rd(&l, &r);
                printf("[A] L=%d R=%d code=%d\r\n", l, r, code);
                if (code == 2) {
                    /* 途中又压到黑(第二段/另一条) */
                    if (gap_n <= ZONE_GAP_HOPS) {
                        zones = (zones >= 2) ? zones : 2;
                        printf("[T] double bar (zones=%d)\r\n", zones);
                    } else {
                        zones = 1;
                        printf("[T] far bar, new group\r\n");
                    }
                    black_n = 0; gap_n = 0; floor_n = 0;
                    state = S_BLACK;
                    break;
                }
                if (code == 1) {
                    /* 途中碰到线: 主线还在(路过), 回DRIVE */
                    printf("[T] line resumes\r\n");
                    line_seen = 1;
                    zones = 0; gap_n = 0; floor_n = 0;
                    state = S_DRIVE;
                    break;
                }
                /* code==0 纯地面 */
                gap_n++;
                floor_n++;
            if (floor_n >= FLOOR_CLEAR_HOPS) {
                /* 到点 */
                printf("[T] point zones=%d ret=%d\r\n", zones, ret);
                if (zones >= 2) {
                    if (line_seen) {
                        printf("[T] GOAL (double, no line, ran track)\r\n");
                        state = S_GOAL;
                    } else {
                        /* 还没跑过主线就遇双条: 刚越过起点, 找主线 */
                        printf("[T] start double passed, search line\r\n");
                        srch = 0;
                        state = S_SEARCH;
                    }
                    break;
                }
                if (zones == 0) zones = 1;
                /* 探臂前先倒车回靠岔口边缘, 避免已溜进岔口腹地探不到臂 */
                back_to_junction();
                {
                    int want, got = 0;
                    if (!ret) {
                        /* 按岔口序号定偏好: 第1个岔口走左, 第2个走右(实测正确路线) */
                        want = (nfork % 2 == 0) ? 1 : -1;
                        got = probe_side(want, SCAN_PROBE_MS);
                        if (!got) { want = -want; got = probe_side(want, SCAN_PROBE_MS); }
                        if (!got) {
                            printf("[T] no arm -> DEAD\r\n");
                            state = S_DEAD;
                            break;
                        }
                        if (want > 0) motor(-SPEED_TURN, SPEED_TURN);
                        else          motor(SPEED_TURN, -SPEED_TURN);
                        usleep(TURN_ARM_MS * 1000);
                        stop();
                        if (nfork < MAX_FORKS) stack[nfork] = want;
                        nfork++;
                        printf("[T] fork#%d take %s\r\n", nfork, want > 0 ? "L" : "R");
                    } else {
                        int last = (nfork > 0) ? stack[nfork - 1]
                                               : ((nfork % 2 == 0) ? 1 : -1);
                        want = -last;
                        got = probe_side(want, SCAN_PROBE_MS);
                        if (!got) { want = -want; got = probe_side(want, SCAN_PROBE_MS); }
                        if (!got) {
                            printf("[T] ret no arm?! -> DEAD\r\n");
                            state = S_DEAD;
                            break;
                        }
                        if (want > 0) motor(-SPEED_TURN, SPEED_TURN);
                        else          motor(SPEED_TURN, -SPEED_TURN);
                        usleep(TURN_ARM_MS * 1000);
                        stop();
                        printf("[T] ret fork last=%s take %s\r\n",
                               last > 0 ? "L" : "R", want > 0 ? "L" : "R");
                        if (nfork > 0) nfork--;
                        ret = 0;
                    }
                }
                zones = 0; gap_n = 0; floor_n = 0;
                state = S_DRIVE;
                usleep(100000);
            }
            }
            break;

        case S_SEARCH:
            /* 越过起点后找主线: 直耸/左右弧耸交替, 见线即回DRIVE */
            {
                int kind = (srch % 2 == 0) ? 0 : ((srch % 4 == 1) ? 1 : -1);
                int code = burst(kind, 0);
                printf("[S] try %d L=%d R=%d code=%d\r\n", srch, l, r, code);
                if (code == 2) {
                    zones = 1; black_n = 0; gap_n = 0; floor_n = 0;
                    state = S_BLACK;
                    break;
                }
                if (code == 1) {
                    line_seen = 1;
                    printf("[T] line found, run\r\n");
                    state = S_DRIVE;
                    break;
                }
                if (++srch >= 10) {
                    printf("[T] cannot find line, HALT\r\n");
                    state = S_HALT;
                }
            }
            break;

        case S_DEAD:
            stop();
            usleep(200000);
            motor(-SPEED_BACK, -SPEED_BACK);
            usleep(BACK_CLEAR_MS * 1000);
            stop();
            usleep(150000);
            motor(-SPEED_TURN, SPEED_TURN);
            usleep(TURN_180_MS * 1000);
            stop();
            usleep(150000);
            ret = 1;
            t0 = hi_get_us();
            printf("[T] DEAD done, retracing\r\n");
            zones = 0; gap_n = 0; floor_n = 0;
            state = S_DRIVE;
            break;

        case S_BOOTBK:
            motor(-SPEED_BACK, -SPEED_BACK);
            usleep(BOOT_BACK_MS * 1000);
            stop();
            usleep(80000);
            rd(&l, &r);
            if (!(l == 1 && r == 1)) {
                printf("[T] off black, start\r\n");
                seen_line = 1;
                state = S_DRIVE;
            } else if (++bk_cnt >= BOOT_BACK_TRIES) {
                printf("[T] cannot leave black, HALT\r\n");
                state = S_HALT;
            }
            break;

        case S_GOAL:
            stop();
            printf("[T] ARRIVED at GOAL, stop!\r\n");
            for (;;) usleep(1000000);

        case S_HALT:
            stop();
            printf("[T] HALT (align car on line, then reset)\r\n");
            for (;;) usleep(1000000);

        default:
            state = S_DRIVE;
            break;
        }

        if (ret && (hi_get_us() - t0) / 1000000 > RETRACE_MAX_S) {
            printf("[T] retrace timeout, give up\r\n");
            ret = 0;
        }
    }
    return NULL;
}

/* ===== 入口 ===== */
static void ir_track_demo(void)
{
    osThreadAttr_t attr;
    WifiIotUartAttribute uattr;

    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    IoSetPull(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_PULL_UP);
    IoSetPull(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_PULL_UP);
    GpioSetDir(PIN_IR_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(PIN_IR_R, WIFI_IOT_GPIO_DIR_IN);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    UartInit(UART_STM32, &uattr, NULL);

    memset(&attr, 0, sizeof(attr));
    attr.name = "TrackTask";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)TrackTask, NULL, &attr);

    printf("[T] IR track ready!\r\n");
}

APP_FEATURE_INIT(ir_track_demo);
