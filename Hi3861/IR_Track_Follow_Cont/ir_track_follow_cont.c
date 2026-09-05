/*
 * IR_Track_Follow_Cont: 连续行驶版(无耸动) - 实验分支
 *
 * 与耸动版(IR_Track_Follow)共享判定思路, 但行驶方式不同:
 *   直线段 速度100 一直走, 每10ms采样; 单侧偏线 -> 原地转回中(短暂停);
 *   压黑(1,1) -> 保持直行穿过, 按"连续采样时间"数区段(单/双条);
 *   黑后前方持续无带 -> 到点: 探臂(先左后右)判岔口/死路/终点。
 *
 * 判据全部用"毫秒/采样数"(速度100下的距离换算需上车标定):
 *   ZONE_MIN_MS     (1,1)持续>=此ms才算压黑(滤毛刺)
 *   ZONE_GAP_MS     两段黑间隔<=此ms => 双条(实车: 双条间距3cm)
 *   FLOOR_CLEAR_MS  黑后连续无带>=此ms => 到点
 *   BLACK_MAX_MS    压黑超过此ms => 长黑异常, 退回按岔口处理
 *
 * 终点判定: 双条 + 无主线, 且此前已沿主线跑过(line_seen) => 终点停车;
 *   未跑过主线 => 越过的是起点, 进入 S_SEARCH 找线。
 * 起点摆放建议: 越过起点双横条、压在主线直路上, 车头朝终点。
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

/* ===== 运动 ===== */
#define SPEED_GO         100        /* 直线行驶速度(用户指定) */
#define SPEED_TURN       120        /* 原地转速度 */
#define SPEED_BACK       80
#define SPIN_MAX_MS      800        /* 纠偏原地转最长时长 */
#define TURN_PROBE_MS    450        /* 探臂转角时长 */
#define TURN_180_MS      1700       /* 死路掉头 */
#define BACK_CLEAR_MS    400
#define RETRACE_MAX_S    25
#define BOOT_BACK_MS     300
#define BOOT_BACK_TRIES  6

/* ===== 判定(毫秒, 10ms采样换算) ===== */
#define TRACE_FLIP       0
#define PREFER_LEFT      1
#define SAMPLE_US        10000
#define ZONE_MIN_MS      30         /* (1,1)持续>=30ms才算压黑 */
#define ZONE_GAP_MS      700        /* 两段黑间隔<=700ms => 双条(间距3cm, 待标定) */
#define FLOOR_CLEAR_MS   700        /* 黑后连续无带>=700ms => 到点 */
#define BLACK_MAX_MS     2500       /* 压黑超时=长黑 */
#define MAX_FORKS        4

enum {
    S_DRIVE = 0, S_CROSS, S_WATCH, S_DEAD, S_GOAL, S_BOOTBK, S_HALT, S_SEARCH
};

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
static void go(void)   { motor(SPEED_GO, SPEED_GO); }
static void stop(void)
{
    int i;
    for (i = 0; i < 4; i++) { motor(0, 0); usleep(15000); }
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

/* 原地转直到回中/压黑(边转边采样); 返回 0回中 2压黑 -1超时 */
static int spin_until_center(int dir, int max_ms)
{
    int i, l = 0, r = 0, ctr = 0;
    int n = max_ms * 1000 / SAMPLE_US;
    for (i = 0; i < n; i++) {
        if (dir > 0) motor(-SPEED_TURN, SPEED_TURN);
        else         motor(SPEED_TURN, -SPEED_TURN);
        usleep(SAMPLE_US);
        rd(&l, &r);
        if (l == 1 && r == 1) { stop(); usleep(80000); return 2; }
        if (l == 0 && r == 0) {
            if (++ctr >= 2) { stop(); usleep(80000); return 0; }
        } else ctr = 0;
    }
    stop();
    usleep(80000);
    return -1;
}

/* 探臂: side>0左 -1右 */
static int probe_side(int side, int probe_ms, int wait_ms)
{
    int i, l = 0, r = 0;
    int n = wait_ms * 1000 / SAMPLE_US;
    if (side > 0) motor(-SPEED_TURN, SPEED_TURN);
    else          motor(SPEED_TURN, -SPEED_TURN);
    usleep(probe_ms * 1000);
    stop();
    for (i = 0; i < n; i++) {
        rd(&l, &r);
        if (l == 1 || r == 1) return 1;
        usleep(SAMPLE_US);
    }
    return 0;
}

/* ===== 主任务 ===== */
static void *TrackTask(void *arg)
{
    (void)arg;
    int l = 0, r = 0;
    int state = S_DRIVE;
    int ret = 0, nfork = 0;
    int stack[MAX_FORKS];
    int zones = 0;
    int line_seen = 0, drive_ms = 0;   /* drive_ms: 连续沿主线行驶毫秒 */
    int seen_line = 0, bk_cnt = 0, srch = 0;
    int black_ms = 0, floor_ms = 0, gap_ms = 0;
    uint64_t t0 = hi_get_us();
    uint32_t diag = 0;

    printf("[T] IR track CONT (v12) start\r\n");
    state = S_DRIVE;

    while (1) {
        rd(&l, &r);

        switch (state) {
        case S_DRIVE:
            if (l == 1 && r == 1) {
                if (!seen_line) { state = S_BOOTBK; bk_cnt = 0; break; }
                stop();
                zones = 1; black_ms = 0; gap_ms = 0; floor_ms = 0;
                printf("[T] black -> cross\r\n");
                state = S_CROSS;
                break;
            }
            seen_line = 1;
            if (l == 0 && r == 0) {
                go();
                drive_ms += SAMPLE_US / 1000;
                if (drive_ms >= 500) { line_seen = 1; drive_ms = 0; }
            } else {
                /* 单侧偏线: 原地转回中(短暂停), 再继续直行 */
                int dir = (l == 1) ? 1 : -1;
                int c = spin_until_center(dir, SPIN_MAX_MS);
                drive_ms = 0;
                if (c == 2) {
                    zones = 1; black_ms = 0; gap_ms = 0; floor_ms = 0;
                    state = S_CROSS;
                    break;
                }
                go();
            }
            if (++diag % 50 == 0) printf("[D] L=%d R=%d\r\n", l, r);
            usleep(SAMPLE_US);
            break;

        case S_CROSS:
            /* 压黑: 保持直行穿过, 按时间计数 */
            go();
            black_ms += SAMPLE_US / 1000;
            if (l == 1 && r == 1) {
                if (black_ms > BLACK_MAX_MS) {
                    /* 长黑: 退回, 按单区岔口处理 */
                    printf("[T] long black, back off\r\n");
                    stop();
                    motor(-SPEED_BACK, -SPEED_BACK);
                    usleep(1000);
                    stop();
                    zones = 1;
                    floor_ms = FLOOR_CLEAR_MS;   /* 直接判点 */
                    state = S_WATCH;
                    break;
                }
                usleep(SAMPLE_US);
                break;              /* 继续压黑 */
            }
            /* 离开黑区: 一段结束 */
            black_ms = 0; gap_ms = 0; floor_ms = 0;
            printf("[T] zone end, zones=%d\r\n", zones);
            state = S_WATCH;
            usleep(SAMPLE_US);
            break;

        case S_WATCH:
            /* 黑后观察(直行): 又来黑/单1主线/连续地面=到点 */
            go();
            if (l == 1 && r == 1) {
                if (gap_ms <= ZONE_GAP_MS) {
                    zones = (zones >= 2) ? zones : 2;
                    printf("[T] double bar (zones=%d)\r\n", zones);
                } else {
                    zones = 1;
                    printf("[T] far bar, new group\r\n");
                }
                black_ms = 0; gap_ms = 0; floor_ms = 0;
                state = S_CROSS;
                usleep(SAMPLE_US);
                break;
            }
            if (l == 1 || r == 1) {
                printf("[T] line resumes\r\n");
                line_seen = 1;
                zones = 0; gap_ms = 0; floor_ms = 0;
                state = S_DRIVE;
                usleep(SAMPLE_US);
                break;
            }
            gap_ms += SAMPLE_US / 1000;
            floor_ms += SAMPLE_US / 1000;
            if (floor_ms >= FLOOR_CLEAR_MS) {
                /* 到点 */
                stop();
                printf("[T] point zones=%d ret=%d\r\n", zones, ret);
                if (zones >= 2) {
                    if (line_seen) {
                        printf("[T] GOAL!\r\n");
                        state = S_GOAL;
                    } else {
                        printf("[T] start passed, search line\r\n");
                        srch = 0;
                        state = S_SEARCH;
                    }
                    break;
                }
                if (zones == 0) zones = 1;
                {
                    int want, got = 0;
                    if (!ret) {
                        want = PREFER_LEFT ? 1 : -1;
                        got = probe_side(want, TURN_PROBE_MS, 700);
                        if (!got) { want = -want; got = probe_side(want, TURN_PROBE_MS, 700); }
                        if (!got) {
                            printf("[T] no arm -> DEAD\r\n");
                            state = S_DEAD;
                            break;
                        }
                        if (want > 0) motor(-SPEED_TURN, SPEED_TURN);
                        else          motor(SPEED_TURN, -SPEED_TURN);
                        usleep(TURN_PROBE_MS * 1000);
                        stop();
                        if (nfork < MAX_FORKS) stack[nfork] = want;
                        nfork++;
                        printf("[T] fork#%d take %s\r\n", nfork, want > 0 ? "L" : "R");
                    } else {
                        int last = (nfork > 0) ? stack[nfork - 1]
                                               : (PREFER_LEFT ? 1 : -1);
                        want = -last;
                        got = probe_side(want, TURN_PROBE_MS, 700);
                        if (!got) { want = -want; got = probe_side(want, TURN_PROBE_MS, 700); }
                        if (!got) {
                            printf("[T] ret no arm?! -> DEAD\r\n");
                            state = S_DEAD;
                            break;
                        }
                        if (want > 0) motor(-SPEED_TURN, SPEED_TURN);
                        else          motor(SPEED_TURN, -SPEED_TURN);
                        usleep(TURN_PROBE_MS * 1000);
                        stop();
                        printf("[T] ret fork last=%s take %s\r\n",
                               last > 0 ? "L" : "R", want > 0 ? "L" : "R");
                        if (nfork > 0) nfork--;
                        ret = 0;
                    }
                }
                zones = 0; gap_ms = 0; floor_ms = 0;
                state = S_DRIVE;
                usleep(150000);
            }
            usleep(SAMPLE_US);
            break;

        case S_SEARCH:
            /* 越过起点后找主线: 直行/左转/右转交替(每次一小段), 见线回DRIVE */
            {
                int kind = (srch % 2 == 0) ? 0 : ((srch % 4 == 1) ? 1 : -1);
                if (kind == 0) {
                    go();
                    usleep(200000);
                    stop();
                } else if (kind > 0) {
                    motor(-SPEED_TURN, SPEED_TURN);
                    usleep(200000);
                    stop();
                } else {
                    motor(SPEED_TURN, -SPEED_TURN);
                    usleep(200000);
                    stop();
                }
                printf("[S] try %d L=%d R=%d\r\n", srch, l, r);
                if (l == 1 && r == 1) {
                    zones = 1; black_ms = 0; gap_ms = 0; floor_ms = 0;
                    state = S_CROSS;
                    break;
                }
                if (l == 1 || r == 1) {
                    line_seen = 1;
                    printf("[T] line found\r\n");
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
            zones = 0; gap_ms = 0; floor_ms = 0;
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
            printf("[T] HALT (align car, reset)\r\n");
            for (;;) usleep(1000000);

        default:
            state = S_DRIVE;
            break;
        }

        if (ret && (hi_get_us() - t0) / 1000000 > RETRACE_MAX_S) {
            printf("[T] retrace timeout\r\n");
            ret = 0;
        }
    }
    return NULL;
}

/* ===== 入口 ===== */
static void ir_track_cont_demo(void)
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

    printf("[T] IR track CONT ready!\r\n");
}

APP_FEATURE_INIT(ir_track_cont_demo);
