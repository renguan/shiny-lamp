# QST先锋号智能小车 开发项目

基于 **QST先锋号(鸿蒙)智能小车** 的开发学习项目，包含 STM32 与 OpenHarmony(Hi3861) 双系统的实验代码与工作日志。

## 项目结构

```
driveless/
├── STM32/                          # STM32F103C8T6 侧 (Keil MDK5 工程)
│   ├── 3_PWM驱动电机/              # 任务21: STM32 TIMER PWM 驱动电机
│   └── 4_编码器测速/               # 任务22: STM32 TIMER 编码器测速
├── Hi3861/                         # Hi3861 (OpenHarmony) 侧
│   └── 任务7_GPIO驱动舵机/         # 任务7: OpenHarmony GPIO 驱动舵机(互斥锁多任务)
├── 26-8-25.md                      # 工作日志 2026-08-25
├── 26-8-26.md                      # 工作日志 2026-08-26
├── .gitignore
└── README.md
```

## 硬件与开发环境

| 部分 | 说明 |
|---|---|
| 主控(运动控制) | STM32F103C8T6，Keil MDK5 (ARMCC V5.06)，ST-Link 烧录 |
| 鸿蒙侧 | Hi3861，OpenHarmony 1.0 源码在 Ubuntu 虚拟机交叉编译，HiBurn 烧录 |
| 电机驱动 | L9110S × 2（PWM 调速 + 方向控制） |
| 测速反馈 | 霍尔编码器 × 2（TIM2/TIM3 编码器接口模式） |
| 舵机 | SG90（GPIO2，20ms 周期 0.5~2.5ms 脉宽控制 0~180°） |
| 串口 | 115200，板载 CH340 |

## 任务说明

### 任务21：STM32 TIMER PWM 驱动电机（`STM32/3_PWM驱动电机/`）

用 TIM4 产生 1000Hz PWM 驱动左右电机，实现前进。

- **引脚映射**（已对照原理图核实）：
  | 信号 | 引脚 |
  |---|---|
  | AIN（左电机方向） | PB14（L-IB） |
  | BIN（右电机方向） | PB13（R-IA） |
  | PWMA（右电机 PWM） | PB6 = TIM4_CH1（R-IB） |
  | PWMB（左电机 PWM） | PB7 = TIM4_CH2（L-IA） |
- 参数：ARR=7199，PSC=9 → 1000Hz；`Set_Pwm(2500,2500)` 前进
- 文件：`QST_HARDWARE/motor/motor.c|h`、`USER/main.c`

### 任务22：STM32 TIMER 编码器测速（`STM32/4_编码器测速/`）

编码器接口模式测速，并把脉冲数换算成小车速度(m/s)打印。

- 左电机编码器：PA0/PA1 → TIM2（编码器模式 TI12，四倍频）
- 右电机编码器：PA6/PA7 → TIM3
- 每 100ms 读一次 `TIMx->CNT` 并清零（SysTick 1ms 中断计数）
- 速度换算：`速度(m/s) = 脉冲数 / 每圈总脉冲数 × 车轮周长 / 时间间隔`
- **实测参数（需按实车修正）**：`ENCODER_PULSES_PER_REV = 360*4`、`WHEEL_CIRCUMFERENCE = 0.204m`（默认直径 65mm）
- 文件：`QST_HARDWARE/encoder/encoder.c|h`、`USER/main.c`

### 任务7：OpenHarmony GPIO 驱动舵机（`Hi3861/任务7_GPIO驱动舵机/`）

OpenHarmony(Hi3861) 下用 GPIO 产生 PWM 驱动 SG90 舵机，并通过互斥锁实现同优先级三任务联动。

- 工程：`3.0_SG90_Mutex/`（放入 OpenHarmony 源码 `applications/sample/wifi-iot/app/` 后执行 `python build.py wifiiot`）
- 任务1：优先运行，串口输出 1 次，舵机左转 45°
- 任务3：任务1 运行 3 秒后，串口输出 2 次，舵机右转 45°
- 任务2：任务3 之后立即运行，串口输出 3 次，舵机居中
- 共享资源 `angle_flag` 在互斥锁保护下独占访问

## 烧录方法

### STM32 侧（Keil + ST-Link）
1. Keil 打开 `STM32/3_PWM驱动电机/USER/PWM_Motor.uvprojx` 或 `STM32/4_编码器测速/USER/Encoder_Speed.uvprojx`，F7 编译（工程已内置 ST-Link 配置）
2. ST-Link 连接，点击 Download
3. 串口开关拨到 **STM32 端**，串口助手 115200 观察输出

### Hi3861 侧（HiBurn）
1. 虚拟机编译 `python build.py wifiiot`，产物 `Hi3861_wifiiot_app_allinone.bin`
2. 串口开关拨到 **5861 端**，数据线供电（关闭小车电源开关）
3. HiBurn 加载 allinone bin → Connect → 按复位键烧录

## 注意事项

- 电机由**电池**供电（M-5V），USB 只能给逻辑部分供电；烧录 Hi3861 时则要关闭电源开关
- PWM 设置值不能超过重装载值 7199
- 编码器测速参数（每圈脉冲数、车轮周长）需实测后修正
- 工作日志按日期命名（`26-8-XX.md`）
