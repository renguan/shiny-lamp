# QST先锋号智能小车 开发项目

基于 **QST先锋号(鸿蒙)智能小车** 的开发学习项目，包含 STM32 与 OpenHarmony(Hi3861) 双系统的实验代码与工作日志。

## 项目结构

```
driveless/
├── STM32/                          # STM32F103C8T6 侧 (Keil MDK5 工程)
│   ├── 3_PWM驱动电机/              # 任务21: STM32 TIMER PWM 驱动电机
│   ├── 4_编码器测速/               # 任务22: STM32 TIMER 编码器测速
│   ├── 5_PID速度闭环/              # 任务23: STM32 PID 电机速度闭环控制
│   └── 6_距离控制/                 # 任务24: STM32 距离控制(1米往返)
├── Hi3861/                         # Hi3861 (OpenHarmony) 侧
│   ├── 任务7_GPIO驱动舵机/         # 任务7: OpenHarmony GPIO 驱动舵机(互斥锁多任务)
│   ├── 6.0_Sum_Experiment_First/   # 任务10: 第一阶段综合实验(舵机测距+红外寻线+蓝牙+消息队列)
│   ├── 7.0_I2c_Ssd1306_OLED/       # 任务11: I2C驱动SSD1306 OLED显示(含中文"鸿蒙先锋号")
│   ├── Cliff_Detection/            # 扩展: 悬崖避障(红外对管检测边缘+后退转向)
│   ├── BLE_Car_Control/            # 扩展: 手机蓝牙遥控小车(前进/后退/转弯, 任务26, 已搁置交同组)
│   ├── Auto_Drive_2min/            # 扩展(旧版v1~v3, 已弃用): 教室自主行驶2分钟
│   └── Ultrasonic_2min_Drive/      # 扩展(最终版): 教室自主行驶2分钟·超声波避障, 心跳发帧
├── 26-8-25.md                      # 工作日志 2026-08-25
├── 26-8-26.md                      # 工作日志 2026-08-26
├── 26-8-27.md                      # 工作日志 2026-08-27
├── 26-8-31.md                      # 工作日志 2026-08-31
├── 26-9-1.md                       # 工作日志 2026-09-01
├── 26-9-2.md                       # 工作日志 2026-09-02
├── 26-9-3.md                       # 工作日志 2026-09-03
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
- **实测参数（已按实车标定，任务22代码内为教材默认值）**：`ENCODER_PULSES_PER_REV = 700*4`、`WHEEL_CIRCUMFERENCE = 0.1382m`（轮径 4.4cm）
- 文件：`QST_HARDWARE/encoder/encoder.c|h`、`USER/main.c`

### 任务23：STM32 PID 电机速度闭环控制（`STM32/5_PID速度闭环/`）

增量式 PI 速度闭环：编码器测速（任务22）作反馈，PWM 驱动（任务21）作输出，实现电机按目标转速稳定运行。

- 控制周期 100ms：`读编码器 → 算目标脉冲数 → PI计算 → Set_Pwm 输出`
- 增量式 PI：`Pwm += Kp·[e(k)-e(k-1)] + Ki·e(k)`，Kp=7.0、Ki=0.010，输出限幅 ±7199
- 目标换算：`Rs_To_CR(r) = r × (700×4) / (1000/100)`（**实车 700 线/转，2800 脉冲/转**）
- 实车联调已修正：Set_Pwm 引脚配对（moto1=左轮 AIN+PWMB、moto2=右轮 BIN+PWMA）、编码器方向取负使前进为正、双轮目标同向
- 文件：`QST_HARDWARE/SYSTEM_CONTROL/control_system.c|h` + 复用 `motor`、`encoder`

### 任务24：STM32 距离控制（`STM32/6_距离控制/`）

前进 1 米 → 停 0.5 秒 → 后退 1 米 → 回起点，状态机实现（0前进→1停止→2后退→3结束）。

- 距离标定：`DISTANCE_COUNTS_PER_METER = 20200`（轮径 4.4cm→周长 0.1382m，700×4 线，实测折中）
- 速度参数：`FORWARD_RPS = 1.5`、`BACKWARD_RPS = -1.7`（后退补偿，按实车微调）
- 已修复 4 个 bug：Set_Pwm 引脚配对交叉；SysTick 中 millis 被清零致状态机计时失效；编码器取负致距离累计不到目标；后退偏慢（清零 PI 累加器立即反转）
- 文件：`QST_HARDWARE/SYSTEM_CONTROL/control_system.c|h` + 复用 `motor`、`encoder`

### 任务10：第一阶段综合实验（`Hi3861/6.0_Sum_Experiment_First/`）

综合舵机、超声波、红外对管、蓝牙、UART、消息队列、多任务知识点，实现多级联动。

- **Task1 舵机左右旋转测距**：SG90（GPIO2，20ms 周期 PWM）左/中/右转动 + HC-SR04 超声波（TRIG=GPIO7、ECHO=GPIO8）测距，结果入消息队列
- **Task2 前15秒红外寻线，15秒后蓝牙通信**：红外对管（GPIO13 左/GPIO14 右）寻线逻辑，通过 UART2（GPIO11/12）协议帧（0xFC+方向+速度+0xFD）指挥 STM32 驱动电机；15 秒后切换为读取 UART1（GPIO0/1，蓝牙模块）数据
- **Task3 消息队列打印**：`osMessageQueueGet` 阻塞接收，按类型打印测距/蓝牙/寻线信息
- **任务1/2 交替运行**：同优先级（osPriorityNormal）时间片轮转
- 文件：`Sum_Experiment_First.c`、`BUILD.gn`（需 `CONFIG_I2C_SUPPORT=y`，见下）

### 任务11：OLED 显示字符串（含中文）（`Hi3861/7.0_I2c_Ssd1306_OLED/`）

I2C 驱动 SSD1306 OLED 显示字符串，并在示范代码基础上实现中文"鸿蒙先锋号"显示。

- **坑**：SDK `build/config/usr_config.mk` 中 `CONFIG_I2C_SUPPORT` 默认未使能，导致链接报 `undefined reference to hi_i2c_*`；改为 `CONFIG_I2C_SUPPORT=y` 后 `libi2c.a` 正常生成
- **中文支持**：fonts.h 新增 `F16x16` 16x16 中文字库（"鸿蒙先锋号"5 字取模）；新增 `SSD1306_ShowCN(x, y, ch[])` 按 UTF-8 匹配显示
- Task1 显示"鸿蒙先锋号"（居中）+ 每秒刷新的实时时钟
- 文件：`I2c_Ssd1306.c`、`include/`、`src/`

### 悬崖避障（`Hi3861/Cliff_Detection/`）— 扩展功能

小车在台面上自主运动，车前端底盘下的红外对管自动识别台面边缘（悬崖），到达边缘时停下并后退一小段距离避开，然后转向继续运动。

- 红外对管：GPIO13（左）/GPIO14（右），朝下检测；有反射（台面）=0，无反射（边缘悬空）=1，任一对管为 1 判定到边缘
- 电机：Hi3861 经 UART2（GPIO11/12）发协议帧（0xFC..0xFD）指挥 STM32
- 行为循环：前进（低速 80，10ms 轮询）→ 检测到边缘 → **立即反转制动+后退 2.5s** → 左/右交替转向 1.5s → 继续前进（`SPEED_FWD`/`BACK_MS`/`TURN_MS` 可按实车微调）
- 文件：`Cliff_Detection.c`、`BUILD.gn`

### 手机蓝牙遥控小车（`Hi3861/BLE_Car_Control/`）— 扩展功能（任务26）

手机蓝牙串口助手连接蓝牙模块，发送指令遥控小车前进/后退/转弯。

- 蓝牙模块：UART1（GPIO0/1，**波特率 9600**，JDY-16）
- 指令协议（老师规范，单个字符）：`0`停 `W`前 `A`左 `D`右 `S`后 `I`(100,100) `K`(150,150)
- **学生扩展「一键系列动作」**：按 `E` 触发整套动作序列（前进1s→左转0.8s→前进1s→右转0.8s→后退1s→停），用「步进表 + 状态机」实现，`demo_seq[]` 表可随意增删/改顺序/改时长，序列中按 `0` 或任意方向键立即打断
- 安全：3 秒无指令自动停车（防手机断开乱跑）
- 电机：UART2（GPIO11/12）协议帧指挥 STM32
- **双串口（UART1 蓝牙 + UART2 电机）需同时初始化**：依赖 `app_main.c` 中 `APP_INIT_EVENT_NUM` 改为 **7**（默认 4 不够，详见 `26-9-2.md`）
- 文件：`BLE_Car_Control.c`、`BUILD.gn`

### 教室自主行驶2分钟·超声波避障（`Hi3861/Auto_Drive_2min/`）— 扩展功能【旧版 v1~v3，已弃用，最终版见 Ultrasonic_2min_Drive】

小车放在教室里自动（非遥控）行驶约 **2 分钟**，利用超声波测距避障，全程不撞东西，到点自动停车。

- 超声波 HC-SR04 装在 **GPIO2 舵机云台**上（TRIG=GPIO7、ECHO=GPIO8），可左/中/右转向测距
- 行为（v2）：**短段前进 0.9s → 停车三向扫描（左/中/右）→ 定航向 → 循环**；前进中正前 <40cm 连续 2 次（或 <15cm 单次）判障 → 制动+后退 → **强制转向较空一侧（小步 300ms/步 + 每步测距验证，前方打通即停转）**；两侧都堵 → 调头（多轮失败再后退重试）；正前畅通但**侧墙逼近（<45cm）→ 小角度反向修正**（防斜向撞墙）；前进**左右轮差速可调**（默认左快右慢，抵消直行偏左）；循环到 120s 自动停
- 运动：UART2（GPIO11/12）协议帧指挥 STM32（0xFC..0xFD）
- 文件：`auto_drive_2min.c`、`BUILD.gn`；实车调参（速度/阈值/时长宏、方向核对）见 `26-9-3.md`

### 教室自主行驶2分钟·超声波避障·最终版（`Hi3861/Ultrasonic_2min_Drive/`）— 扩展功能

教室自主行驶 2 分钟任务的**全新最终版**（旧版 `Auto_Drive_2min` v1~v3 已弃用，勿混用）。

- 硬件：超声波 HC-SR04 装在 GPIO2 舵机云台（TRIG=GPIO7、ECHO=GPIO8）；电机经 UART2（GPIO11/12）协议帧指挥 STM32
- **心跳式发帧**：前进/后退/转向/制动指令每 30ms 重发一帧（MotorDiag 实测：单发一帧不动、持续发帧能动）
- 行为：前进（云台居中盯正前，<30cm×2 次或 <14cm 1 次判障）→ 反转制动 250ms → 后退 500ms → 云台扫左/右选空侧 → 小步转向（每步验证正前 ≥45cm 打通即停）→ 循环至 120s 自动停车；两侧都堵 → 原地调头 180°
- 文件：`ultrasonic_2min_drive.c`、`BUILD.gn`、`Hi3861_wifiiot_app_allinone.bin`（虚拟机编译产物）、`使用说明.md`（编译/烧录/方向核对/调参）

### 任务7：OpenHarmony GPIO 驱动舵机（`Hi3861/任务7_GPIO驱动舵机/`）

OpenHarmony(Hi3861) 下用 GPIO 产生 PWM 驱动 SG90 舵机，并通过互斥锁实现同优先级三任务联动。

- 工程：`3.0_SG90_Mutex/`（放入 OpenHarmony 源码 `applications/sample/wifi-iot/app/` 后执行 `python build.py wifiiot`）
- 任务1：优先运行，串口输出 1 次，舵机左转 45°
- 任务3：任务1 运行 3 秒后，串口输出 2 次，舵机右转 45°
- 任务2：任务3 之后立即运行，串口输出 3 次，舵机居中
- 共享资源 `angle_flag` 在互斥锁保护下独占访问

## 烧录方法

### STM32 侧（Keil + ST-Link）
1. Keil 打开 `STM32/3_PWM驱动电机/USER/PWM_Motor.uvprojx`、`STM32/4_编码器测速/USER/Encoder_Speed.uvprojx`、`STM32/5_PID速度闭环/USER/PID_Speed.uvprojx` 或 `STM32/6_距离控制/USER/Distance_Control.uvprojx`，F7 编译（工程已内置 ST-Link 配置）
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
