/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 机器狗运动状态：本测试只有“停止”和“前进”两种 */
typedef enum
{
  MOTION_STOP = 0,     /* 站立，四腿回到中位 */
  MOTION_FORWARD       /* 按 S1 后持续前进，直到前方 10cm 内出现障碍 */
} Motion;

/* VL53L0X 单次测距的非阻塞状态机 */
typedef enum
{
  TOF_STATE_IDLE = 0,  /* 空闲，等待下一次测量周期 */
  TOF_STATE_WAIT_START,/* 已写 SYSRANGE_START，等待启动位自动清零 */
  TOF_STATE_WAIT_READY /* 测量中，轮询中断状态位 */
} TofState;

/* 测量序列中各步骤的开关（SYSTEM_SEQUENCE_CONFIG 的位） */
typedef struct
{
  uint8_t tcc;
  uint8_t dss;
  uint8_t msrc;
  uint8_t pre_range;
  uint8_t final_range;
} TOF_SeqEnables;

/* 各步骤的 VCSEL 周期与超时，时间预算计算要用 */
typedef struct
{
  uint8_t  pre_range_vcsel_period_pclks;
  uint8_t  final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks;
  uint16_t pre_range_mclks;
  uint16_t final_range_mclks;
  uint32_t msrc_dss_tcc_us;
  uint32_t pre_range_us;
  uint32_t final_range_us;
} TOF_SeqTimeouts;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- 舵机（TIM1 CH1~CH4 = PA8/PA9/PA10/PA11，50Hz） ---- */
#define SERVO_COUNT             4U
#define SERVO_MIN_US            500U     /* 0° 脉宽 */
#define SERVO_MAX_US            2500U    /* 180° 脉宽 */

/* ---- 机械尺寸（mm）----
   与 ble_walk_test 相同，改这三个宏即可重新标定，步态角度会自动重算。 */
#define BODY_WHEELBASE_MM       75.0f    /* 前后腿轴距 */
#define BODY_TRACK_MM           82.0f    /* 左右腿轴距（本测试不转向，仅保留标定） */
#define LEG_LENGTH_MM           46.0f    /* 腿长（舵机轴心到脚底） */

/* ---- 步态参数（对角小跑 Trot）----
   单舵机腿绕水平轴前后摆，靠支撑相/摆动相的时间不对称产生净位移：
     - 支撑相占 GAIT_STANCE_PCT，脚缓慢划动，接地时间长、冲量大；
     - 摆动相快速回摆，接地时间短、反向冲量小。
   两组对角腿同步：PA8+PA11 与 PA9+PA10，交替摆动。

   周期 550ms、支撑 80%：支撑段约 440ms，复位段约 110ms；
   44mm 步幅对应约 ±28.6°，约 57°摆角在复位段内完成。 */
#define GAIT_CENTER_DEG         90       /* 腿竖直向下 */
#define GAIT_STRIDE_MM          44.0f    /* 前进每周期脚的前后行程 */
#define GAIT_CYCLE_MS           550U     /* 一个完整步态周期 */
#define GAIT_TICK_MS            20U      /* 步态更新周期 */
#define GAIT_STANCE_PCT         80U      /* 支撑相占比；复位相占剩余 20% */
#define GAIT_SLEW_DEG           8        /* 每 tick 最大角度变化；必须大于
                                            摆动相速度，否则会把快摆削平 */
#define GAIT_MAX_TILT_DEG       30.0f    /* 腿前后最大摆角（机械限位保护） */
#define GAIT_MIN_FOOT_GAP_MM    20.0f    /* 前后脚最小间距，防止撞腿 */

/* 方向标定（机械结构改动后重新标定）：
   头部旋转了 180°，原来的“前进”变成了现在的“后退”，左右方向不变。
   ble_walk_test 里逻辑角减小 = 脚相对机身向前，前进用 -1；
   头部转 180° 后，新的前进方向 = 旧的后退方向，所以这里取 +1。
   如果实物上 S1 后反而往后退，把本宏改回 -1 即可（只影响前后）。

   Trot 的对角分组由实物腿位决定；改方向符号后仍需检查两组对角腿。 */
#define GAIT_FORWARD_SIGN       (+1)

/* ---- 直行标定（只在实物上微调，默认不改变原有行为）---- */

/* 每路舵机零位补偿（0.1°，逻辑角域）。
   舵机臂装偏一个齿、左右机械中位不一致时，四腿站立不竖直，
   摆角以偏心的中位为中心，直行会被推着往一侧偏。
   索引 0~3 对应 1~4 号舵机。正值让该路逻辑角整体增大（多摆向
   “逻辑角大”的一端），负值相反。先在悬空/垫起状态下把四条腿
   调竖直，再看直行是否需要纠偏。 */
#define GAIT_TRIM_1_DDEG        0
#define GAIT_TRIM_2_DDEG        0
#define GAIT_TRIM_3_DDEG        0
#define GAIT_TRIM_4_DDEG        0

/* 左侧步幅补偿（%）：PA8/PA10（1/3 号腿）增加本值，PA9/PA11 保持基准。 */
#define GAIT_LEFT_STEP_BOOST_PCT   3

/* 相位刻度：0~999 表示一个周期，全部用整数运算 */
#define GAIT_PHASE_MAX          1000U
/* 每次更新的最大相位增量；相位累加器保留余数以避免周期累计误差 */
#define GAIT_PHASE_STEP_MAX     ((uint16_t)(((GAIT_PHASE_MAX * GAIT_TICK_MS) + \
                                             GAIT_CYCLE_MS - 1U) / GAIT_CYCLE_MS))

#if (GAIT_STANCE_PCT < 1U) || (GAIT_STANCE_PCT > 99U)
#error "GAIT_STANCE_PCT must be 1..99"
#endif
#if (GAIT_STANCE_PCT <= 50U)
#warning "stance <= 50% may reduce net displacement with this single-axis gait"
#endif

/* ---- S1 按键（PB3，下拉输入，按下为高电平） ---- */
#define KEY_DEBOUNCE_MS         20U      /* 消抖时间 */

/* ---- VL53L0X（I2C1：PB8 SCL / PB9 SDA，XSHUT = PC14） ---- */
#define TOF_I2C_ADDR            (0x29U << 1)   /* HAL 使用 8 位地址 */
#define TOF_I2C_TIMEOUT_MS      50U            /* 单次 I2C 传输超时 */
#define TOF_IO_TIMEOUT_MS       500U           /* 等待传感器内部完成的超时 */
#define TOF_MEASURE_MS          100U           /* 两次测量的间隔（非阻塞） */
#define TOF_POLL_MS             5U             /* 轮询状态寄存器的间隔 */
#define TOF_REPORT_MS           500U           /* 串口打印距离的间隔 */
#define TOF_MAX_VALID_MM        2000U          /* 超过该值视为前方空旷 */
#define TOF_OBSTACLE_MM         100U           /* ≤ 该距离判定为障碍，立即停止 */
#define TOF_CLEAR_MM            130U           /* ≥ 该距离才解除障碍（回差） */
#define TOF_ERROR_STOP_COUNT    3U             /* 连续测量失败次数达到后按障碍处理 */
#define TOF_RANGE_ERROR         65535U         /* 读寄存器失败时的返回值 */

/* 寄存器地址（VL53L0X API 中的命名，仅列出本工程用到的） */
#define TOF_REG_SYSRANGE_START                      0x00U
#define TOF_REG_SYSTEM_SEQUENCE_CONFIG              0x01U
#define TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO        0x0AU
#define TOF_REG_SYSTEM_INTERRUPT_CLEAR              0x0BU
#define TOF_REG_RESULT_INTERRUPT_STATUS             0x13U
#define TOF_REG_RESULT_RANGE_STATUS                 0x14U
#define TOF_REG_MSRC_CONFIG_CONTROL                 0x60U
#define TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP          0x46U
#define TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH             0x84U
#define TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV    0x89U
#define TOF_REG_IDENTIFICATION_MODEL_ID             0xC0U
#define TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0    0xB0U
#define TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET    0x4FU
#define TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4EU
#define TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT   0xB6U
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW    0x56U
#define TOF_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH   0x57U
#define TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD       0x50U
#define TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  0x51U
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW  0x47U
#define TOF_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48U
#define TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD     0x70U
#define TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71U
#define TOF_REG_FINAL_RANGE_MIN_COUNT_RATE_RTN_LIMIT 0x44U

/* 宏周期 (ns) = 2304 × VCSEL 周期 (PCLK) × 1655 / 1000，四舍五入 */
#define TOF_CALC_MACRO_PERIOD_NS(pclks) \
  ((((uint32_t)2304U * (uint32_t)(pclks) * 1655U) + 500U) / 1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint32_t s_servo_channel[SERVO_COUNT] =
{
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* 机械安装方向：1/3 号同向，2/4 号反向安装，逻辑角度需取 180-angle */
static const uint8_t s_servo_invert[SERVO_COUNT] = { 0U, 1U, 0U, 1U };

/* 每路零位补偿（0.1°，逻辑角域），来自 PD 里的 GAIT_TRIM_x_DDEG */
static const int16_t s_servo_trim_ddeg[SERVO_COUNT] =
{
  GAIT_TRIM_1_DDEG, GAIT_TRIM_2_DDEG, GAIT_TRIM_3_DDEG, GAIT_TRIM_4_DDEG
};

/* 每路步幅比例（%）：左侧 PA8/PA10 增加补偿，右侧 PA9/PA11 为基准。 */
static const int16_t s_servo_scale_pct[SERVO_COUNT] =
{
  (int16_t)(100 + GAIT_LEFT_STEP_BOOST_PCT), 100,
  (int16_t)(100 + GAIT_LEFT_STEP_BOOST_PCT), 100
};

/*
 * Trot 对角相位偏移，索引 0~3 对应 PA8~PA11（1~4 号舵机）。
 * PA8(新左后)+PA11(新右前)同相；PA9(新右后)+PA10(新左前)同相，
 * 两组相差半周期。每腿支撑 80%、复位 20%；单轴机构不主动抬脚，
 * 因此这是对角配对的水平摆腿试验，不代表具有腾空相的动态小跑。
 */
static const uint16_t s_phase_offset[SERVO_COUNT] = { 0U, 500U, 500U, 0U };

static Motion   s_motion = MOTION_STOP;      /* 当前运动，按键/障碍时修改 */
static uint16_t s_gait_phase = 0U;           /* 步态相位 0~999 */
static uint32_t s_gait_phase_remainder = 0U; /* 相位小数余数，保持实际周期准确 */
static uint32_t s_last_gait_tick = 0U;       /* 上一次步态更新时刻 */

/* 由机械尺寸算出的步幅参数（0.1° 为单位，Gait_Config 里初始化） */
static int16_t s_amp_fwd_ddeg = 0;           /* 前进摆角 */
static int16_t s_slew_ddeg = GAIT_SLEW_DEG * 10;
                                             /* 每 tick 限速，摆动更快时自动上调 */

/* 当前实际输出的逻辑角度（0.1° 为单位，900 = 腿竖直） */
static int16_t  s_servo_logical_ddeg[SERVO_COUNT] = { 900, 900, 900, 900 };

/* S1（K1 = PB3）消抖状态 */
static uint8_t  s_key_stable = 0U;           /* 消抖后的按键状态 */
static uint8_t  s_key_raw    = 0U;           /* 上一次采样的原始状态 */
static uint32_t s_key_tick   = 0U;           /* 原始状态最后一次变化的时间 */

/* VL53L0X 驱动内部状态 */
static uint8_t  s_tof_stop_variable;         /* 从 0x91 读出、启动测量前写回 */
static uint32_t s_tof_budget_us;             /* 当前时间预算（us） */

/* VL53L0X 非阻塞测距状态机 */
static TofState s_tof_state = TOF_STATE_IDLE;
static uint32_t s_tof_state_tick = 0U;       /* 进入当前状态的时刻 */
static uint32_t s_tof_measure_tick = 0U;     /* 上一次启动测量的时刻 */
static uint32_t s_tof_poll_tick = 0U;        /* 上一次轮询寄存器的时刻 */
static uint32_t s_tof_report_tick = 0U;      /* 上一次串口打印的时刻 */
static uint8_t  s_tof_error_count = 0U;      /* 连续测量失败次数 */
static uint8_t  s_tof_valid = 0U;            /* 上电后是否已取得有效测量 */
static uint8_t  s_tof_obstacle = 0U;         /* 1 = 前方 10cm 内有障碍 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t Servo_AngleToPulse(uint8_t angle);
static void     Servo_SetLogical(uint8_t index, uint8_t logical);
static void     Uart_SendString(const char *text);
static void     Uart_SendDistance(uint16_t mm);
static void     Gait_Config(void);
static int16_t  Gait_Wave(uint16_t phase);
static void     Gait_Tick(void);
static uint8_t  Key1_Scan(void);
static void     TOF_PowerUp(void);
static uint8_t  TOF_Init(void);
static uint8_t  TOF_StartSingle(void);
static void     TOF_Task(uint32_t now);
static void     Tof_SetObstacle(uint8_t obstacle, const char *reason);
static void     Tof_HandleResult(uint16_t range, uint32_t now);
static void     Tof_HandleError(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  角度转换为比较值（TIM1 计数 1MHz，1 tick = 1us）
  */
static uint32_t Servo_AngleToPulse(uint8_t angle)
{
  return SERVO_MIN_US + ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180U;
}

/**
  * @brief  按逻辑角度设置某一路舵机（自动处理反向安装）
  * @param  index   0~3，对应 1~4 号舵机
  * @param  logical 四条腿同向运动时的统一角度 0~180
  */
static void Servo_SetLogical(uint8_t index, uint8_t logical)
{
  uint8_t physical;

  if (index >= SERVO_COUNT)
  {
    return;
  }

  if (logical > 180U)
  {
    logical = 180U;
  }

  physical = (s_servo_invert[index] != 0U) ? (uint8_t)(180U - logical) : logical;
  __HAL_TIM_SET_COMPARE(&htim1, s_servo_channel[index],
                        Servo_AngleToPulse(physical));
}

/**
  * @brief  通过 USART3 发送字符串（主循环调用，不放在中断里）
  */
static void Uart_SendString(const char *text)
{
  (void)HAL_UART_Transmit(&huart3, (const uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

/**
  * @brief  串口打印距离，格式 "TOF: 123 mm\r\n"
  */
static void Uart_SendDistance(uint16_t mm)
{
  char    buf[24];
  char    digits[5];
  uint8_t len = 0U;
  uint8_t n = 0U;

  buf[len++] = 'T'; buf[len++] = 'O'; buf[len++] = 'F'; buf[len++] = ':';
  buf[len++] = ' ';

  /* 手动把毫米数转成十进制，避免依赖 printf/浮点库 */
  do
  {
    digits[n++] = (char)('0' + (mm % 10U));
    mm = (uint16_t)(mm / 10U);
  } while ((mm > 0U) && (n < 5U));

  while (n > 0U)
  {
    buf[len++] = digits[--n];
  }

  buf[len++] = ' ';
  buf[len++] = 'm';
  buf[len++] = 'm';
  buf[len++] = '\r';
  buf[len++] = '\n';

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100U);
}

/**
  * @brief  脚的前后位移（mm）换算成腿的摆角（°）
  *         单节腿：脚的水平位移 = 2 × 腿长 × sin(摆角)
  */
static float Gait_StrokeToTiltDeg(float stroke_mm)
{
  float s = stroke_mm / (2.0f * LEG_LENGTH_MM);

  if (s > 0.999f)
  {
    s = 0.999f;
  }
  else if (s < -0.999f)
  {
    s = -0.999f;
  }

  return asinf(s) * (180.0f / 3.14159265f);
}

/**
  * @brief  摆角限幅后换算成 0.1°
  */
static int16_t Gait_TiltToDdeg(float deg)
{
  if (deg < 0.0f)
  {
    deg = 0.0f;
  }
  else if (deg > GAIT_MAX_TILT_DEG)
  {
    deg = GAIT_MAX_TILT_DEG;
  }

  return (int16_t)(deg * 10.0f + 0.5f);
}

/**
  * @brief  按机械尺寸计算步幅对应的摆角，并核算限速
  */
static void Gait_Config(void)
{
  float    stride = GAIT_STRIDE_MM;
  float    max_stride = BODY_WHEELBASE_MM - GAIT_MIN_FOOT_GAP_MM;
  int32_t  need_ddeg;
  uint16_t swing = (uint16_t)(GAIT_PHASE_MAX - (GAIT_STANCE_PCT * 10U));

  if (stride > max_stride)
  {
    stride = max_stride;
  }
  else if (stride < 0.0f)
  {
    stride = 0.0f;
  }

  s_amp_fwd_ddeg = Gait_TiltToDdeg(Gait_StrokeToTiltDeg(stride));

  /*
   * 限速必须大于摆动相的实际速度，否则快摆会被削平、占空比失效，
   * 又会退化成“原地踏步”。这里按摆角自动核算，只升不降。
   */
  need_ddeg = ((int32_t)s_amp_fwd_ddeg * 2 * GAIT_PHASE_STEP_MAX * 11) /
              (10 * (int32_t)swing);

  if (need_ddeg > (int32_t)(GAIT_SLEW_DEG * 10))
  {
    s_slew_ddeg = (int16_t)need_ddeg;
  }
}

/**
  * @brief  步态波形：输入相位 0~999，返回 -1000~+1000
  *         支撑相从 +1000 缓慢降到 -1000，摆动相快速升回 +1000。
  *         对称三角波会让前后划水的冲量抵消，这里必须用非对称波形。
  */
static int16_t Gait_Wave(uint16_t phase)
{
  const uint16_t stance = (uint16_t)(GAIT_STANCE_PCT * 10U);

  phase = (uint16_t)(phase % GAIT_PHASE_MAX);

  if (phase < stance)
  {
    return (int16_t)(1000 - ((int32_t)phase * 2000) / stance);
  }

  return (int16_t)(-1000 + ((int32_t)(phase - stance) * 2000) /
                   ((int32_t)GAIT_PHASE_MAX - (int32_t)stance));
}

/**
  * @brief  步态更新：每 GAIT_TICK_MS 调用一次
  *         前进时四条腿同向，支撑相慢速向后（按新方向标定）划。
  *         停止时冻结相位，四腿平滑回到中位。
  */
static void Gait_Tick(void)
{
  uint8_t  i;
  uint8_t  walking = (s_motion == MOTION_FORWARD) ? 1U : 0U;

  /* 停止时冻结相位，下次启动从当前位置继续；运动中相位持续前进 */
  if (walking != 0U)
  {
    uint32_t phase_numerator = s_gait_phase_remainder +
                               (GAIT_PHASE_MAX * GAIT_TICK_MS);
    uint16_t phase_advance = (uint16_t)(phase_numerator / GAIT_CYCLE_MS);

    s_gait_phase_remainder = phase_numerator % GAIT_CYCLE_MS;
    s_gait_phase = (uint16_t)((s_gait_phase + phase_advance) % GAIT_PHASE_MAX);
  }

  for (i = 0U; i < SERVO_COUNT; i++)
  {
    /* 停止时回到各自标定过的中位，保证四腿竖直 */
    int16_t target_ddeg = (int16_t)(GAIT_CENTER_DEG * 10 + s_servo_trim_ddeg[i]);

    if (walking != 0U)
    {
      uint16_t phase = (uint16_t)((s_gait_phase + s_phase_offset[i]) % GAIT_PHASE_MAX);
      int16_t  wave = Gait_Wave(phase);
      int32_t  amp_ddeg = ((int32_t)s_amp_fwd_ddeg * s_servo_scale_pct[i]) / 100;

      target_ddeg = (int16_t)(target_ddeg +
                              ((int32_t)GAIT_FORWARD_SIGN * amp_ddeg * wave) / 1000);
    }

    /* 限速：切换运动时不会从当前角度直接跳到新目标 */
    if (s_servo_logical_ddeg[i] < target_ddeg)
    {
      s_servo_logical_ddeg[i] = (int16_t)(s_servo_logical_ddeg[i] + s_slew_ddeg);
      if (s_servo_logical_ddeg[i] > target_ddeg)
      {
        s_servo_logical_ddeg[i] = target_ddeg;
      }
    }
    else if (s_servo_logical_ddeg[i] > target_ddeg)
    {
      s_servo_logical_ddeg[i] = (int16_t)(s_servo_logical_ddeg[i] - s_slew_ddeg);
      if (s_servo_logical_ddeg[i] < target_ddeg)
      {
        s_servo_logical_ddeg[i] = target_ddeg;
      }
    }

    if (s_servo_logical_ddeg[i] < 0)
    {
      s_servo_logical_ddeg[i] = 0;
    }
    else if (s_servo_logical_ddeg[i] > 1800)
    {
      s_servo_logical_ddeg[i] = 1800;
    }

    Servo_SetLogical(i, (uint8_t)((s_servo_logical_ddeg[i] + 5) / 10));
  }

  /* LED0 低电平点亮：行走中亮，停止后灭 */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                    (walking != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
  * @brief  扫描 S1（K1 = PB3，下拉输入，按下为高电平）
  * @retval 1 = 检测到一次新的按下（已消抖），0 = 无按下事件
  */
static uint8_t Key1_Scan(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  raw = (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  if (raw != s_key_raw)
  {
    /* 状态抖动中，重新计时 */
    s_key_raw  = raw;
    s_key_tick = now;
  }
  else if ((raw != s_key_stable) && ((now - s_key_tick) >= KEY_DEBOUNCE_MS))
  {
    /* 电平保持稳定超过消抖时间，确认状态切换 */
    s_key_stable = raw;
    if (s_key_stable != 0U)
    {
      return 1U;
    }
  }

  return 0U;
}

/* ==========================================================================
 * VL53L0X 驱动（移植自 Pololu VL53L0X Arduino 库，改为 HAL 阻塞式 I2C）
 * 所有函数返回 1 = 成功，0 = 失败。
 * ========================================================================== */

/* ---- 底层寄存器读写 ---- */

static uint8_t TOF_WriteMulti(uint8_t reg, const uint8_t *src, uint16_t len)
{
  /* HAL 的 pData 不是 const，这里强制转换；不会修改内容 */
  return (HAL_I2C_Mem_Write(&hi2c1, TOF_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)src, len, TOF_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t TOF_ReadMulti(uint8_t reg, uint8_t *dst, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&hi2c1, TOF_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           dst, len, TOF_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t TOF_WriteReg(uint8_t reg, uint8_t value)
{
  return TOF_WriteMulti(reg, &value, 1U);
}

static uint8_t TOF_ReadReg(uint8_t reg, uint8_t *value)
{
  return TOF_ReadMulti(reg, value, 1U);
}

static uint8_t TOF_WriteReg16(uint8_t reg, uint16_t value)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(value >> 8);
  buf[1] = (uint8_t)(value & 0xFFU);
  return TOF_WriteMulti(reg, buf, 2U);
}

static uint8_t TOF_ReadReg16(uint8_t reg, uint16_t *value)
{
  uint8_t buf[2];

  if (!TOF_ReadMulti(reg, buf, 2U))
  {
    return 0U;
  }

  *value = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  return 1U;
}

/* ---- 超时值编解码（寄存器格式：LSByte × 2^MSByte + 1） ---- */

static uint16_t TOF_DecodeTimeout(uint16_t reg_val)
{
  return (uint16_t)(((reg_val & 0x00FFU) << ((reg_val & 0xFF00U) >> 8)) + 1U);
}

static uint16_t TOF_EncodeTimeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte;
  uint16_t ms_byte = 0U;

  if (timeout_mclks == 0U)
  {
    return 0U;
  }

  ls_byte = (uint32_t)timeout_mclks - 1U;
  while ((ls_byte & 0xFFFFFF00U) > 0U)
  {
    ls_byte >>= 1;
    ms_byte++;
  }

  return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFFU));
}

static uint32_t TOF_MclksToUs(uint16_t mclks, uint8_t pclks)
{
  uint32_t macro_ns;

  if (pclks == 0U)
  {
    return 0U;                         /* 读周期失败，避免除零 */
  }

  macro_ns = TOF_CALC_MACRO_PERIOD_NS(pclks);
  return (((uint32_t)mclks * macro_ns) + (macro_ns / 2U)) / 1000U;
}

static uint32_t TOF_UsToMclks(uint32_t us, uint8_t pclks)
{
  uint32_t macro_ns;

  if (pclks == 0U)
  {
    return 0U;
  }

  macro_ns = TOF_CALC_MACRO_PERIOD_NS(pclks);
  return ((us * 1000U) + (macro_ns / 2U)) / macro_ns;
}

/* ---- 序列步骤信息 ---- */

static uint8_t TOF_GetVcselPeriod(uint8_t final_range)
{
  uint8_t reg_val;

  if (!TOF_ReadReg(final_range ? TOF_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD
                               : TOF_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &reg_val))
  {
    return 0U;
  }

  return (uint8_t)((reg_val + 1U) << 1);   /* 寄存器值解码为 PCLK 周期 */
}

static void TOF_GetSequenceStepEnables(TOF_SeqEnables *enables)
{
  uint8_t seq = 0U;

  (void)TOF_ReadReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, &seq);

  enables->tcc         = (uint8_t)((seq >> 4) & 0x01U);
  enables->dss         = (uint8_t)((seq >> 3) & 0x01U);
  enables->msrc        = (uint8_t)((seq >> 2) & 0x01U);
  enables->pre_range   = (uint8_t)((seq >> 6) & 0x01U);
  enables->final_range = (uint8_t)((seq >> 7) & 0x01U);
}

static void TOF_GetSequenceStepTimeouts(const TOF_SeqEnables *enables,
                                        TOF_SeqTimeouts *timeouts)
{
  uint8_t  reg8 = 0U;
  uint16_t reg16 = 0U;

  timeouts->pre_range_vcsel_period_pclks = TOF_GetVcselPeriod(0U);

  (void)TOF_ReadReg(TOF_REG_MSRC_CONFIG_TIMEOUT_MACROP, &reg8);
  timeouts->msrc_dss_tcc_mclks = (uint16_t)reg8 + 1U;
  timeouts->msrc_dss_tcc_us =
    TOF_MclksToUs(timeouts->msrc_dss_tcc_mclks,
                  timeouts->pre_range_vcsel_period_pclks);

  (void)TOF_ReadReg16(TOF_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg16);
  timeouts->pre_range_mclks = TOF_DecodeTimeout(reg16);
  timeouts->pre_range_us =
    TOF_MclksToUs(timeouts->pre_range_mclks,
                  timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = TOF_GetVcselPeriod(1U);

  (void)TOF_ReadReg16(TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg16);
  timeouts->final_range_mclks = TOF_DecodeTimeout(reg16);

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    TOF_MclksToUs(timeouts->final_range_mclks,
                  timeouts->final_range_vcsel_period_pclks);
}

/* ---- 信号速率与时间预算 ---- */

static uint8_t TOF_SetSignalRateLimit(float limit_mcps)
{
  /* 单位是 MCPS，寄存器值 = MCPS × 2^7 */
  if (limit_mcps < 0.0f)
  {
    limit_mcps = 0.0f;
  }
  if (limit_mcps > 511.99f)
  {
    limit_mcps = 511.99f;
  }

  return TOF_WriteReg16(TOF_REG_FINAL_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
                        (uint16_t)(limit_mcps * 128.0f));
}

static uint32_t TOF_GetTimingBudget(void)
{
  const uint16_t start_overhead      = 1910U;
  const uint16_t end_overhead        = 960U;
  const uint16_t msrc_overhead       = 660U;
  const uint16_t tcc_overhead        = 590U;
  const uint16_t dss_overhead        = 690U;
  const uint16_t pre_range_overhead  = 660U;
  const uint16_t final_range_overhead = 550U;

  TOF_SeqEnables  enables;
  TOF_SeqTimeouts timeouts;
  uint32_t budget_us = (uint32_t)start_overhead + end_overhead;

  TOF_GetSequenceStepEnables(&enables);
  TOF_GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += timeouts.msrc_dss_tcc_us + tcc_overhead;
  }

  if (enables.dss)
  {
    budget_us += 2U * (timeouts.msrc_dss_tcc_us + dss_overhead);
  }
  else if (enables.msrc)
  {
    budget_us += timeouts.msrc_dss_tcc_us + msrc_overhead;
  }

  if (enables.pre_range)
  {
    budget_us += timeouts.pre_range_us + pre_range_overhead;
  }

  if (enables.final_range)
  {
    budget_us += timeouts.final_range_us + final_range_overhead;
  }

  s_tof_budget_us = budget_us;
  return budget_us;
}

static uint8_t TOF_SetTimingBudget(uint32_t budget_us)
{
  const uint16_t start_overhead       = 1320U;
  const uint16_t end_overhead         = 960U;
  const uint16_t msrc_overhead        = 660U;
  const uint16_t tcc_overhead         = 590U;
  const uint16_t dss_overhead         = 690U;
  const uint16_t pre_range_overhead   = 660U;
  const uint16_t final_range_overhead = 550U;
  const uint32_t min_timing_budget    = 20000U;

  TOF_SeqEnables  enables;
  TOF_SeqTimeouts timeouts;
  uint32_t used_budget_us;
  uint32_t final_range_timeout_us;
  uint16_t final_range_timeout_mclks;

  if (budget_us < min_timing_budget)
  {
    return 0U;
  }

  used_budget_us = (uint32_t)start_overhead + end_overhead;

  TOF_GetSequenceStepEnables(&enables);
  TOF_GetSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += timeouts.msrc_dss_tcc_us + tcc_overhead;
  }

  if (enables.dss)
  {
    used_budget_us += 2U * (timeouts.msrc_dss_tcc_us + dss_overhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += timeouts.msrc_dss_tcc_us + msrc_overhead;
  }

  if (enables.pre_range)
  {
    used_budget_us += timeouts.pre_range_us + pre_range_overhead;
  }

  if (enables.final_range)
  {
    used_budget_us += final_range_overhead;

    if (used_budget_us > budget_us)
    {
      return 0U;                     /* 预算太小，放不下其它步骤 */
    }

    final_range_timeout_us = budget_us - used_budget_us;
    final_range_timeout_mclks =
      (uint16_t)TOF_UsToMclks(final_range_timeout_us,
                              timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    if (!TOF_WriteReg16(TOF_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                        TOF_EncodeTimeout(final_range_timeout_mclks)))
    {
      return 0U;
    }

    s_tof_budget_us = budget_us;
  }

  return 1U;
}

/* ---- 参考校准 ---- */

static uint8_t TOF_PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
  uint8_t  status = 0U;
  uint32_t start;

  if (!TOF_WriteReg(TOF_REG_SYSRANGE_START, (uint8_t)(0x01U | vhv_init_byte)))
  {
    return 0U;
  }

  start = HAL_GetTick();
  do
  {
    if (!TOF_ReadReg(TOF_REG_RESULT_INTERRUPT_STATUS, &status))
    {
      return 0U;
    }
    if ((status & 0x07U) != 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - start) < TOF_IO_TIMEOUT_MS);

  if ((status & 0x07U) == 0U)
  {
    return 0U;                       /* 等待校准完成超时 */
  }

  (void)TOF_WriteReg(TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
  (void)TOF_WriteReg(TOF_REG_SYSRANGE_START, 0x00U);

  return 1U;
}

/* ---- SPAD 信息（每一片的参考 SPAD 数量/类型不同，必须从芯片读取） ---- */

static uint8_t TOF_GetSpadInfo(uint8_t *count, uint8_t *type_is_aperture)
{
  uint8_t  tmp = 0U;
  uint8_t  reg83 = 0U;
  uint32_t start;

  if (!TOF_WriteReg(0x80U, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x01U) ||
      !TOF_WriteReg(0x00U, 0x00U) ||
      !TOF_WriteReg(0xFFU, 0x06U))
  {
    return 0U;
  }

  if (!TOF_ReadReg(0x83U, &reg83) ||
      !TOF_WriteReg(0x83U, (uint8_t)(reg83 | 0x04U)) ||
      !TOF_WriteReg(0xFFU, 0x07U) ||
      !TOF_WriteReg(0x81U, 0x01U) ||
      !TOF_WriteReg(0x80U, 0x01U) ||
      !TOF_WriteReg(0x94U, 0x6BU) ||
      !TOF_WriteReg(0x83U, 0x00U))
  {
    return 0U;
  }

  start = HAL_GetTick();
  do
  {
    if (!TOF_ReadReg(0x83U, &tmp))
    {
      return 0U;
    }
    if (tmp != 0x00U)
    {
      break;
    }
  } while ((HAL_GetTick() - start) < TOF_IO_TIMEOUT_MS);

  if (tmp == 0x00U)
  {
    return 0U;                       /* 芯片没有返回 SPAD 信息 */
  }

  if (!TOF_WriteReg(0x83U, 0x01U) ||
      !TOF_ReadReg(0x92U, &tmp))
  {
    return 0U;
  }

  *count = (uint8_t)(tmp & 0x7FU);
  *type_is_aperture = (uint8_t)((tmp >> 7) & 0x01U);

  (void)TOF_WriteReg(0x81U, 0x00U);
  (void)TOF_WriteReg(0xFFU, 0x06U);
  if (TOF_ReadReg(0x83U, &reg83))
  {
    (void)TOF_WriteReg(0x83U, (uint8_t)(reg83 & (uint8_t)~0x04U));
  }
  (void)TOF_WriteReg(0xFFU, 0x01U);
  (void)TOF_WriteReg(0x00U, 0x01U);
  (void)TOF_WriteReg(0xFFU, 0x00U);
  (void)TOF_WriteReg(0x80U, 0x00U);

  return 1U;
}

/* ---- 上电与初始化 ---- */

static void TOF_PowerUp(void)
{
  /* 复位 I2C 外设，清掉上一次初始化失败可能留下的忙/错误状态 */
  (void)HAL_I2C_DeInit(&hi2c1);
  (void)HAL_I2C_Init(&hi2c1);

  /* XSHUT 低电平使模块进入关断，拉高后需要一点启动时间 */
  HAL_GPIO_WritePin(XSHUT_TOF_GPIO_Port, XSHUT_TOF_Pin, GPIO_PIN_RESET);
  HAL_Delay(2U);
  HAL_GPIO_WritePin(XSHUT_TOF_GPIO_Port, XSHUT_TOF_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);
}

static uint8_t TOF_Init(void)
{
  uint8_t  model_id = 0U;
  uint8_t  spad_count = 0U;
  uint8_t  spad_type_is_aperture = 0U;
  uint8_t  ref_spad_map[6];
  uint8_t  reg_val = 0U;
  uint8_t  first_spad_to_enable;
  uint8_t  spads_enabled = 0U;
  uint8_t  i;

  /* 先确认地址 0x29 上有 VL53L0X（型号 ID 固定为 0xEE） */
  if (!TOF_ReadReg(TOF_REG_IDENTIFICATION_MODEL_ID, &model_id) || (model_id != 0xEEU))
  {
    return 0U;
  }

  /* ---- VL53L0X_DataInit() ---- */

  /* 传感器 I/O 默认 1.8V，置位后使用 2.8V 模式 */
  if (!TOF_ReadReg(TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &reg_val) ||
      !TOF_WriteReg(TOF_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                    (uint8_t)(reg_val | 0x01U)))
  {
    return 0U;
  }

  /* I2C 标准模式 */
  if (!TOF_WriteReg(0x88U, 0x00U) ||
      !TOF_WriteReg(0x80U, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x01U) ||
      !TOF_WriteReg(0x00U, 0x00U) ||
      !TOF_ReadReg(0x91U, &s_tof_stop_variable) ||
      !TOF_WriteReg(0x00U, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x00U) ||
      !TOF_WriteReg(0x80U, 0x00U))
  {
    return 0U;
  }

  /* 关闭 MSRC / PRE_RANGE 的信号速率限制检查 */
  if (!TOF_ReadReg(TOF_REG_MSRC_CONFIG_CONTROL, &reg_val) ||
      !TOF_WriteReg(TOF_REG_MSRC_CONFIG_CONTROL, (uint8_t)(reg_val | 0x12U)))
  {
    return 0U;
  }

  /* 最终距离信号速率门限 0.25 MCPS */
  if (!TOF_SetSignalRateLimit(0.25f) ||
      !TOF_WriteReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0xFFU))
  {
    return 0U;
  }

  /* ---- VL53L0X_StaticInit() ---- */

  if (!TOF_GetSpadInfo(&spad_count, &spad_type_is_aperture) ||
      !TOF_ReadMulti(TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6U))
  {
    return 0U;
  }

  /* ---- VL53L0X_set_reference_spads() ---- */
  if (!TOF_WriteReg(0xFFU, 0x01U) ||
      !TOF_WriteReg(TOF_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00U) ||
      !TOF_WriteReg(TOF_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2CU) ||
      !TOF_WriteReg(0xFFU, 0x00U) ||
      !TOF_WriteReg(TOF_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4U))
  {
    return 0U;
  }

  first_spad_to_enable = (spad_type_is_aperture != 0U) ? 12U : 0U;

  for (i = 0U; i < 48U; i++)
  {
    if ((i < first_spad_to_enable) || (spads_enabled == spad_count))
    {
      /* 低于起始编号、或已达到所需数量：清零该位 */
      ref_spad_map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
    }
    else if (((ref_spad_map[i / 8U] >> (i % 8U)) & 0x01U) != 0U)
    {
      spads_enabled++;
    }
  }

  if (!TOF_WriteMulti(TOF_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6U))
  {
    return 0U;
  }

  /* ---- VL53L0X_load_tuning_settings()：官方默认调参表 ---- */
  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x00U, 0x00U) ||
      !TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x09U, 0x00U) ||
      !TOF_WriteReg(0x10U, 0x00U) || !TOF_WriteReg(0x11U, 0x00U) ||
      !TOF_WriteReg(0x24U, 0x01U) || !TOF_WriteReg(0x25U, 0xFFU) ||
      !TOF_WriteReg(0x75U, 0x00U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x4EU, 0x2CU) ||
      !TOF_WriteReg(0x48U, 0x00U) || !TOF_WriteReg(0x30U, 0x20U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x30U, 0x09U) ||
      !TOF_WriteReg(0x54U, 0x00U) || !TOF_WriteReg(0x31U, 0x04U) ||
      !TOF_WriteReg(0x32U, 0x03U) || !TOF_WriteReg(0x40U, 0x83U) ||
      !TOF_WriteReg(0x46U, 0x25U) || !TOF_WriteReg(0x60U, 0x00U) ||
      !TOF_WriteReg(0x27U, 0x00U) || !TOF_WriteReg(0x50U, 0x06U) ||
      !TOF_WriteReg(0x51U, 0x00U) || !TOF_WriteReg(0x52U, 0x96U) ||
      !TOF_WriteReg(0x56U, 0x08U) || !TOF_WriteReg(0x57U, 0x30U) ||
      !TOF_WriteReg(0x61U, 0x00U) || !TOF_WriteReg(0x62U, 0x00U) ||
      !TOF_WriteReg(0x64U, 0x00U) || !TOF_WriteReg(0x65U, 0x00U) ||
      !TOF_WriteReg(0x66U, 0xA0U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x22U, 0x32U) ||
      !TOF_WriteReg(0x47U, 0x14U) || !TOF_WriteReg(0x49U, 0xFFU) ||
      !TOF_WriteReg(0x4AU, 0x00U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x7AU, 0x0AU) ||
      !TOF_WriteReg(0x7BU, 0x00U) || !TOF_WriteReg(0x78U, 0x21U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x23U, 0x34U) ||
      !TOF_WriteReg(0x42U, 0x00U) || !TOF_WriteReg(0x44U, 0xFFU) ||
      !TOF_WriteReg(0x45U, 0x26U) || !TOF_WriteReg(0x46U, 0x05U) ||
      !TOF_WriteReg(0x40U, 0x40U) || !TOF_WriteReg(0x0EU, 0x06U) ||
      !TOF_WriteReg(0x20U, 0x1AU) || !TOF_WriteReg(0x43U, 0x40U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x34U, 0x03U) ||
      !TOF_WriteReg(0x35U, 0x44U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x31U, 0x04U) ||
      !TOF_WriteReg(0x4BU, 0x09U) || !TOF_WriteReg(0x4CU, 0x05U) ||
      !TOF_WriteReg(0x4DU, 0x04U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x44U, 0x00U) ||
      !TOF_WriteReg(0x45U, 0x20U) || !TOF_WriteReg(0x47U, 0x08U) ||
      !TOF_WriteReg(0x48U, 0x28U) || !TOF_WriteReg(0x67U, 0x00U) ||
      !TOF_WriteReg(0x70U, 0x04U) || !TOF_WriteReg(0x71U, 0x01U) ||
      !TOF_WriteReg(0x72U, 0xFEU) || !TOF_WriteReg(0x76U, 0x00U) ||
      !TOF_WriteReg(0x77U, 0x00U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x0DU, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x00U) || !TOF_WriteReg(0x80U, 0x01U) ||
      !TOF_WriteReg(0x01U, 0xF8U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(0xFFU, 0x01U) || !TOF_WriteReg(0x8EU, 0x01U) ||
      !TOF_WriteReg(0x00U, 0x01U) || !TOF_WriteReg(0xFFU, 0x00U) ||
      !TOF_WriteReg(0x80U, 0x00U))
  {
    return 0U;
  }

  /* 中断输出配置为“新样本就绪”，低电平有效（本工程做轮询，仅按官方流程配置） */
  if (!TOF_WriteReg(TOF_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04U) ||
      !TOF_ReadReg(TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH, &reg_val) ||
      !TOF_WriteReg(TOF_REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(reg_val & (uint8_t)~0x10U)) ||
      !TOF_WriteReg(TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U))
  {
    return 0U;
  }

  /* 读取默认时间预算（约 33ms），再关闭 MSRC/TCC 后按原预算重新计算 */
  (void)TOF_GetTimingBudget();

  if (!TOF_WriteReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U) ||
      !TOF_SetTimingBudget(s_tof_budget_us))
  {
    return 0U;
  }

  /* ---- 参考校准：VHV（0x40）与相位（0x00） ---- */

  if (!TOF_WriteReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0x01U) ||
      !TOF_PerformSingleRefCalibration(0x40U))
  {
    return 0U;
  }

  if (!TOF_WriteReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0x02U) ||
      !TOF_PerformSingleRefCalibration(0x00U))
  {
    return 0U;
  }

  return TOF_WriteReg(TOF_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U);
}

/**
  * @brief  启动一次单次测量（不等待结果，供状态机调用）
  * @retval 1 = 已启动，0 = I2C 通信失败
  */
static uint8_t TOF_StartSingle(void)
{
  /* 启动前先写回停止变量（官方流程） */
  if (!TOF_WriteReg(0x80U, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x01U) ||
      !TOF_WriteReg(0x00U, 0x00U) ||
      !TOF_WriteReg(0x91U, s_tof_stop_variable) ||
      !TOF_WriteReg(0x00U, 0x01U) ||
      !TOF_WriteReg(0xFFU, 0x00U) ||
      !TOF_WriteReg(0x80U, 0x00U) ||
      !TOF_WriteReg(TOF_REG_SYSRANGE_START, 0x01U))
  {
    return 0U;
  }

  return 1U;
}

/**
  * @brief  设置/解除障碍标志，并按需要停止机械狗
  * @param  obstacle 1 = 有障碍，0 = 无障碍
  * @param  reason   打印到串口的原因
  */
static void Tof_SetObstacle(uint8_t obstacle, const char *reason)
{
  if (obstacle != 0U)
  {
    if (s_tof_obstacle == 0U)
    {
      s_tof_obstacle = 1U;

      if (s_motion != MOTION_STOP)
      {
        s_motion = MOTION_STOP;
        Uart_SendString("STOP: ");
      }
      else
      {
        Uart_SendString("BLOCKED: ");
      }
      Uart_SendString(reason);
      Uart_SendString("\r\n");
    }
  }
  else if (s_tof_obstacle != 0U)
  {
    s_tof_obstacle = 0U;
    Uart_SendString("CLEAR: press S1 to walk\r\n");
  }
}

/**
  * @brief  处理一次有效距离：更新障碍标志，并按需打印距离
  */
static void Tof_HandleResult(uint16_t range, uint32_t now)
{
  s_tof_valid = 1U;
  s_tof_error_count = 0U;

  if (range > TOF_MAX_VALID_MM)
  {
    /* 超出量程 = 前方空旷 */
    Tof_SetObstacle(0U, "clear");
    if ((now - s_tof_report_tick) >= TOF_REPORT_MS)
    {
      s_tof_report_tick = now;
      Uart_SendString("TOF: out of range\r\n");
    }
  }
  else
  {
    if (range <= TOF_OBSTACLE_MM)
    {
      Tof_SetObstacle(1U, "obstacle within 10cm");
    }
    else if (range >= TOF_CLEAR_MM)
    {
      Tof_SetObstacle(0U, "clear");
    }
    /* 10~13cm 之间保持原状态，避免临界距离反复启停 */

    if ((now - s_tof_report_tick) >= TOF_REPORT_MS)
    {
      s_tof_report_tick = now;
      Uart_SendDistance(range);
    }
  }
}

/**
  * @brief  一次测量失败：累计错误，连续失败达到阈值按障碍处理（失效安全）
  */
static void Tof_HandleError(uint32_t now)
{
  s_tof_state = TOF_STATE_IDLE;
  s_tof_measure_tick = now;

  if (s_tof_error_count < 255U)
  {
    s_tof_error_count++;
  }

  if (s_tof_error_count >= TOF_ERROR_STOP_COUNT)
  {
    Tof_SetObstacle(1U, "TOF measurement error");
  }
}

/**
  * @brief  VL53L0X 非阻塞轮询：每 TOF_POLL_MS 推进一次状态机
  * @note   单次测量约 33ms，放在主循环里做状态机，不能阻塞步态更新。
  */
static void TOF_Task(uint32_t now)
{
  uint8_t status = 0U;

  if ((now - s_tof_poll_tick) < TOF_POLL_MS)
  {
    return;
  }
  s_tof_poll_tick = now;

  switch (s_tof_state)
  {
    case TOF_STATE_IDLE:
      if ((now - s_tof_measure_tick) >= TOF_MEASURE_MS)
      {
        if (TOF_StartSingle())
        {
          s_tof_state = TOF_STATE_WAIT_START;
          s_tof_state_tick = now;
        }
        else
        {
          Tof_HandleError(now);
        }
      }
      break;

    case TOF_STATE_WAIT_START:
      if (!TOF_ReadReg(TOF_REG_SYSRANGE_START, &status))
      {
        Tof_HandleError(now);
      }
      else if ((status & 0x01U) == 0U)
      {
        /* 启动位自动清零，说明测量已经开始 */
        s_tof_state = TOF_STATE_WAIT_READY;
        s_tof_state_tick = now;
      }
      else if ((now - s_tof_state_tick) >= TOF_IO_TIMEOUT_MS)
      {
        Tof_HandleError(now);
      }
      else
      {
        /* 继续等待 */
      }
      break;

    case TOF_STATE_WAIT_READY:
    default:
      if (!TOF_ReadReg(TOF_REG_RESULT_INTERRUPT_STATUS, &status))
      {
        Tof_HandleError(now);
      }
      else if ((status & 0x07U) != 0U)
      {
        uint16_t range = TOF_RANGE_ERROR;

        /* 距离结果在 RESULT_RANGE_STATUS + 10 处，高字节在前 */
        if (TOF_ReadReg16((uint8_t)(TOF_REG_RESULT_RANGE_STATUS + 10U), &range))
        {
          (void)TOF_WriteReg(TOF_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
          s_tof_state = TOF_STATE_IDLE;
          s_tof_measure_tick = now;
          Tof_HandleResult(range, now);
        }
        else
        {
          Tof_HandleError(now);
        }
      }
      else if ((now - s_tof_state_tick) >= TOF_IO_TIMEOUT_MS)
      {
        Tof_HandleError(now);
      }
      else
      {
        /* 测量进行中 */
      }
      break;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  {
    uint8_t i;

    /* 模板把所有 LED 引脚拉低（低电平点亮 = 全亮），这里先全部熄灭 */
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);

    /* 清空 S1 的初始状态，避免上电瞬间误触发 */
    s_key_stable = (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    s_key_raw    = s_key_stable;
    s_key_tick   = HAL_GetTick();

    /* 按机械尺寸预先算好前进摆角 */
    Gait_Config();

    /* 启动 4 路 PWM，上电先站立在（带零位补偿的）中位 */
    for (i = 0U; i < SERVO_COUNT; i++)
    {
      (void)HAL_TIM_PWM_Start(&htim1, s_servo_channel[i]);
      s_servo_logical_ddeg[i] = (int16_t)(GAIT_CENTER_DEG * 10 +
                                          s_servo_trim_ddeg[i]);
      Servo_SetLogical(i, (uint8_t)((s_servo_logical_ddeg[i] + 5) / 10));
    }
  }

  s_last_gait_tick = HAL_GetTick();

  Uart_SendString("\r\n[SWITCH WALK] S1 = walk forward, stop within 10cm\r\n");

  /* PC14 拉高给 TOF 模块上电，初始化失败时每秒重试（便于热插拔排查） */
  TOF_PowerUp();
  while (!TOF_Init())
  {
    Uart_SendString("VL53L0X init failed, retrying...\r\n");
    HAL_Delay(1000U);
    TOF_PowerUp();
  }
  Uart_SendString("VL53L0X ready.\r\n");

  /* 立即启动第一次测量；首次有效距离出来之前不允许 S1 启动 */
  s_tof_measure_tick = HAL_GetTick() - TOF_MEASURE_MS;
  s_tof_poll_tick    = HAL_GetTick();
  s_tof_report_tick  = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* S1：按一次开始前进；有障碍或 TOF 未就绪时拒绝启动 */
    if (Key1_Scan() != 0U)
    {
      if (s_tof_valid == 0U)
      {
        Uart_SendString("WAIT: TOF not ready\r\n");
      }
      else if (s_tof_obstacle != 0U)
      {
        Uart_SendString("BLOCKED: obstacle within 10cm\r\n");
      }
      else if (s_motion == MOTION_STOP)
      {
        s_motion = MOTION_FORWARD;
        Uart_SendString("S1: walk forward\r\n");
      }
      else
      {
        /* 正在前进，重复按 S1 不打断步态 */
      }
    }

    /* VL53L0X 非阻塞测距；检测到障碍会立即把 s_motion 置为 STOP */
    TOF_Task(now);

    /* 步态按固定周期更新，不用 HAL_Delay，按键/障碍随时可以打断 */
    if ((now - s_last_gait_tick) >= GAIT_TICK_MS)
    {
      s_last_gait_tick = now;
      Gait_Tick();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
