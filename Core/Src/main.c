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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { MOTOR_L = 0, MOTOR_R = 1 } Motor_t;

/* One HC020K encoder on a TIM2 input-capture channel, both edges */
typedef struct {
  volatile uint32_t t1, t2, t3;  /* absolute times of the last 3 edges [10 us] */
  volatile uint8_t  n;           /* valid edges stored (0..3)                  */
  volatile int32_t  count;       /* signed edge count (odometry)               */
  volatile int8_t   dir;         /* last commanded direction: +1 / -1          */
} Encoder_t;

typedef struct { float integ; } SpeedPI_t;

typedef enum {
  ST_IDLE = 0,
  ST_STRAIGHT,
  ST_PAUSE_AFTER_STRAIGHT,
  ST_TURN,
  ST_PAUSE_AFTER_TURN,
  ST_DONE
} State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- Polygon defaults (the variables below can be edited) ---------- */
#define DEFAULT_SIDE_CM      100.0f   /* side length [cm]                   */
#define DEFAULT_SIDES        4       /* number of sides (>= 3)             */
#define DEFAULT_LAPS         1       /* number of laps (>= 1)              */

/* ---------- Mechanics ---------- */
#define WHEEL_DIAM_CM        6.2f
#define ENC_SLOTS            20.0f
#define CM_PER_SLOT          (3.14159265f * WHEEL_DIAM_CM / ENC_SLOTS) /* ~0.974 cm */
#define CM_PER_EDGE          (CM_PER_SLOT * 0.5f)                      /* ~0.487 cm */
#define TRACK_WIDTH_CM       23.0f   /* MEASURE IT: center to center of the wheels */
#define RAD2DEG              57.2957795f

/* ---------- TIM2: 10 us ticks, overflow every 10 ms ---------- */
#define ENC_TICK_HZ          100000.0f
#define ENC_TICKS_PER_OVF    1000u
#define LOOP_DT              0.01f
#define ENC_TIMEOUT_TICKS    30000u  /* 300 ms without edges -> speed 0    */
#define ENC_MIN_EDGE_TICKS   100u    /* < 1 ms between edges = glitch      */
#define ENC_IC_FILTER        15      /* max hardware input filter          */

/* ---------- Motors (TB6612FNG) ---------- */
#define PWM_MAX              3599
#define MOTOR_L_INVERT       0
#define MOTOR_R_INVERT       0

/* ---------- Trajectory ---------- */
#define START_DELAY_MS       2000
#define TURN_DIR             1.0f    /* +1 = turn left (CCW), -1 = right   */
#define V_CRUISE             30.0f   /* cm/s                               */
#define V_MIN                12.0f
#define ACCEL                30.0f   /* cm/s^2                             */
#define SLOW_DIST            10.0f   /* cm                                 */
#define PAUSE_S              0.4f

/* ---------- Wheel speed PI ---------- */
#define PWM_DEADZONE         1500.0f  /* PWM to start moving straight       */
#define TURN_PWM_DEADZONE    1200.0f /* turning in place needs more        */
#define KFF                  60.0f
#define KP_V                 40.0f
#define KI_V                 150.0f
#define I_MAX                10.0f

/* ---------- Heading hold on straights (odometry) ---------- */
#define KH                   1.0f    /* cm/s per degree                    */
#define H_CORR_MAX           6.0f

/* ---------- In-place turn (odometry) ---------- */
#define TURN_K               0.25f    /* cm/s per degree remaining          */
#define V_TURN_MAX           5.0f
#define V_TURN_MIN           2.0f
#define TURN_STOP_DEG        10.0f    /* stop this early to absorb coasting */
#define TURN_TIMEOUT_S       5.0f

/* ---------- Diagnostics ---------- */
#define DIAG_MOTOR_TEST      0             /* 1 = spin both wheels 1 s at power-on */
#define DIAG_LED_PORT        GPIOC
#define DIAG_LED_PIN         GPIO_PIN_13   /* Blue Pill LED, active low */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile float   side_length_cm = DEFAULT_SIDE_CM;
volatile uint8_t num_sides      = DEFAULT_SIDES;
volatile uint8_t num_laps       = DEFAULT_LAPS;

volatile uint8_t diag_stage = 0;

static Encoder_t enc_l = { .dir = 1 };   /* TIM2_CH1 (PA15) */
static Encoder_t enc_r = { .dir = 1 };   /* TIM2_CH2 (PB3)  */
static volatile uint32_t tim2_ovf = 0;   /* TIM2 overflows since start */
static volatile uint32_t loop_ticks = 0;

/* Globals so they can be watched live in the debugger */
float dist_l = 0.0f, dist_r = 0.0f;      /* signed wheel travel [cm] */
float heading_deg = 0.0f;                /* odometry heading, CCW + */
float speed_l = 0.0f, speed_r = 0.0f;    /* wheel speed magnitude [cm/s] */
float heading_target = 0.0f;
State_t state = ST_IDLE;

static SpeedPI_t pi_l, pi_r;
static float     state_time = 0.0f;
static float     v_ramp = 0.0f;
static float     side_len = DEFAULT_SIDE_CM;
static float     ext_angle = 90.0f;
static uint16_t  total_sides = 0;
static uint16_t  side_idx = 0;
static float     side_start_l = 0.0f, side_start_r = 0.0f;
static float     turn_sign = 1.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void  motor_set(Motor_t m, int32_t pwm);
static void  motors_brake(void);
static float encoder_speed(Encoder_t *e);
static int32_t speed_pi(SpeedPI_t *c, float target, float meas, float dt, float deadzone);
static void  update_odometry(void);
static void  enter_state(State_t s);
static void  start_side(void);
static void  control_step(float dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline float absf(float x) { return (x < 0.0f) ? -x : x; }

static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* ===================== Time base (TIM2) ===================== */

/* Absolute time [10 us ticks] of a TIM2 count value.
   Must be called from the TIM2 ISR or with interrupts disabled.
   If the counter already wrapped but the update IRQ is still pending,
   that overflow is added here. */
static inline uint32_t tim2_abs(uint32_t cnt)
{
  uint32_t ovf = tim2_ovf;
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) && cnt < (ENC_TICKS_PER_OVF / 2u)) ovf++;
  return ovf * ENC_TICKS_PER_OVF + cnt;
}

/* ===================== Motors (TB6612FNG) ===================== */

/* pwm in [-PWM_MAX, PWM_MAX]; positive = forward, 0 = coast */
static void motor_set(Motor_t m, int32_t pwm)
{
  if (pwm > PWM_MAX)  pwm = PWM_MAX;
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;

  /* remember the logical direction so encoder edges get the right sign */
  Encoder_t *e = (m == MOTOR_L) ? &enc_l : &enc_r;
  if (pwm > 0) e->dir = 1;
  else if (pwm < 0) e->dir = -1;

  if ((m == MOTOR_L && MOTOR_L_INVERT) || (m == MOTOR_R && MOTOR_R_INVERT)) pwm = -pwm;

  GPIO_PinState in1 = (pwm > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState in2 = (pwm < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  uint32_t duty = (pwm >= 0) ? (uint32_t)pwm : (uint32_t)(-pwm);

  if (m == MOTOR_L)
  {
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, in1);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
  }
  else
  {
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, in1);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
  }
}

/* IN1 = IN2 = H -> short brake. Direction is kept: while the wheel
   coasts to a stop, its edges are still counted the right way. */
static void motors_brake(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

/* ===================== Encoders ===================== */

/* Called from the TIM2 capture ISR on every edge (rising or falling) */
static void enc_edge(Encoder_t *e, uint32_t cap, uint32_t ccer_pol_bit)
{
  uint32_t t = tim2_abs(cap);

  /* Too close to the previous edge: comparator chatter. Ignore it and
     keep waiting for the same polarity. */
  if (e->n >= 1 && (t - e->t1) < ENC_MIN_EDGE_TICKS) return;

  e->t3 = e->t2;
  e->t2 = e->t1;
  e->t1 = t;
  if (e->n < 3) e->n++;
  e->count += e->dir;

  /* F1 timers cannot capture both edges: flip the polarity by hand */
  htim2.Instance->CCER ^= ccer_pol_bit;
}

/* Wheel speed magnitude [cm/s]. Uses a full slot period (3 edges)
   so the uneven duty cycle of the HC020K does not matter. */
static float encoder_speed(Encoder_t *e)
{
  uint32_t t1, t3, now;
  uint8_t  n;

  __disable_irq();
  now = tim2_abs(__HAL_TIM_GET_COUNTER(&htim2));
  n  = e->n;
  t1 = e->t1;
  t3 = e->t3;
  if (n > 0 && (now - t1) > ENC_TIMEOUT_TICKS) { e->n = 0; n = 0; }
  __enable_irq();

  if (n < 3) return 0.0f;

  uint32_t period = t1 - t3;
  uint32_t est = 2u * (now - t1);      /* wheel slowing down: decay estimate */
  if (est > period) period = est;
  if (period == 0) return 0.0f;

  return CM_PER_SLOT * ENC_TICK_HZ / (float)period;
}

/* Wheel distances and heading from the signed edge counts */
static void update_odometry(void)
{
  int32_t cl = enc_l.count;   /* 32-bit reads are atomic on Cortex-M3 */
  int32_t cr = enc_r.count;
  dist_l = (float)cl * CM_PER_EDGE;
  dist_r = (float)cr * CM_PER_EDGE;
  heading_deg = (dist_r - dist_l) / TRACK_WIDTH_CM * RAD2DEG;
}

/* ===================== Controllers ===================== */

/* Speed-magnitude PI with feed-forward. Returns PWM counts (>= 0). */
static int32_t speed_pi(SpeedPI_t *c, float target, float meas, float dt, float deadzone)
{
  if (target <= 0.0f) { c->integ = 0.0f; return 0; }

  float e = target - meas;
  c->integ = clampf(c->integ + e * dt, -I_MAX, I_MAX);

  float u = deadzone + KFF * target + KP_V * e + KI_V * c->integ;
  return (int32_t)clampf(u, 0.0f, (float)PWM_MAX);
}

static void enter_state(State_t s)
{
  state = s;
  state_time = 0.0f;
}

static void start_side(void)
{
  heading_target = (float)side_idx * ext_angle * TURN_DIR;
  side_start_l = dist_l;
  side_start_r = dist_r;
  pi_l.integ = 0.0f;
  pi_r.integ = 0.0f;
  v_ramp = V_MIN;
  enter_state(ST_STRAIGHT);
}

/* Runs every 10 ms from the main loop */
static void control_step(float dt)
{
  update_odometry();
  speed_l = encoder_speed(&enc_l);
  speed_r = encoder_speed(&enc_r);

  state_time += dt;

  switch (state)
  {
    case ST_STRAIGHT:
    {
      float dist = 0.5f * ((dist_l - side_start_l) + (dist_r - side_start_r));
      float remaining = side_len - dist;

      if (remaining <= 0.0f)
      {
        motors_brake();
        enter_state(ST_PAUSE_AFTER_STRAIGHT);
        break;
      }

      v_ramp += ACCEL * dt;
      if (v_ramp > V_CRUISE) v_ramp = V_CRUISE;
      float v = v_ramp;
      if (remaining < SLOW_DIST)
      {
        float vs = V_MIN + (V_CRUISE - V_MIN) * (remaining / SLOW_DIST);
        if (vs < v) v = vs;
      }

      /* heading hold: positive error -> turn CCW -> right wheel faster */
      float herr = heading_target - heading_deg;
      float corr = clampf(KH * herr, -H_CORR_MAX, H_CORR_MAX);

      motor_set(MOTOR_L, speed_pi(&pi_l, v - corr, speed_l, dt, PWM_DEADZONE));
      motor_set(MOTOR_R, speed_pi(&pi_r, v + corr, speed_r, dt, PWM_DEADZONE));
      break;
    }

    case ST_PAUSE_AFTER_STRAIGHT:
      motors_brake();
      if (state_time >= PAUSE_S)
      {
        if (side_idx + 1 >= total_sides)
        {
          enter_state(ST_DONE);
        }
        else
        {
          update_odometry();
          heading_target = (float)(side_idx + 1) * ext_angle * TURN_DIR;
          turn_sign = (heading_target - heading_deg >= 0.0f) ? 1.0f : -1.0f;
          pi_l.integ = 0.0f;
          pi_r.integ = 0.0f;
          enter_state(ST_TURN);
        }
      }
      break;

    case ST_TURN:
    {
      /* degrees still to go in the turning direction (negative = overshoot) */
      float remaining = (heading_target - heading_deg) * turn_sign;

      if (remaining <= TURN_STOP_DEG || state_time > TURN_TIMEOUT_S)
      {
        motors_brake();
        enter_state(ST_PAUSE_AFTER_TURN);
        break;
      }

      float v = clampf(TURN_K * remaining, V_TURN_MIN, V_TURN_MAX);
      int32_t pl = speed_pi(&pi_l, v, speed_l, dt, TURN_PWM_DEADZONE);
      int32_t pr = speed_pi(&pi_r, v, speed_r, dt, TURN_PWM_DEADZONE);

      /* turn_sign > 0 -> CCW: left backward, right forward */
      motor_set(MOTOR_L, (turn_sign > 0.0f) ? -pl :  pl);
      motor_set(MOTOR_R, (turn_sign > 0.0f) ?  pr : -pr);
      break;
    }

    case ST_PAUSE_AFTER_TURN:
      motors_brake();
      if (state_time >= PAUSE_S)
      {
        side_idx++;
        start_side();
      }
      break;

    case ST_DONE:
      motors_brake();
      if (state_time >= PAUSE_S)
      {
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
      }
      break;

    case ST_IDLE:
    default:
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Motors off, driver in standby */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  motor_set(MOTOR_L, 0);
  motor_set(MOTOR_R, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

#if DIAG_MOTOR_TEST
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  motor_set(MOTOR_L, 2000);
  motor_set(MOTOR_R, 2000);
  HAL_Delay(1000);
  motors_brake();
  HAL_Delay(300);
  motor_set(MOTOR_L, 0);
  motor_set(MOTOR_R, 0);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
#endif

  /* Read the polygon parameters */
  {
    uint8_t sides = (num_sides < 3) ? 3 : num_sides;
    uint8_t laps  = (num_laps  < 1) ? 1 : num_laps;
    total_sides = (uint16_t)sides * laps;
    ext_angle   = 360.0f / (float)sides;
    side_len    = side_length_cm;
  }

  HAL_Delay(START_DELAY_MS);

  /* Start the encoders and the 100 Hz loop */
  diag_stage = 2;   /* timers failed to start -> 2 blinks */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)              Error_Handler();
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  diag_stage = 3;   /* running: LED blinks once per second */

  /* Go */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  side_idx = 0;
  start_side();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (loop_ticks > 0)
    {
      __disable_irq();
      uint32_t n = loop_ticks;
      loop_ticks = 0;
      __enable_irq();

      control_step(LOOP_DT * (float)n);

      /* Heartbeat: toggles every 0.5 s while the control loop runs */
      static uint8_t hb = 0;
      if (++hb >= 50) { hb = 0; HAL_GPIO_TogglePin(DIAG_LED_PORT, DIAG_LED_PIN); }
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* Both channels: rising edge first, maximum input filter */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICFilter = ENC_IC_FILTER;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* Period was generated as 65535 (~1.1 kHz): force 3599 -> 20 kHz */
  htim3.Init.Period = PWM_MAX;
  __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_MAX);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STBY_Pin */
  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Diagnostic LED on PC13 (active low, start off) */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(DIAG_LED_PORT, DIAG_LED_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = DIAG_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIAG_LED_PORT, &GPIO_InitStruct);
  diag_stage = 1;   /* errors in peripheral init -> 1 blink */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* TIM2 update: every 10 ms -> control loop tick + time base */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    tim2_ovf++;
    loop_ticks++;
  }
}

/* TIM2 capture: one HC020K edge */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    enc_edge(&enc_l, HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1), TIM_CCER_CC1P);
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    enc_edge(&enc_r, HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2), TIM_CCER_CC2P);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Put the driver in standby so the car never runs away on an error */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  __disable_irq();
  /* Blink diag_stage times, then a long pause (busy-wait: SysTick is off) */
  while (1)
  {
    for (uint8_t k = 0; k < diag_stage; k++)
    {
      DIAG_LED_PORT->BRR  = DIAG_LED_PIN;                  /* LED on  */
      for (volatile uint32_t i = 0; i < 600000; i++) {}
      DIAG_LED_PORT->BSRR = DIAG_LED_PIN;                  /* LED off */
      for (volatile uint32_t i = 0; i < 600000; i++) {}
    }
    for (volatile uint32_t i = 0; i < 3000000; i++) {}
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
