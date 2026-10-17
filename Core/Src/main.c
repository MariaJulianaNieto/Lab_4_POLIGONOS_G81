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
  *
  * Polygon car (STM32F103C6, 72 MHz):
  *   - TB6612FNG : PWMA = TIM2_CH1 (PA0), PWMB = TIM2_CH3 (PB10), 20 kHz
  *                 AIN1, AIN2, BIN1, BIN2, STBY = GPIO outputs
  *   - HC-020K   : ENC_I = TIM1_CH1 (PA8), ENC_D = TIM1_CH3 (PA10)
  *                 input capture, 10 us resolution -> speed from pulse period
  *   - MPU6050   : SCL / SDA, bit-banged I2C (open drain)
  *
  * Main loop at 200 Hz: gyro integration, per-wheel speed PI, heading hold,
  * in-place turns with the gyro.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  GPIO_TypeDef *in1_port;  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;  uint16_t in2_pin;
  uint32_t      channel;   /* TIM2 channel */
} Motor_t;

typedef struct
{
  volatile int32_t  pulses;    /* rising edges counted                 */
  volatile uint16_t last_cap;  /* last captured TIM1 value             */
  volatile uint16_t period;    /* TIM1 ticks between pulses, 0 = none  */
  volatile uint32_t last_ms;   /* HAL_GetTick() of the last pulse      */
  volatile uint8_t  seen;      /* at least one pulse received          */
} Encoder_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------------- Polygon (defaults, can be changed at runtime) ---------- */
#define DEFAULT_SIDES            4
#define DEFAULT_SIDE_LENGTH_CM   50.0f
#define TURN_DIRECTION           (+1.0f)   /* +1 = turn left (CCW), -1 = right */

/* ---------------- Mechanics ---------------------------------------------- */
#define PI_F                3.14159265f
#define WHEEL_DIAMETER_CM   6.2f
#define ENCODER_SLOTS       20U            /* HC-020K disk, rising edges only  */
#define CM_PER_PULSE        ((PI_F * WHEEL_DIAMETER_CM) / (float)ENCODER_SLOTS)  /* 0.974 cm */

/* ---------------- Wiring / orientation ----------------------------------- */
#define LEFT_MOTOR_IS_A     1              /* 1: A = left, B = right           */
#define INVERT_LEFT         0              /* 1 if left wheel spins backwards  */
#define INVERT_RIGHT        0
#define GYRO_SIGN           (+1.0f)        /* yaw must INCREASE on a left turn */

/* ---------------- Encoders (TIM1: 72 MHz / 720 = 100 kHz) ---------------- */
#define TIM1_TICK_S         10.0e-6f       /* 10 us per TIM1 count             */
#define ENC_MIN_PERIOD      200U           /* 2 ms: shorter = noise, ignored   */
#define ENC_FRESH_MS        600U           /* < 655 ms TIM1 overflow time      */
#define ENC_STOP_MS         400U           /* no pulse for this long = stopped */
#define ENC_IC_FILTER       15U            /* input capture digital filter     */

/* ---------------- Control ------------------------------------------------ */
#define CONTROL_PERIOD_MS   5U             /* 200 Hz control loop              */
#define CONTROL_DT_S        (CONTROL_PERIOD_MS / 1000.0f)

#define CRUISE_SPEED_CMS    25.0f
#define MIN_SPEED_CMS       8.0f
#define ACCEL_CMS2          40.0f
#define DECEL_DIST_CM       10.0f
#define BRAKE_COMP_CM       0.5f           /* stop this much early (coasting)  */

#define PWM_DEADZONE        25.0f          /* % needed to start moving         */
#define PWM_PER_CMS         1.2f           /* feed-forward % per cm/s          */
#define PWM_MAX_STRAIGHT    95.0f

#define SPEED_KP            1.5f           /* % per cm/s                       */
#define SPEED_KI            4.0f           /* % per (cm/s * s)                 */
#define SPEED_I_LIMIT       25.0f          /* max integral contribution (%)    */

#define HEAD_KP             0.8f           /* cm/s per degree                  */
#define HEAD_KD             0.02f          /* cm/s per deg/s                   */
#define HEAD_CORR_MAX       10.0f          /* cm/s                             */

#define TURN_KP             1.2f           /* % per degree                     */
#define TURN_KD             0.15f          /* % per deg/s                      */
#define TURN_PWM_MIN        28.0f
#define TURN_PWM_MAX        55.0f
#define TURN_TOL_DEG        1.0f
#define TURN_RATE_TOL_DPS   5.0f
#define TURN_TIMEOUT_MS     5000U

#define SETTLE_MS           300U

/* ---------------- MPU6050 ------------------------------------------------ */
#define MPU_ADDR            0x68U          /* AD0 = GND                        */
#define MPU_SMPLRT_DIV      0x19U
#define MPU_CONFIG          0x1AU
#define MPU_GYRO_CONFIG     0x1BU
#define MPU_ACCEL_CONFIG    0x1CU
#define MPU_GYRO_ZOUT_H     0x47U
#define MPU_PWR_MGMT_1      0x6BU
#define MPU_WHO_AM_I        0x75U

#define GYRO_LSB_PER_DPS    65.5f          /* +-500 dps range                  */
#define CALIB_SAMPLES       1000U
#define CALIB_MAX_RANGE     150            /* LSB; larger = car was moving     */
#define CALIB_RETRIES       5U

/* ---------------- Bit-banged I2C ----------------------------------------- */
#define I2C_HALF_US         4U             /* ~100 kHz                         */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SCL_H()      (SCL_GPIO_Port->BSRR = SCL_Pin)
#define SCL_L()      (SCL_GPIO_Port->BRR  = SCL_Pin)
#define SDA_H()      (SDA_GPIO_Port->BSRR = SDA_Pin)
#define SDA_L()      (SDA_GPIO_Port->BRR  = SDA_Pin)
#define SDA_READ()   ((SDA_GPIO_Port->IDR & SDA_Pin) != 0U)
#define I2C_DELAY()  delay_us(I2C_HALF_US)

#define ABSF(x)      (((x) < 0.0f) ? -(x) : (x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static const Motor_t motorA = { AIN1_GPIO_Port, AIN1_Pin,
                                AIN2_GPIO_Port, AIN2_Pin, TIM_CHANNEL_1 };
static const Motor_t motorB = { BIN1_GPIO_Port, BIN1_Pin,
                                BIN2_GPIO_Port, BIN2_Pin, TIM_CHANNEL_3 };

#if LEFT_MOTOR_IS_A
static const Motor_t * const mL = &motorA;
static const Motor_t * const mR = &motorB;
#else
static const Motor_t * const mL = &motorB;
static const Motor_t * const mR = &motorA;
#endif

static Encoder_t encL = {0};   /* ENC_I, TIM1_CH1 */
static Encoder_t encR = {0};   /* ENC_D, TIM1_CH3 */

/* Visible in Live Expressions */
volatile float yaw_deg     = 0.0f;
volatile float yaw_rate    = 0.0f;
volatile float speed_left  = 0.0f;   /* cm/s */
volatile float speed_right = 0.0f;   /* cm/s */
static float   gyro_bias   = 0.0f;
static uint32_t last_gyro_cyc = 0;

/* Polygon parameters: change them in the debugger and set run_request = 1 */
volatile uint8_t polygon_sides   = DEFAULT_SIDES;
volatile float   polygon_side_cm = DEFAULT_SIDE_LENGTH_CM;
volatile uint8_t run_request     = 0;

static uint32_t next_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void delay_us(uint32_t us);

static void Motors_Init(void);
static void Motor_Set(float left, float right);
static void Motor_Brake(void);

static void  Encoders_Init(void);
static float Enc_Speed(const Encoder_t *e);

static void i2c_bus_recover(void);
static bool MPU_Write(uint8_t reg, uint8_t val);
static bool MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len);
static bool MPU_Init(void);
static bool MPU_ReadGyroZ(int16_t *gz);
static void MPU_Calibrate(void);
static void Gyro_Update(void);

static void Control_Start(void);
static void Control_WaitTick(void);
static void Settle(uint32_t ms);
static void Drive_Straight(float dist_cm, float heading_ref);
static void Turn_To(float heading_ref);
static void Polygon_Run(uint8_t sides, float side_cm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ======================= Timing (DWT cycle counter) ======================= */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t n = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < n) { }
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* ======================= Motors: TB6612FNG + TIM2 PWM ===================== */
static void Motor_Drive(const Motor_t *m, float pct)
{
  pct = clampf(pct, -100.0f, 100.0f);

  uint32_t top = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;   /* 3600 */
  uint32_t ccr = (uint32_t)((ABSF(pct) * (float)top) / 100.0f);

  if (pct > 0.0f)           /* forward: IN1 = H, IN2 = L */
  {
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
  }
  else if (pct < 0.0f)      /* reverse: IN1 = L, IN2 = H */
  {
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
  }
  else                      /* short brake: IN1 = H, IN2 = H */
  {
    ccr = 0;
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
  }
  __HAL_TIM_SET_COMPARE(&htim2, m->channel, ccr);
}

/* left/right in percent, -100..100 (positive = forward) */
static void Motor_Set(float left, float right)
{
#if INVERT_LEFT
  left = -left;
#endif
#if INVERT_RIGHT
  right = -right;
#endif
  Motor_Drive(mL, left);
  Motor_Drive(mR, right);
}

static void Motor_Brake(void)
{
  Motor_Set(0.0f, 0.0f);
}

static void Motors_Init(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  Motor_Brake();
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);   /* enable driver */
}

/* ======================= Encoders: TIM1 input capture ===================== */
static void Encoders_Init(void)
{
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
}

/* Called from the capture interrupt for every rising edge */
static void Enc_OnCapture(Encoder_t *e, uint16_t cap)
{
  uint32_t now = HAL_GetTick();

  if (e->seen && (now - e->last_ms) < ENC_FRESH_MS)
  {
    uint16_t dt = (uint16_t)(cap - e->last_cap);   /* wraps correctly */
    if (dt < ENC_MIN_PERIOD)
    {
      return;                                      /* glitch, ignore */
    }
    e->period = dt;
  }
  else
  {
    e->period = 0;                                 /* first pulse after a stop */
  }

  e->last_cap = cap;
  e->last_ms  = now;
  e->seen     = 1;
  e->pulses++;
}

/* Wheel speed in cm/s (magnitude) */
static float Enc_Speed(const Encoder_t *e)
{
  __disable_irq();
  uint16_t per  = e->period;
  uint32_t last = e->last_ms;
  uint8_t  seen = e->seen;
  __enable_irq();

  uint32_t age = HAL_GetTick() - last;
  if (!seen || per == 0U || age > ENC_STOP_MS)
  {
    return 0.0f;
  }

  float v = CM_PER_PULSE / ((float)per * TIM1_TICK_S);

  /* if no pulse arrived for longer than the last period, the wheel is slowing */
  if (age > 0U)
  {
    float v_max = CM_PER_PULSE * 1000.0f / (float)age;
    if (v_max < v) v = v_max;
  }
  return v;
}

/* ======================= Bit-banged I2C =================================== */
static void i2c_start(void)
{
  SDA_H(); I2C_DELAY();
  SCL_H(); I2C_DELAY();
  SDA_L(); I2C_DELAY();
  SCL_L(); I2C_DELAY();
}

static void i2c_stop(void)
{
  SDA_L(); I2C_DELAY();
  SCL_H(); I2C_DELAY();
  SDA_H(); I2C_DELAY();
}

static bool i2c_write_byte(uint8_t b)
{
  for (uint8_t i = 0; i < 8U; i++)
  {
    if (b & 0x80U) SDA_H(); else SDA_L();
    b <<= 1;
    I2C_DELAY();
    SCL_H(); I2C_DELAY();
    SCL_L();
  }
  SDA_H();                     /* release SDA for ACK */
  I2C_DELAY();
  SCL_H(); I2C_DELAY();
  bool ack = !SDA_READ();
  SCL_L(); I2C_DELAY();
  return ack;
}

static uint8_t i2c_read_byte(bool ack)
{
  uint8_t b = 0;
  SDA_H();
  for (uint8_t i = 0; i < 8U; i++)
  {
    b <<= 1;
    I2C_DELAY();
    SCL_H(); I2C_DELAY();
    if (SDA_READ()) b |= 1U;
    SCL_L();
  }
  if (ack) SDA_L(); else SDA_H();
  I2C_DELAY();
  SCL_H(); I2C_DELAY();
  SCL_L();
  SDA_H(); I2C_DELAY();
  return b;
}

static void i2c_bus_recover(void)
{
  SDA_H();
  for (uint8_t i = 0; i < 9U; i++)
  {
    SCL_L(); I2C_DELAY();
    SCL_H(); I2C_DELAY();
  }
  i2c_stop();
}

/* ======================= MPU6050 ========================================== */
static bool MPU_Write(uint8_t reg, uint8_t val)
{
  i2c_start();
  if (!i2c_write_byte((uint8_t)(MPU_ADDR << 1))) goto fail;
  if (!i2c_write_byte(reg))                      goto fail;
  if (!i2c_write_byte(val))                      goto fail;
  i2c_stop();
  return true;
fail:
  i2c_stop();
  return false;
}

static bool MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len)
{
  i2c_start();
  if (!i2c_write_byte((uint8_t)(MPU_ADDR << 1)))         goto fail;
  if (!i2c_write_byte(reg))                              goto fail;
  i2c_start();                                           /* repeated start */
  if (!i2c_write_byte((uint8_t)((MPU_ADDR << 1) | 1U)))  goto fail;
  while (len--)
  {
    *buf++ = i2c_read_byte(len > 0U);                    /* NACK last byte */
  }
  i2c_stop();
  return true;
fail:
  i2c_stop();
  return false;
}

static bool MPU_Init(void)
{
  uint8_t who = 0;

  HAL_Delay(100);
  i2c_bus_recover();

  if (!MPU_Read(MPU_WHO_AM_I, &who, 1)) return false;    /* no ACK = not wired */
  /* 0x68 on originals; clones may answer 0x70/0x72/0x98, so not checked */

  if (!MPU_Write(MPU_PWR_MGMT_1, 0x80))   return false;  /* device reset       */
  HAL_Delay(100);
  if (!MPU_Write(MPU_PWR_MGMT_1, 0x01))   return false;  /* wake, PLL gyro X   */
  HAL_Delay(10);
  if (!MPU_Write(MPU_SMPLRT_DIV, 0x00))   return false;  /* 1 kHz              */
  if (!MPU_Write(MPU_CONFIG, 0x03))       return false;  /* DLPF ~44 Hz        */
  if (!MPU_Write(MPU_GYRO_CONFIG, 0x08))  return false;  /* +-500 dps          */
  if (!MPU_Write(MPU_ACCEL_CONFIG, 0x00)) return false;  /* +-2 g              */
  HAL_Delay(50);
  return true;
}

static bool MPU_ReadGyroZ(int16_t *gz)
{
  uint8_t b[2];
  if (!MPU_Read(MPU_GYRO_ZOUT_H, b, 2))
  {
    i2c_bus_recover();
    return false;
  }
  *gz = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  return true;
}

/* Car must be still. Repeats if movement is detected. */
static void MPU_Calibrate(void)
{
  Motor_Brake();
  HAL_Delay(1000);                         /* let vibrations die out */

  for (uint8_t attempt = 0; attempt < CALIB_RETRIES; attempt++)
  {
    int32_t  sum = 0;
    uint32_t ok  = 0;
    int16_t  gz, gmin = 32767, gmax = -32768;

    for (uint32_t i = 0; i < CALIB_SAMPLES; i++)
    {
      if (MPU_ReadGyroZ(&gz))
      {
        sum += gz;
        ok++;
        if (gz < gmin) gmin = gz;
        if (gz > gmax) gmax = gz;
      }
      HAL_Delay(2);
    }

    if (ok > 0U)
    {
      gyro_bias = (float)sum / (float)ok;
      if ((gmax - gmin) <= CALIB_MAX_RANGE)
      {
        break;                             /* good calibration */
      }
    }
  }

  yaw_deg  = 0.0f;
  yaw_rate = 0.0f;
  last_gyro_cyc = DWT->CYCCNT;
}

static void Gyro_Update(void)
{
  int16_t  gz;
  uint32_t now = DWT->CYCCNT;
  float    dt  = (float)(now - last_gyro_cyc) / (float)SystemCoreClock;
  last_gyro_cyc = now;

  if (MPU_ReadGyroZ(&gz))
  {
    yaw_rate = GYRO_SIGN * ((float)gz - gyro_bias) / GYRO_LSB_PER_DPS;
  }
  /* if the read failed, the previous rate is used for this step */
  yaw_deg += yaw_rate * dt;
}

/* ======================= Control loop helpers ============================= */
static void Control_Start(void)
{
  next_tick = HAL_GetTick() + CONTROL_PERIOD_MS;
}

static void Control_WaitTick(void)
{
  while ((int32_t)(HAL_GetTick() - next_tick) < 0) { }
  next_tick += CONTROL_PERIOD_MS;
  Gyro_Update();
  speed_left  = Enc_Speed(&encL);
  speed_right = Enc_Speed(&encR);
}

static void Settle(uint32_t ms)
{
  Motor_Brake();
  Control_Start();
  for (uint32_t t = 0; t < ms; t += CONTROL_PERIOD_MS)
  {
    Control_WaitTick();                   /* keep integrating the gyro */
  }
}

/* Feed-forward: PWM needed for a given wheel speed */
static float FeedForward(float v_cms)
{
  return (v_cms > 0.0f) ? (PWM_DEADZONE + PWM_PER_CMS * v_cms) : 0.0f;
}

/* ======================= Straight side ==================================== */
static void Drive_Straight(float dist_cm, float heading_ref)
{
  const int32_t  l0 = encL.pulses;
  const int32_t  r0 = encR.pulses;
  const uint32_t timeout_ms = (uint32_t)(dist_cm / MIN_SPEED_CMS * 1000.0f) * 2U + 2000U;
  const float    i_lim = SPEED_I_LIMIT / SPEED_KI;

  float    iL = 0.0f, iR = 0.0f;
  uint32_t t_ms = 0;

  Control_Start();
  for (;;)
  {
    Control_WaitTick();
    t_ms += CONTROL_PERIOD_MS;

    int32_t sum       = (encL.pulses - l0) + (encR.pulses - r0);
    float   traveled  = 0.5f * (float)sum * CM_PER_PULSE;
    float   remaining = dist_cm - traveled;

    if (remaining <= BRAKE_COMP_CM || t_ms > timeout_ms)
    {
      break;
    }

    /* speed profile: ramp up, cruise, ramp down */
    float v_ref = MIN_SPEED_CMS + ACCEL_CMS2 * ((float)t_ms / 1000.0f);
    if (v_ref > CRUISE_SPEED_CMS) v_ref = CRUISE_SPEED_CMS;
    if (remaining < DECEL_DIST_CM)
    {
      float v_dec = MIN_SPEED_CMS + (CRUISE_SPEED_CMS - MIN_SPEED_CMS) * remaining / DECEL_DIST_CM;
      if (v_dec < v_ref) v_ref = v_dec;
    }

    /* heading hold: yaw below reference -> car drifted right -> right wheel faster */
    float herr = heading_ref - yaw_deg;
    float corr = clampf(HEAD_KP * herr - HEAD_KD * yaw_rate, -HEAD_CORR_MAX, HEAD_CORR_MAX);

    float refL = v_ref - corr;
    float refR = v_ref + corr;
    if (refL < 0.0f) refL = 0.0f;
    if (refR < 0.0f) refR = 0.0f;

    /* per-wheel speed PI with feed-forward */
    float eL = refL - speed_left;
    float eR = refR - speed_right;
    iL = clampf(iL + eL * CONTROL_DT_S, -i_lim, i_lim);
    iR = clampf(iR + eR * CONTROL_DT_S, -i_lim, i_lim);

    float pL = FeedForward(refL) + SPEED_KP * eL + SPEED_KI * iL;
    float pR = FeedForward(refR) + SPEED_KP * eR + SPEED_KI * iR;

    Motor_Set(clampf(pL, 0.0f, PWM_MAX_STRAIGHT),
              clampf(pR, 0.0f, PWM_MAX_STRAIGHT));
  }
  Motor_Brake();
}

/* ======================= Turn in place ==================================== */
static void Turn_To(float heading_ref)
{
  uint8_t  in_tol = 0;
  uint32_t t_ms   = 0;

  Control_Start();
  for (;;)
  {
    Control_WaitTick();
    t_ms += CONTROL_PERIOD_MS;
    if (t_ms > TURN_TIMEOUT_MS) break;

    float err = heading_ref - yaw_deg;

    if (ABSF(err) < TURN_TOL_DEG && ABSF(yaw_rate) < TURN_RATE_TOL_DPS)
    {
      Motor_Brake();
      if (++in_tol >= 4U) break;          /* 20 ms stable on target */
      continue;
    }
    in_tol = 0;

    float cmd = TURN_KP * err - TURN_KD * yaw_rate;
    float mag = clampf(ABSF(cmd), TURN_PWM_MIN, TURN_PWM_MAX);
    float p   = (cmd >= 0.0f) ? mag : -mag;

    /* p > 0 -> turn left (CCW): left wheel back, right wheel forward */
    Motor_Set(-p, p);
  }
  Motor_Brake();
}

/* ======================= Polygon ========================================== */
static void Polygon_Run(uint8_t sides, float side_cm)
{
  if (sides < 3U)       sides = 3U;
  if (side_cm < 5.0f)   side_cm = 5.0f;

  const float exterior = TURN_DIRECTION * 360.0f / (float)sides;
  float heading = yaw_deg;                /* absolute reference: no error build-up */

  for (uint8_t i = 0; i < sides; i++)
  {
    Drive_Straight(side_cm, heading);
    Settle(SETTLE_MS);

    heading += exterior;
    Turn_To(heading);
    Settle(SETTLE_MS);
  }
  Motor_Brake();
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  Motors_Init();
  Encoders_Init();

  if (!MPU_Init())
  {
    Error_Handler();                      /* MPU6050 not answering: check wiring */
  }

  run_request = 1;                        /* draw one polygon after power-up */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (run_request)
    {
      MPU_Calibrate();                    /* keep the car still (~3 s) */
      HAL_Delay(500);
      Polygon_Run(polygon_sides, polygon_side_cm);
      run_request = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  TIM_IC_InitTypeDef sConfigIC = {0};

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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* Same (stronger) noise filter on both encoder channels */
  sConfigIC.ICFilter = ENC_IC_FILTER;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin STBY_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* I2C1 peripheral is not enabled, so SCL/SDA are switched to plain
     open-drain outputs for the bit-banged I2C (GY-521 has pull-ups).
     Released = high. */
  SCL_GPIO_Port->BSRR = SCL_Pin;
  SDA_GPIO_Port->BSRR = SDA_Pin;

  GPIO_InitStruct.Pin   = SCL_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = SDA_Pin;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* TIM1 input capture: one call per rising edge of an encoder.
   Needs "TIM1 capture compare interrupt" enabled in CubeMX NVIC. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)          /* ENC_I, PA8 */
  {
    Enc_OnCapture(&encL, (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)     /* ENC_D, PA10 */
  {
    Enc_OnCapture(&encR, (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3));
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
  /* User can add his own implementation to report the HAL error return state */
  TIM2->CCR1 = 0;                                               /* PWM off    */
  TIM2->CCR3 = 0;
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);  /* driver off */
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
