/* USER CODE BEGIN Header */
/**
  * @file main.c
  * @brief Simple polygon loop: Advance (encoders) -> Pause -> Turn (time) -> Pause.
  */
/* USER CODE END Header */
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= User-adjustable parameters ============================ */
#define PI_F                                3.14159265f
#define WHEEL_DIAMETER_CM                   6.2f
#define WHEEL_TRACK_CM                     23.0f
#define ENCODER_PULSES_PER_WHEEL_REV       20U

/* Parámetros calibrados */
#define DIST_CAL_FACTOR                    1.33f
#define PWM_AVANCE_PCT                     35.0f
#define TURN_PWM_PERCENT                   25.0f

#define DEFAULT_POLYGON_SIDES              4U
#define DEFAULT_SIDE_LENGTH_CM            15.0f
#define DEFAULT_TURN_DIRECTION             1    /* +1 left, -1 right */
#define AUTO_RUN_AT_BOOT                   1U   /* 1: run once after startup */

/* TIEMPO EXACTO DE GIRO PARA 90° (Ajusta este valor si le falta o se pasa) */
#define TURN_DURATION_90_MS                650U

#define CM_PER_PULSE (DIST_CAL_FACTOR * PI_F * WHEEL_DIAMETER_CM / \
                     (float)ENCODER_PULSES_PER_WHEEL_REV)

#define LEFT_MOTOR_IS_A                    1
#define INVERT_LEFT                        1
#define INVERT_RIGHT                       1

#define PWM_CH_A                           TIM_CHANNEL_1
#define PWM_CH_B                           TIM_CHANNEL_2
#define PWM_ARR                            3599U
#define ENC_L_CH                           TIM_CHANNEL_1
#define ENC_R_CH                           TIM_CHANNEL_2
#define ENC_L_ACTIVE                       HAL_TIM_ACTIVE_CHANNEL_1
#define ENC_R_ACTIVE                       HAL_TIM_ACTIVE_CHANNEL_2
#define ENC_ARR                            65535U
#define ENC_IC_FILTER                      10U

#define ADVANCE_TIMEOUT_MS                 10000U
#define ENCODER_STALL_TIMEOUT_MS            1200U
#define PAUSE_AFTER_ADVANCE_MS             1500U
#define PAUSE_AFTER_TURN_MS                1500U
#define STARTUP_DELAY_MS                   2000U

typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t channel;
} Motor_t;

typedef struct {
    volatile uint32_t pulses;
} Encoder_t;

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

volatile uint8_t polygon_sides = DEFAULT_POLYGON_SIDES;
volatile float polygon_side_cm = DEFAULT_SIDE_LENGTH_CM;
volatile int8_t polygon_turn_direction = DEFAULT_TURN_DIRECTION;
volatile uint8_t run_request = 0U;

static const Motor_t motorA = { AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, PWM_CH_A };
static const Motor_t motorB = { BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, PWM_CH_B };
#if LEFT_MOTOR_IS_A
static const Motor_t * const mL = &motorA;
static const Motor_t * const mR = &motorB;
#else
static const Motor_t * const mL = &motorB;
static const Motor_t * const mR = &motorA;
#endif
static Encoder_t encL = {0U};
static Encoder_t encR = {0U};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Error_Handler(void);

static void Motors_Init(void);
static void Encoders_Init(void);
static void Motor_Drive(const Motor_t *motor, float percent);
static void Motor_Set(float left, float right);
static void Motor_Brake(void);
static void Encoder_Reset(void);
static void Encoder_Snapshot(uint32_t *left, uint32_t *right);
static void Drive_Straight(float distance_cm);
static void Turn_Once(int8_t direction);
static float AbsF(float x);
static float ClampF(float x, float lo, float hi);

static float AbsF(float x) { return (x < 0.0f) ? -x : x; }
static float ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void Motor_Drive(const Motor_t *motor, float percent)
{
    percent = ClampF(percent, -100.0f, 100.0f);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr = (uint32_t)(AbsF(percent) * (float)(arr + 1U) / 100.0f);

    if (percent > 0.0f) {
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
    } else if (percent < 0.0f) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    } else {
        ccr = 0U;
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, motor->channel, ccr);
}

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
    __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH_A, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH_B, 0U);
    if (HAL_TIM_PWM_Start(&htim3, PWM_CH_A) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, PWM_CH_B) != HAL_OK) Error_Handler();
    Motor_Brake();
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}

static void Encoders_Init(void)
{
    __HAL_TIM_SET_AUTORELOAD(&htim2, ENC_ARR);
    encL.pulses = 0U;
    encR.pulses = 0U;
    if (HAL_TIM_IC_Start_IT(&htim2, ENC_L_CH) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_Start_IT(&htim2, ENC_R_CH) != HAL_OK) Error_Handler();
}

static void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    encL.pulses = 0U;
    encR.pulses = 0U;
    if (primask == 0U) __enable_irq();
}

static void Encoder_Snapshot(uint32_t *left, uint32_t *right)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *left = encL.pulses;
    *right = encR.pulses;
    if (primask == 0U) __enable_irq();
}

/* ================= AVANCE RECTO (Exactamente el que te funcionó) ================= */
static void Drive_Straight(float distance_cm)
{
    Encoder_Reset();
    uint32_t left_start, right_start;
    Encoder_Snapshot(&left_start, &right_start);
    uint32_t target_pulses = (uint32_t)(distance_cm / CM_PER_PULSE + 0.99999f);
    float measured_progress_cm = 0.0f;

    uint32_t start_ms = HAL_GetTick();
    uint32_t last_left_ms = start_ms;
    uint32_t last_right_ms = start_ms;
    uint32_t prev_left = left_start;
    uint32_t prev_right = right_start;

    Motor_Set(PWM_AVANCE_PCT, PWM_AVANCE_PCT);

    for (;;) {
        uint32_t now = HAL_GetTick();
        uint32_t left_now, right_now;
        Encoder_Snapshot(&left_now, &right_now);
        uint32_t left_delta = left_now - left_start;
        uint32_t right_delta = right_now - right_start;
        measured_progress_cm = 0.5f * (float)(left_delta + right_delta) * CM_PER_PULSE;

        if (left_now != prev_left) { prev_left = left_now; last_left_ms = now; }
        if (right_now != prev_right) { prev_right = right_now; last_right_ms = now; }

        if ((now - last_left_ms) >= ENCODER_STALL_TIMEOUT_MS ||
            (now - last_right_ms) >= ENCODER_STALL_TIMEOUT_MS ||
            (now - start_ms) >= ADVANCE_TIMEOUT_MS) {
            break; /* Salir por seguridad ante cualquier bloqueo */
        }

        if (measured_progress_cm >= distance_cm) {
            break;
        }
        HAL_Delay(2U);
    }

    Motor_Brake();
    HAL_Delay(PAUSE_AFTER_ADVANCE_MS); /* Pausa visible al terminar de avanzar */
}

/* ================= GIRO DE 90° (Por tiempo calibrado y limpio) ================= */
static void Turn_Once(int8_t direction)
{
    Motor_Brake();
    HAL_Delay(100U);

    if (direction > 0) {
        Motor_Set(-TURN_PWM_PERCENT, TURN_PWM_PERCENT); /* Izquierda */
    } else {
        Motor_Set(TURN_PWM_PERCENT, -TURN_PWM_PERCENT); /* Derecha */
    }

    HAL_Delay(TURN_DURATION_90_MS); /* Duración exacta del giro de 90° */

    Motor_Brake();
    HAL_Delay(PAUSE_AFTER_TURN_MS); /* Pausa visible al terminar de girar */
}

/* ================= BUCLE PRINCIPAL DEL POLÍGONO ================= */
static void Polygon_Run(uint8_t sides, float side_cm, int8_t direction)
{
    for (uint8_t i = 0U; i < sides; i++) {
        // 1. Avanzar el tramo recto y parar
        Drive_Straight(side_cm);

        // Si es el último lado, rompemos para cerrar el polígono sin girar de más
        if (i == (sides - 1U)) {
            break;
        }

        // 2. Girar 90° y parar
        Turn_Once(direction);
    }
    Motor_Brake();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    Motors_Init();
    Encoders_Init();
    HAL_Delay(STARTUP_DELAY_MS);

#if AUTO_RUN_AT_BOOT
    run_request = 1U;
#endif

    while (1) {
        if (run_request != 0U) {
            run_request = 0U;

            /* Ejecuta la secuencia completa en bucle por cada lado del polígono */
            Polygon_Run(polygon_sides, polygon_side_cm, polygon_turn_direction);

            Motor_Brake();
        }
        HAL_Delay(10U);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (htim->Channel == ENC_L_ACTIVE) {
        (void)HAL_TIM_ReadCapturedValue(htim, ENC_L_CH);
        encL.pulses++;
    } else if (htim->Channel == ENC_R_ACTIVE) {
        (void)HAL_TIM_ReadCapturedValue(htim, ENC_R_CH);
        encR.pulses++;
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  __HAL_TIM_SET_AUTORELOAD(&htim2, ENC_ARR);
  sConfigIC.ICFilter = ENC_IC_FILTER;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, ENC_L_CH) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, ENC_R_CH) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_ARR);
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH_A, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CH_B, 0U);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    __disable_irq();
    while (1) { }
}
