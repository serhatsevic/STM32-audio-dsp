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
#include "i2c_lcd.h"
#include "stm32f4_dac3100.h"
#include <string.h>
#include <stdio.h>

#include "arm_math.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  UI_MODE_ADJUST = 0,
  UI_MODE_FUNCTION = 1
} ui_mode_t;

typedef enum {
  UI_PARAM_EQ_BASS = 0,
  UI_PARAM_COMP_ATTACK,
  UI_PARAM_EQ_MID,
  UI_PARAM_COMP_HOLD,
  UI_PARAM_EQ_TREBLE,
  UI_PARAM_COMP_RELEASE,
  UI_PARAM_DELAY_MS,
  UI_PARAM_DELAY_FB,
  UI_PARAM_MASTER_VOL,
  UI_PARAM_COMP_RATIO,
  UI_PARAM_NONE = 255
} ui_param_id_t;

typedef struct {
  const char *name;
  const char *unit;
  int32_t ui_min;
  int32_t ui_max;
  int32_t ui_step;
  int32_t ui_value;
} ui_dataset_t;

typedef struct {
  TIM_HandleTypeDef *htim;
  GPIO_TypeDef *sw_port;
  uint16_t sw_pin;

  uint32_t last_rawVal;
  int32_t accum_counts;

  volatile uint8_t btn_armed;
  volatile uint8_t btn_event;
  uint32_t last_btn_ms;

  ui_mode_t mode;

  ui_param_id_t selected0;
  ui_param_id_t selected1;
  ui_param_id_t selected2;
  ui_param_id_t active_param;
  ui_param_id_t cursor;
} ui_control_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENC_COUNTS_PER_STEP 4

#define DMA_BUFFER_SIZE 512
#define BTN_DEBOUNCE_MS  40

#define EQ_STAGES   3  // 3 cascaded biquads: low-shelf + peaking + high-shelf

#define EQ_BASS_FREQ_HZ    200.0f
#define EQ_MID_FREQ_HZ    1000.0f
#define EQ_TREBLE_FREQ_HZ 4000.0f
#define EQ_MID_Q             1.3f
#define EQ_SHELF_SLOPE       1.5f
#define EQ_PI                3.14159265358979323846f

#define FS_HZ            48000
#define DELAY_MAX_MS     150
#define DELAY_MAX_SAMPLES ((FS_HZ * DELAY_MAX_MS) / 1000)   // 7200


//COMPRESSOR

#define COMP_ATTACK_MS    5
#define COMP_HOLD_MS      20
#define COMP_RELEASE_MS   80
#define COMP_THRESHOLD    0.25f
#define COMP_END_GAIN     0.5f
#define COMP_ENABLE       1

// Compressor ratio presets.
// The compressor algorithm is unchanged: selecting a ratio only loads
// a preset threshold and end-gain value.
#define COMP_RATIO_2_TO_1_THRESHOLD   0.25f
#define COMP_RATIO_2_TO_1_END_GAIN    0.50f
#define COMP_RATIO_4_TO_1_THRESHOLD   0.20f
#define COMP_RATIO_4_TO_1_END_GAIN    0.25f

// TLV320DAC3100 DAC digital volume registers
// Page 0, Reg 65/66: Left/Right DAC digital volume control
#define DAC3100_I2C_ADDR_7BIT       0x18
#define DAC3100_PAGE_SELECT_REG     0x00
#define DAC3100_DAC_LEFT_VOL_REG    0x41
#define DAC3100_DAC_RIGHT_VOL_REG   0x42


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//DEFINE ADC AND DAC DATA BUFFERS
int16_t adcData[DMA_BUFFER_SIZE];
int16_t dacData[DMA_BUFFER_SIZE];

static volatile int16_t *inBufferPtr;
static volatile int16_t *outBufferPtr = &dacData[0];

static volatile uint8_t dataReady; //Data ready flag

static tlv320dac3100_t g_dac;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void dsp_apply_ui_values(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



static ui_dataset_t g_ds[] = {
  { "BASS", "dB",  -12,  12, 1,  0 },
  { "ATTK", "ms",    0, 200, 1,  5 },
  { "MID",  "dB",  -12,  12, 1,  0 },
  { "HOLD", "ms",    0, 500, 1, 20 },
  { "TREB", "dB",  -12,  12, 1,  0 },
  { "RELS", "ms",    1, 800, 1, 80 },
  { "DLY",  "ms",    0, 150, 1,  0 },
  { "FB",   "%",     0, 100, 1,  0 },
  { "VOL",  "dB",  -63,   0, 1,  0 },
  { "RATIO",":1",     2,   4, 2,  2 }
};

static ui_control_t g_ui[4] = {
  /* ENC1: Bass / Attack / Delay */
  { &htim2, GPIOC, GPIO_PIN_8,  0, 0, 1, 0, 0, UI_MODE_ADJUST, UI_PARAM_EQ_BASS,    UI_PARAM_COMP_ATTACK,  UI_PARAM_DELAY_MS,    UI_PARAM_EQ_BASS,     UI_PARAM_EQ_BASS     },

  /* ENC2: Mid / Hold / Compressor ratio */
  { &htim1, GPIOC, GPIO_PIN_9,  0, 0, 1, 0, 0, UI_MODE_ADJUST, UI_PARAM_EQ_MID,     UI_PARAM_COMP_HOLD,    UI_PARAM_COMP_RATIO,  UI_PARAM_EQ_MID,      UI_PARAM_EQ_MID      },

  /* ENC3: Treble / Release */
  { &htim3, GPIOC, GPIO_PIN_7,  0, 0, 1, 0, 0, UI_MODE_ADJUST, UI_PARAM_EQ_TREBLE,  UI_PARAM_COMP_RELEASE, UI_PARAM_NONE,        UI_PARAM_EQ_TREBLE,   UI_PARAM_EQ_TREBLE   },

  /* ENC4/SW4 on PC10: Master DAC volume only */
  { &htim4, GPIOC, GPIO_PIN_10, 0, 0, 1, 0, 0, UI_MODE_ADJUST, UI_PARAM_MASTER_VOL, UI_PARAM_NONE,         UI_PARAM_NONE,        UI_PARAM_MASTER_VOL,  UI_PARAM_MASTER_VOL  }
};

static volatile uint8_t g_ui_dirty = 1;
static uint32_t g_last_lcd_ms = 0;
static uint8_t g_active_ui = 0;

// START VALUES(DEFAULT)
static int32_t g_eq_bass_db_cache   = 999;
static int32_t g_eq_mid_db_cache    = 999;
static int32_t g_eq_treble_db_cache = 999;
static int32_t g_master_vol_db_cache = 999;
static int32_t g_comp_ratio_cache = 999;
static volatile HAL_StatusTypeDef g_last_dac_vol_status = HAL_OK;

static float32_t g_delay_mix = 0.35f;

/* Helper: clamp */
static inline int32_t ui_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static HAL_StatusTypeDef dac3100_write_reg_direct(uint8_t page, uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef st;
  uint8_t page_value;

  page_value = page;

  st = HAL_I2C_Mem_Write(&hi2c1,
                         (uint16_t)(DAC3100_I2C_ADDR_7BIT << 1),
                         DAC3100_PAGE_SELECT_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &page_value,
                         1,
                         HAL_MAX_DELAY);

  if (st != HAL_OK) {
    return st;
  }

  st = HAL_I2C_Mem_Write(&hi2c1,
                         (uint16_t)(DAC3100_I2C_ADDR_7BIT << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         &value,
                         1,
                         HAL_MAX_DELAY);

  return st;
}

static uint8_t dac3100_volume_db_to_reg(int32_t vol_db)
{
  int32_t half_db_steps;

  /*
   * TLV320DAC3100 DAC digital volume uses 0.5 dB steps.
   *  0 dB   = 0x00
   * -0.5 dB = 0xFF
   * -63 dB  = 0x82
   *
   * This UI uses integer dB steps, so each step is 2 register counts.
   */
  vol_db = ui_clamp_i32(vol_db, -63, 0);
  half_db_steps = vol_db * 2;

  return (uint8_t)((int8_t)half_db_steps);
}

static HAL_StatusTypeDef dac3100_set_master_volume_db(int32_t vol_db)
{
  HAL_StatusTypeDef st;
  uint8_t reg_value;

  reg_value = dac3100_volume_db_to_reg(vol_db);

  st = dac3100_write_reg_direct(0, DAC3100_DAC_LEFT_VOL_REG, reg_value);
  if (st != HAL_OK) {
    return st;
  }

  st = dac3100_write_reg_direct(0, DAC3100_DAC_RIGHT_VOL_REG, reg_value);
  return st;
}

static uint8_t ui_menu_count(uint8_t i)
{
  uint8_t count = 1U;

  if (g_ui[i].selected1 != UI_PARAM_NONE) {
    count++;
  }

  if (g_ui[i].selected2 != UI_PARAM_NONE) {
    count++;
  }

  return count;
}

static ui_param_id_t ui_menu_param_at(uint8_t i, uint8_t index)
{
  if (index == 0U) {
    return g_ui[i].selected0;
  }

  if ((index == 1U) && (g_ui[i].selected1 != UI_PARAM_NONE)) {
    return g_ui[i].selected1;
  }

  if ((index == 2U) && (g_ui[i].selected2 != UI_PARAM_NONE)) {
    return g_ui[i].selected2;
  }

  return g_ui[i].selected0;
}

static int8_t ui_menu_index_of(uint8_t i, ui_param_id_t p)
{
  uint8_t count;
  uint8_t index;

  count = ui_menu_count(i);

  for (index = 0U; index < count; index++) {
    if (ui_menu_param_at(i, index) == p) {
      return (int8_t)index;
    }
  }

  return 0;
}

static void ui_menu_move_cursor(uint8_t i, int32_t step_delta)
{
  uint8_t count;
  int8_t index;

  count = ui_menu_count(i);

  if (count <= 1U) {
    g_ui[i].cursor = g_ui[i].selected0;
    return;
  }

  index = ui_menu_index_of(i, g_ui[i].cursor);

  while (step_delta > 0) {
    index++;

    if (index >= (int8_t)count) {
      index = 0;
    }

    step_delta--;
  }

  while (step_delta < 0) {
    index--;

    if (index < 0) {
      index = (int8_t)(count - 1U);
    }

    step_delta++;
  }

  g_ui[i].cursor = ui_menu_param_at(i, (uint8_t)index);
}

/* LCD helpers */
static void lcd_showAdjust(void)
{
  char line0[17];
  char line1[17];
  ui_param_id_t p;

  p = g_ui[g_active_ui].active_param;

  snprintf(line0, sizeof(line0), "ENC%u:%s",
           (unsigned)(g_active_ui + 1), g_ds[p].name);

  if (p == UI_PARAM_COMP_RATIO) {
    snprintf(line1, sizeof(line1), "%ld:1",
             (long)g_ds[p].ui_value);
  } else {
    snprintf(line1, sizeof(line1), "%ld %s",
             (long)g_ds[p].ui_value, g_ds[p].unit);
  }

  lcd_clearLine(0);
  lcd_clearLine(1);
  lcd_put_cur(0, 0);
  lcd_send_string(line0);
  lcd_put_cur(1, 0);
  lcd_send_string(line1);
}

static void lcd_showMenu(void)
{
  char line0[17];
  char line1[17];
  ui_param_id_t p0;
  ui_param_id_t p1;
  ui_param_id_t p2;
  uint8_t count;

  p0 = g_ui[g_active_ui].selected0;
  p1 = g_ui[g_active_ui].selected1;
  p2 = g_ui[g_active_ui].selected2;
  count = ui_menu_count(g_active_ui);

  if (count == 1U) {
    snprintf(line0, sizeof(line0), "ENC%u MENU",
             (unsigned)(g_active_ui + 1U));
    snprintf(line1, sizeof(line1), ">%s", g_ds[p0].name);
  } else if (count == 2U) {
    if (g_ui[g_active_ui].cursor == p0) {
      snprintf(line0, sizeof(line0), ">%s", g_ds[p0].name);
      snprintf(line1, sizeof(line1), " %s", g_ds[p1].name);
    } else {
      snprintf(line0, sizeof(line0), " %s", g_ds[p0].name);
      snprintf(line1, sizeof(line1), ">%s", g_ds[p1].name);
    }
  } else {
    snprintf(line0, sizeof(line0), "ENC%u MENU",
             (unsigned)(g_active_ui + 1U));

    if (g_ui[g_active_ui].cursor == p0) {
      snprintf(line1, sizeof(line1), ">%s %s %s",
               g_ds[p0].name, g_ds[p1].name, g_ds[p2].name);
    } else if (g_ui[g_active_ui].cursor == p1) {
      snprintf(line1, sizeof(line1), " %s>%s %s",
               g_ds[p0].name, g_ds[p1].name, g_ds[p2].name);
    } else {
      snprintf(line1, sizeof(line1), " %s %s>%s",
               g_ds[p0].name, g_ds[p1].name, g_ds[p2].name);
    }
  }

  lcd_clearLine(0);
  lcd_clearLine(1);
  lcd_put_cur(0, 0);
  lcd_send_string(line0);
  lcd_put_cur(1, 0);
  lcd_send_string(line1);
}

static void ui_lcd_task(void)
{
  uint32_t now = HAL_GetTick();

  if (!g_ui_dirty) {
    return;
  }

  if ((now - g_last_lcd_ms) < 40) {
    return;
  }

  g_last_lcd_ms = now;
  g_ui_dirty = 0;

  if (g_ui[g_active_ui].mode == UI_MODE_FUNCTION) {
    lcd_showMenu();
  } else {
    lcd_showAdjust();
  }
}

static void ui_handle_button(uint8_t i)
{
  g_active_ui = i;

  if (g_ui[i].mode == UI_MODE_ADJUST) {
    g_ui[i].mode = UI_MODE_FUNCTION;
    g_ui[i].cursor = g_ui[i].active_param;
    g_ui[i].last_rawVal = __HAL_TIM_GET_COUNTER(g_ui[i].htim);
    g_ui[i].accum_counts = 0;
    g_ui_dirty = 1;
  } else {
    g_ui[i].active_param = g_ui[i].cursor;
    g_ui[i].mode = UI_MODE_ADJUST;
    g_ui[i].last_rawVal = __HAL_TIM_GET_COUNTER(g_ui[i].htim);
    g_ui[i].accum_counts = 0;
    g_ui_dirty = 1;
  }
}

static void ui_process_encoder(uint8_t i)
{
  uint32_t rawVal;
  int32_t delta_counts;
  int32_t step_delta;
  ui_param_id_t p;

  rawVal = __HAL_TIM_GET_COUNTER(g_ui[i].htim);
  delta_counts = (int32_t)((int32_t)rawVal - (int32_t)g_ui[i].last_rawVal);
  g_ui[i].last_rawVal = rawVal;

  g_ui[i].accum_counts += delta_counts;
  step_delta = g_ui[i].accum_counts / ENC_COUNTS_PER_STEP;
  g_ui[i].accum_counts -= step_delta * ENC_COUNTS_PER_STEP;

  if (step_delta == 0) {
    return;
  }

  g_active_ui = i;

  if (g_ui[i].mode == UI_MODE_FUNCTION) {
    ui_menu_move_cursor(i, step_delta);
    g_ui_dirty = 1;
  } else {
    p = g_ui[i].active_param;

    g_ds[p].ui_value += step_delta * g_ds[p].ui_step;
    g_ds[p].ui_value = ui_clamp_i32(g_ds[p].ui_value,
                                    g_ds[p].ui_min,
                                    g_ds[p].ui_max);

    dsp_apply_ui_values();
    g_ui_dirty = 1;
  }
}

static void ui_init_all(void)
{
  uint8_t i;

  for (i = 0; i < 4; i++) {
    g_ui[i].last_rawVal = __HAL_TIM_GET_COUNTER(g_ui[i].htim);
    g_ui[i].accum_counts = 0;
    g_ui[i].btn_armed = 1;
    g_ui[i].btn_event = 0;
    g_ui[i].last_btn_ms = 0;
  }

  g_active_ui = 0;
  lcd_showAdjust();
  dsp_apply_ui_values();
}


static void ui_task(void)
{
  uint8_t i;

  for (i = 0; i < 4; i++) {
    if ((HAL_GPIO_ReadPin(g_ui[i].sw_port, g_ui[i].sw_pin) == GPIO_PIN_SET) &&
        (g_ui[i].btn_armed == 0)) {
      g_ui[i].btn_armed = 1;
    }
  }

  for (i = 0; i < 4; i++) {
    if (g_ui[i].btn_event) {
      g_ui[i].btn_event = 0;
      ui_handle_button(i);
    }
  }

  ui_process_encoder(0);
  ui_process_encoder(1);
  ui_process_encoder(2);
  ui_process_encoder(3);

  ui_lcd_task();
}

/* Int to float and float to int */
float int16_to_float(int16_t x)
{
    return (float)x / 32768.0f;
}

int16_t float_to_int16(float x)
{
    if (x >= 1.0f)  return 32767;
    if (x <= -1.0f) return -32768;
    return (int16_t)(x * 32768.0f);
}




// Runtime CMSIS coefficients for the 3-stage EQ cascade.
// Initialised as three identity sections so startup is safe before UI applies values.
static float32_t g_eq_coeffs_cmsis[5 * EQ_STAGES] = {
  1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

typedef struct {
  arm_biquad_cascade_df2T_instance_f32 cascade;
  float32_t state[2 * EQ_STAGES];
} eq_3band_t;

static eq_3band_t g_eqL;
static eq_3band_t g_eqR;

static void eq_3band_init(eq_3band_t *eq)
{
  memset((*eq).state, 0, sizeof((*eq).state));
  arm_biquad_cascade_df2T_init_f32(&(*eq).cascade, EQ_STAGES, g_eq_coeffs_cmsis, (*eq).state);
}







/* DMA SECTION */

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	inBufferPtr = &adcData[0];
	outBufferPtr = &dacData[0];

	dataReady=1;

}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	inBufferPtr = &adcData[DMA_BUFFER_SIZE/2];
	outBufferPtr = &dacData[DMA_BUFFER_SIZE/2];

	dataReady=1;

}

typedef struct {
  float32_t *buf;     // stores v[]
  uint32_t   len;     // delay length in samples (k)
  uint32_t   idx;     // circular index
  float32_t  g;       // feedbackGain
  float32_t  m;       // mixWet
} comb_delay_f32_t;

static inline float32_t clamp_f32(float32_t x, float32_t lo, float32_t hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void comb_delay_init(comb_delay_f32_t *s, float32_t *buffer,
                            uint32_t delaySamples, float32_t feedbackGain, float32_t mixWet)
{
  (*s).buf = buffer;
  (*s).len = (delaySamples < 1) ? 1 : delaySamples;
  (*s).idx = 0;
  (*s).g   = feedbackGain;
  (*s).m   = mixWet;

  for (uint32_t i = 0; i < (*s).len; i++) {
    (*s).buf[i] = 0.0f;
  }
}

static inline float32_t comb_delay_process_1sample(comb_delay_f32_t *s, float32_t x)
{
  float32_t d = (*s).buf[(*s).idx];     // d[n] = v[n-k]
  float32_t v = x + (*s).g * d;         // v[n] = x[n] + g*d[n]

  // Prevent runaway in feedback
  v = clamp_f32(v, -1.2f, 1.2f);

  (*s).buf[(*s).idx] = v;

  (*s).idx = (*s).idx + 1;
  if ((*s).idx >= (*s).len) {
    (*s).idx = 0;
  }

  return ((*s).m * d) + ((1.0f - (*s).m) * x);
}

//stereo delays
static float32_t g_dlyBufL[DELAY_MAX_SAMPLES];
static float32_t g_dlyBufR[DELAY_MAX_SAMPLES];
static comb_delay_f32_t g_dlyL;
static comb_delay_f32_t g_dlyR;

static uint8_t g_delay_enabled = 1;   // set 0 to bypass


/*
 * COMPRESSOR SECTION
 */

typedef enum
{
  COMP_STATE_NO_OPERATION = 0,
  COMP_STATE_ATTACK,
  COMP_STATE_GAIN_REDUCTION,
  COMP_STATE_RELEASE
} comp_state_t;

typedef struct
{
  comp_state_t state;

  float32_t fs;
  float32_t threshold;
  float32_t end_gain;

  float32_t gain;
  float32_t target_gain;
  float32_t gain_step;

  uint32_t attack_samples;
  uint32_t hold_samples;
  uint32_t release_samples;
  uint32_t timeout;
} compressor_t;




//compressor statics
static compressor_t g_compL;
static compressor_t g_compR;
static uint8_t g_comp_enabled = 1;

static void compressor_start_attack(compressor_t *c, float32_t target)
{
  (*c).state = COMP_STATE_ATTACK;
  (*c).target_gain = target;
  (*c).timeout = (*c).attack_samples;

  if ((*c).attack_samples == 0)
  {
    (*c).gain = (*c).target_gain;
    (*c).gain_step = 0.0f;
    (*c).state = COMP_STATE_GAIN_REDUCTION;
    (*c).timeout = (*c).hold_samples;
  }
  else
  {
    (*c).gain_step = ((*c).gain - (*c).target_gain) / (float32_t)(*c).attack_samples;

    if ((*c).gain_step < 0.0f)
    {
      (*c).gain_step = 0.0f;
    }
  }
}

static void compressor_start_release(compressor_t *c)
{
  (*c).state = COMP_STATE_RELEASE;
  (*c).target_gain = 1.0f;
  (*c).timeout = (*c).release_samples;

  if ((*c).release_samples == 0)
  {
    (*c).gain = 1.0f;
    (*c).gain_step = 0.0f;
    (*c).state = COMP_STATE_NO_OPERATION;
    (*c).timeout = 0;
  }
  else
  {
    (*c).gain_step = (1.0f - (*c).gain) / (float32_t)(*c).release_samples;

    if ((*c).gain_step < 0.0f)
    {
      (*c).gain_step = 0.0f;
    }
  }
}

static void compressor_init(compressor_t *c, float32_t fs)
{
  (*c).state = COMP_STATE_NO_OPERATION;
  (*c).fs = fs;

  (*c).threshold = COMP_THRESHOLD;
  (*c).end_gain = COMP_END_GAIN;

  (*c).gain = 1.0f;
  (*c).target_gain = 1.0f;
  (*c).gain_step = 0.0f;

  (*c).attack_samples  = (uint32_t)((COMP_ATTACK_MS  * fs) / 1000.0f);
  (*c).hold_samples    = (uint32_t)((COMP_HOLD_MS    * fs) / 1000.0f);
  (*c).release_samples = (uint32_t)((COMP_RELEASE_MS * fs) / 1000.0f);

  (*c).timeout = 0;
}

static float32_t compressor_process_sample(compressor_t *c, float32_t x)
{
  float32_t level_abs = fabsf(x);

  switch ((*c).state)
  {
    case COMP_STATE_NO_OPERATION:
      (*c).gain = 1.0f;

      if ((level_abs > (*c).threshold) && ((*c).gain > (*c).end_gain))
      {
        compressor_start_attack(c, (*c).end_gain);
      }
      break;

    case COMP_STATE_ATTACK:
      if ((*c).gain > (*c).target_gain)
      {
        (*c).gain -= (*c).gain_step;

        if ((*c).gain < (*c).target_gain)
        {
          (*c).gain = (*c).target_gain;
        }
      }

      if ((*c).timeout > 0)
      {
        (*c).timeout--;
      }

      if (((*c).timeout == 0) || ((*c).gain <= (*c).target_gain))
      {
        (*c).state = COMP_STATE_GAIN_REDUCTION;
        (*c).timeout = (*c).hold_samples;
      }
      break;

    case COMP_STATE_GAIN_REDUCTION:
      (*c).gain = (*c).target_gain;

      if (level_abs > (*c).threshold)
      {
        (*c).timeout = (*c).hold_samples;
      }
      else
      {
        if ((*c).timeout > 0)
        {
          (*c).timeout--;
        }

        if ((*c).timeout == 0)
        {
          compressor_start_release(c);
        }
      }
      break;

    case COMP_STATE_RELEASE:
      if ((level_abs > (*c).threshold) && ((*c).gain > (*c).end_gain))
      {
        compressor_start_attack(c, (*c).end_gain);
      }
      else
      {
        (*c).gain += (*c).gain_step;

        if ((*c).gain > 1.0f)
        {
          (*c).gain = 1.0f;
        }

        if ((*c).timeout > 0)
        {
          (*c).timeout--;
        }

        if (((*c).timeout == 0) || ((*c).gain >= 1.0f))
        {
          (*c).state = COMP_STATE_NO_OPERATION;
          (*c).gain = 1.0f;
          (*c).target_gain = 1.0f;
          (*c).gain_step = 0.0f;
          (*c).timeout = 0;
        }
      }
      break;

    default:
      (*c).state = COMP_STATE_NO_OPERATION;
      (*c).gain = 1.0f;
      (*c).target_gain = 1.0f;
      (*c).gain_step = 0.0f;
      (*c).timeout = 0;
      break;
  }

  return x * (*c).gain;
}

/* ENCODER CONTROLLED DATA PROCESSING */

static void eq_write_biquad_cmsis(float32_t *dst,
                                  float32_t b0, float32_t b1, float32_t b2,
                                  float32_t a0, float32_t a1, float32_t a2)
{
  float32_t inv_a0 = 1.0f / a0;

  dst[0] = b0 * inv_a0;
  dst[1] = b1 * inv_a0;
  dst[2] = b2 * inv_a0;
  dst[3] = -(a1 * inv_a0);
  dst[4] = -(a2 * inv_a0);
}

static void eq_design_peaking(float32_t *dst, float32_t f0, float32_t gain_dB,
                              float32_t Fs, float32_t Q)
{
  float32_t A;
  float32_t w0;
  float32_t alpha;
  float32_t cw0;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t a0;
  float32_t a1;
  float32_t a2;

  A = powf(10.0f, gain_dB / 40.0f);
  w0 = 2.0f * EQ_PI * f0 / Fs;
  alpha = sinf(w0) / (2.0f * Q);
  cw0 = cosf(w0);

  b0 = 1.0f + alpha * A;
  b1 = -2.0f * cw0;
  b2 = 1.0f - alpha * A;
  a0 = 1.0f + alpha / A;
  a1 = -2.0f * cw0;
  a2 = 1.0f - alpha / A;

  eq_write_biquad_cmsis(dst, b0, b1, b2, a0, a1, a2);
}

static void eq_design_low_shelf(float32_t *dst, float32_t f0, float32_t gain_dB,
                                float32_t Fs, float32_t S)
{
  float32_t A;
  float32_t w0;
  float32_t cw0;
  float32_t sw0;
  float32_t alpha;
  float32_t beta;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t a0;
  float32_t a1;
  float32_t a2;

  A = powf(10.0f, gain_dB / 40.0f);
  w0 = 2.0f * EQ_PI * f0 / Fs;
  cw0 = cosf(w0);
  sw0 = sinf(w0);

  alpha = (sw0 * 0.5f) * sqrtf((A + (1.0f / A)) * ((1.0f / S) - 1.0f) + 2.0f);
  beta = 2.0f * sqrtf(A) * alpha;

  b0 = A * ((A + 1.0f) - ((A - 1.0f) * cw0) + beta);
  b1 = 2.0f * A * ((A - 1.0f) - ((A + 1.0f) * cw0));
  b2 = A * ((A + 1.0f) - ((A - 1.0f) * cw0) - beta);
  a0 = (A + 1.0f) + ((A - 1.0f) * cw0) + beta;
  a1 = -2.0f * ((A - 1.0f) + ((A + 1.0f) * cw0));
  a2 = (A + 1.0f) + ((A - 1.0f) * cw0) - beta;

  eq_write_biquad_cmsis(dst, b0, b1, b2, a0, a1, a2);
}

static void eq_design_high_shelf(float32_t *dst, float32_t f0, float32_t gain_dB,
                                 float32_t Fs, float32_t S)
{
  float32_t A;
  float32_t w0;
  float32_t cw0;
  float32_t sw0;
  float32_t alpha;
  float32_t beta;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t a0;
  float32_t a1;
  float32_t a2;

  A = powf(10.0f, gain_dB / 40.0f);
  w0 = 2.0f * EQ_PI * f0 / Fs;
  cw0 = cosf(w0);
  sw0 = sinf(w0);

  alpha = (sw0 * 0.5f) * sqrtf((A + (1.0f / A)) * ((1.0f / S) - 1.0f) + 2.0f);
  beta = 2.0f * sqrtf(A) * alpha;

  b0 = A * ((A + 1.0f) + ((A - 1.0f) * cw0) + beta);
  b1 = -2.0f * A * ((A - 1.0f) + ((A + 1.0f) * cw0));
  b2 = A * ((A + 1.0f) + ((A - 1.0f) * cw0) - beta);
  a0 = (A + 1.0f) - ((A - 1.0f) * cw0) + beta;
  a1 = 2.0f * ((A - 1.0f) - ((A + 1.0f) * cw0));
  a2 = (A + 1.0f) - ((A - 1.0f) * cw0) - beta;

  eq_write_biquad_cmsis(dst, b0, b1, b2, a0, a1, a2);
}

static void eq_update_coefficients_from_ui(int32_t bass_db, int32_t mid_db, int32_t treble_db)
{
  eq_design_low_shelf(&g_eq_coeffs_cmsis[0],
                      EQ_BASS_FREQ_HZ,
                      (float32_t)bass_db,
                      (float32_t)FS_HZ,
                      EQ_SHELF_SLOPE);

  eq_design_peaking(&g_eq_coeffs_cmsis[5],
                    EQ_MID_FREQ_HZ,
                    (float32_t)mid_db,
                    (float32_t)FS_HZ,
                    EQ_MID_Q);

  eq_design_high_shelf(&g_eq_coeffs_cmsis[10],
                       EQ_TREBLE_FREQ_HZ,
                       (float32_t)treble_db,
                       (float32_t)FS_HZ,
                       EQ_SHELF_SLOPE);
}

static inline uint32_t ms_to_samples_u32(uint32_t ms)
{
  uint32_t s;

  s = (uint32_t)(((uint64_t)ms * (uint64_t)FS_HZ) / 1000ULL);

  if (s > DELAY_MAX_SAMPLES) {
    s = DELAY_MAX_SAMPLES;
  }

  if (s == 0U) {
    s = 1U;
  }

  return s;
}

static void compressor_set_times_ms(compressor_t *c,
                                    uint32_t attack_ms,
                                    uint32_t hold_ms,
                                    uint32_t release_ms)
{
  (*c).attack_samples  = (uint32_t)(((float32_t)attack_ms  * (*c).fs) / 1000.0f);
  (*c).hold_samples    = (uint32_t)(((float32_t)hold_ms    * (*c).fs) / 1000.0f);
  (*c).release_samples = (uint32_t)(((float32_t)release_ms * (*c).fs) / 1000.0f);
}

static void compressor_ratio_to_params(uint32_t ratio,
                                       float32_t *threshold,
                                       float32_t *end_gain)
{
  if (ratio >= 4U) {
    *threshold = COMP_RATIO_4_TO_1_THRESHOLD;
    *end_gain  = COMP_RATIO_4_TO_1_END_GAIN;
  } else {
    *threshold = COMP_RATIO_2_TO_1_THRESHOLD;
    *end_gain  = COMP_RATIO_2_TO_1_END_GAIN;
  }
}

static void compressor_set_ratio_preset(compressor_t *c, uint32_t ratio)
{
  float32_t threshold;
  float32_t end_gain;

  compressor_ratio_to_params(ratio, &threshold, &end_gain);

  (*c).threshold = threshold;
  (*c).end_gain = end_gain;

  /* Keep the state-machine behaviour unchanged.
   * If the compressor is already attacking or holding gain reduction,
   * the new ratio simply changes the target end-gain.
   */
  if (((*c).state == COMP_STATE_ATTACK) ||
      ((*c).state == COMP_STATE_GAIN_REDUCTION)) {
    (*c).target_gain = end_gain;
  }
}

static inline float32_t eq_process_3band_1sample(eq_3band_t *eq, float32_t x)
{
  float32_t y;

  arm_biquad_cascade_df2T_f32(&(*eq).cascade, &x, &y, 1);

  //return clamp_f32(y, -1.0f, 1.0f);
  return y; //no clamp
}
static void dsp_apply_ui_values(void)
{
  int32_t bass_db;
  int32_t mid_db;
  int32_t treble_db;
  uint8_t eq_changed;
  uint32_t attack_ms;
  uint32_t hold_ms;
  uint32_t release_ms;
  uint32_t delay_ms;
  uint32_t delay_fb_pct;
  uint32_t delay_len;
  uint32_t comp_ratio;
  int32_t master_vol_db;

  bass_db = ui_clamp_i32(g_ds[UI_PARAM_EQ_BASS].ui_value,
                         g_ds[UI_PARAM_EQ_BASS].ui_min,
                         g_ds[UI_PARAM_EQ_BASS].ui_max);
  mid_db = ui_clamp_i32(g_ds[UI_PARAM_EQ_MID].ui_value,
                        g_ds[UI_PARAM_EQ_MID].ui_min,
                        g_ds[UI_PARAM_EQ_MID].ui_max);
  treble_db = ui_clamp_i32(g_ds[UI_PARAM_EQ_TREBLE].ui_value,
                           g_ds[UI_PARAM_EQ_TREBLE].ui_min,
                           g_ds[UI_PARAM_EQ_TREBLE].ui_max);

  g_ds[UI_PARAM_EQ_BASS].ui_value = bass_db;
  g_ds[UI_PARAM_EQ_MID].ui_value = mid_db;
  g_ds[UI_PARAM_EQ_TREBLE].ui_value = treble_db;

  eq_changed = 0U;

  if ((bass_db != g_eq_bass_db_cache) ||
      (mid_db != g_eq_mid_db_cache) ||
      (treble_db != g_eq_treble_db_cache)) {
    g_eq_bass_db_cache = bass_db;
    g_eq_mid_db_cache = mid_db;
    g_eq_treble_db_cache = treble_db;
    eq_changed = 1U;
  }

  if (eq_changed != 0U) {
    eq_update_coefficients_from_ui(bass_db, mid_db, treble_db);

    /* Rebind both channels to the updated coefficients and clear old IIR state. */
    eq_3band_init(&g_eqL);
    eq_3band_init(&g_eqR);
  }

  attack_ms = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_COMP_ATTACK].ui_value,
                                     g_ds[UI_PARAM_COMP_ATTACK].ui_min,
                                     g_ds[UI_PARAM_COMP_ATTACK].ui_max);
  hold_ms = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_COMP_HOLD].ui_value,
                                   g_ds[UI_PARAM_COMP_HOLD].ui_min,
                                   g_ds[UI_PARAM_COMP_HOLD].ui_max);
  release_ms = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_COMP_RELEASE].ui_value,
                                      g_ds[UI_PARAM_COMP_RELEASE].ui_min,
                                      g_ds[UI_PARAM_COMP_RELEASE].ui_max);

  g_ds[UI_PARAM_COMP_ATTACK].ui_value = (int32_t)attack_ms;
  g_ds[UI_PARAM_COMP_HOLD].ui_value = (int32_t)hold_ms;
  g_ds[UI_PARAM_COMP_RELEASE].ui_value = (int32_t)release_ms;

  compressor_set_times_ms(&g_compL, attack_ms, hold_ms, release_ms);
  compressor_set_times_ms(&g_compR, attack_ms, hold_ms, release_ms);

  comp_ratio = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_COMP_RATIO].ui_value,
                                      g_ds[UI_PARAM_COMP_RATIO].ui_min,
                                      g_ds[UI_PARAM_COMP_RATIO].ui_max);

  if (comp_ratio < 3U) {
    comp_ratio = 2U;
  } else {
    comp_ratio = 4U;
  }

  g_ds[UI_PARAM_COMP_RATIO].ui_value = (int32_t)comp_ratio;

  if ((int32_t)comp_ratio != g_comp_ratio_cache) {
    g_comp_ratio_cache = (int32_t)comp_ratio;
    compressor_set_ratio_preset(&g_compL, comp_ratio);
    compressor_set_ratio_preset(&g_compR, comp_ratio);
  }

  g_comp_enabled = 1U;

  delay_ms = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_DELAY_MS].ui_value,
                                    g_ds[UI_PARAM_DELAY_MS].ui_min,
                                    g_ds[UI_PARAM_DELAY_MS].ui_max);
  delay_fb_pct = (uint32_t)ui_clamp_i32(g_ds[UI_PARAM_DELAY_FB].ui_value,
                                        g_ds[UI_PARAM_DELAY_FB].ui_min,
                                        g_ds[UI_PARAM_DELAY_FB].ui_max);

  g_ds[UI_PARAM_DELAY_MS].ui_value = (int32_t)delay_ms;
  g_ds[UI_PARAM_DELAY_FB].ui_value = (int32_t)delay_fb_pct;

  if (delay_ms == 0U) {
    g_delay_enabled = 0U;
    g_dlyL.idx = 0U;
    g_dlyR.idx = 0U;
    memset(g_dlyBufL, 0, sizeof(g_dlyBufL));
    memset(g_dlyBufR, 0, sizeof(g_dlyBufR));
  } else {
    g_delay_enabled = 1U;
    delay_len = ms_to_samples_u32(delay_ms);

    if ((g_dlyL.len != delay_len) || (g_dlyR.len != delay_len)) {
      g_dlyL.len = delay_len;
      g_dlyR.len = delay_len;
      g_dlyL.idx = 0U;
      g_dlyR.idx = 0U;
      memset(g_dlyBufL, 0, sizeof(g_dlyBufL));
      memset(g_dlyBufR, 0, sizeof(g_dlyBufR));
    }
  }

  g_dlyL.g = clamp_f32(((float32_t)delay_fb_pct) / 100.0f, 0.0f, 0.95f);
  g_dlyR.g = g_dlyL.g;

  g_dlyL.m = g_delay_mix;
  g_dlyR.m = g_delay_mix;

  master_vol_db = ui_clamp_i32(g_ds[UI_PARAM_MASTER_VOL].ui_value,
                               g_ds[UI_PARAM_MASTER_VOL].ui_min,
                               g_ds[UI_PARAM_MASTER_VOL].ui_max);

  g_ds[UI_PARAM_MASTER_VOL].ui_value = master_vol_db;

  if (master_vol_db != g_master_vol_db_cache) {
    g_master_vol_db_cache = master_vol_db;
    g_last_dac_vol_status = dac3100_set_master_volume_db(master_vol_db);
  }
}

/* PROCESS LEFT AND RIGHT */
 void dataProcess(void)
	{
	  float32_t leftIn;
	  float32_t leftOut;
	  float32_t rightIn;
	  float32_t rightOut;

	  for (uint32_t x = 0; x < (DMA_BUFFER_SIZE / 2); x += 2)
	  {
	    leftIn  = int16_to_float(inBufferPtr[x]);
	    rightIn = int16_to_float(inBufferPtr[x + 1]);

//	    		leftOut  = leftIn;
//	    	    rightOut = rightIn;//Testing if DSP chain itself is causing distortion problem.

	    leftOut  =  eq_process_3band_1sample(&g_eqL, leftIn);
	     rightOut = eq_process_3band_1sample(&g_eqR, rightIn);

	    if (g_comp_enabled) {
	      leftOut  = compressor_process_sample(&g_compL, leftOut);
	      rightOut = compressor_process_sample(&g_compR, rightOut);
	    }

	    if (g_delay_enabled) {
	      leftOut  = comb_delay_process_1sample(&g_dlyL, leftOut);
	      rightOut = comb_delay_process_1sample(&g_dlyR, rightOut);
	    }
//	    leftOut  = leftIn;
//	    rightOut = rightIn;//Testing if DSP chain itself is causing distortion problem.



	    outBufferPtr[x]     = float_to_int16(clamp_f32(leftOut,  -1.0f, 1.0f));
	    outBufferPtr[x + 1] = float_to_int16(clamp_f32(rightOut, -1.0f, 1.0f));
	  }

	  dataReady = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2S2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


  tlv320dac3100_cfg_t cfg = {
    .i2c_addr_7bit = DAC3100_I2C_ADDR_7BIT, // common default for DAC3100 boards (change if yours differs)
    .pll_input     = TLV320_PLL_INPUT_MCLK,// since you enabled MCLK output in I2S init
    .fs_hz         = 48000,
    .hp_vol_reg    = 0x06                  // example; adjust later
  };

  if (TLV320DAC3100_Init(&g_dac, &hi2c1, &cfg) != HAL_OK) {
    Error_Handler();
  }

  eq_3band_init(&g_eqL);
  eq_3band_init(&g_eqR);

  compressor_init(&g_compL, 48000.0f);
  compressor_init(&g_compR, 48000.0f);

  comb_delay_init(&g_dlyL, g_dlyBufL, DELAY_MAX_SAMPLES, 0.5f, 0.5f);
  comb_delay_init(&g_dlyR, g_dlyBufR, DELAY_MAX_SAMPLES, 0.5f, 0.5f);

  dataReady = 0;
  HAL_StatusTypeDef status = HAL_I2SEx_TransmitReceive_DMA(&hi2s2,(uint16_t *)dacData,(uint16_t *)adcData, DMA_BUFFER_SIZE);

  lcd_init();
  ui_init_all();
  //lcd_send_string("ENC VAL:");
  //HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (dataReady)
    {
      dataReady = 0;
      dataProcess();
    }

    ui_task();
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now;
  uint8_t i;

  now = HAL_GetTick();

  for (i = 0; i < 4; i++) {
    if ((GPIO_Pin == g_ui[i].sw_pin) && g_ui[i].btn_armed) {
      if ((now - g_ui[i].last_btn_ms) >= BTN_DEBOUNCE_MS) {
        g_ui[i].last_btn_ms = now;
        g_ui[i].btn_event = 1;
        g_ui[i].btn_armed = 0;
      }
    }
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
  __disable_irq();
  while (1)
  {


  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
