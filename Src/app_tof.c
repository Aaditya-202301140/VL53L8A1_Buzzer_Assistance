/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.3 instances.
  *
  * MODIFIED: Full live UART integration with Python GUI
  * TX format: DIST:%lu,ZONE:%s,ALERT:%s,STATUS:RUNNING\r\n
  * RX commands:
  *   THR:<danger>,<warning>,<caution>        -> live threshold update
  *   SENSOR:<res>,<timing>,<freq>             -> live sensor reconfigure
  *   CMD:START / CMD:STOP / CMD:BUZZ_TEST     -> runtime control
  *
  * FIX 1 (kept from previous version): Thresholds are runtime variables
  *        (not macros) — buzzer logic and Send_GUI_Data both read live
  *        values, never stale.
  * FIX 2 (kept from previous version): Sensor resolution/timing/frequency
  *        changes are applied via Stop -> ConfigProfile -> Start whenever
  *        a SENSOR: command arrives.
  *
  * FIX 3 (NEW): UART RX switched from polled com_has_data()/single-byte
  *        reads to INTERRUPT-DRIVEN reception (HAL_UART_Receive_IT +
  *        HAL_UART_RxCpltCallback). This was the actual root cause of
  *        "threshold changes don't take effect": the old polling loop
  *        only checked for a new byte once per while(1) iteration, and
  *        that same iteration also runs GetDistance() + a UART TX with
  *        a 100ms timeout. STM32 USART RX has no hardware FIFO, so any
  *        byte of "THR:700,1700,3200\r\n" that arrived while the loop was
  *        busy transmitting got silently dropped (overrun). Long commands
  *        therefore arrived corrupted/truncated and either failed the
  *        strtok parse or got silently rejected, while DIST: streaming
  *        kept working fine (TX is unaffected by RX overrun) — exactly
  *        matching the reported symptom.
  *
  * FIX 4 (NEW): RX error flags (ORE/NE/FE) are explicitly cleared in
  *        HAL_UART_ErrorCallback so a single overrun early on can never
  *        permanently wedge reception for the rest of the session.
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "53l8a1_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "stm32f4xx_nucleo.h"

extern TIM_HandleTypeDef htim2;

/* ── UART for Python GUI ── */
static char gui_buf[128];

/* ── UART RX line buffer (for THR:/SENSOR:/CMD: commands) ──
 * FIX 3: now filled by the interrupt callback, one byte at a time,
 * completely decoupled from whatever the main loop is doing. */
#define RX_LINE_MAX  64
static char    rx_line[RX_LINE_MAX];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_line_ready = 0;   /* set by ISR, consumed by main loop */
static uint8_t  rx_byte;                     /* single-byte landing zone for IT receive */

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET_DEFAULT      (30U)
#define RANGING_FREQUENCY_DEFAULT  (10U)
#define POLLING_PERIOD             (1)

#define DIST_NEAR_DEFAULT   500U
#define DIST_FAR_1_DEFAULT  1500U
#define DIST_FAR_2_DEFAULT  2500U

/* ── Runtime threshold variables (FIX 1 — unchanged from previous version) ── */
static volatile uint32_t dist_near  = DIST_NEAR_DEFAULT;
static volatile uint32_t dist_far_1 = DIST_FAR_1_DEFAULT;
static volatile uint32_t dist_far_2 = DIST_FAR_2_DEFAULT;

/* ── Runtime sensor config state (FIX 2 — unchanged from previous version) ── */
static volatile uint8_t  sensor_reconfig_pending = 0;
static uint32_t pending_profile      = RS_PROFILE_8x8_CONTINUOUS;
static uint32_t pending_timing_ms    = 20;
static uint32_t pending_freq_hz      = 10;

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t  Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static RANGING_SENSOR_Result_t        Result;
static int32_t status = 0;
static volatile uint8_t PushButtonDetected = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static void toggle_resolution(void);
static void toggle_signal_and_ambient(void);
static void clear_screen(void);
static void display_commands_banner(void);
static void handle_cmd(uint8_t cmd);
static uint8_t get_key(void);
static uint32_t com_has_data(void);

static void Send_GUI_Data(uint32_t dist_mm);
static void Process_GUI_Line(const char *line);   /* FIX 3: renamed/repurposed */
static void Apply_Thresholds(const char *line);
static void Parse_Sensor_Config(const char *line);
static void Apply_Pending_Sensor_Config(void);
static void UART_StartReceive_IT(void);            /* FIX 3: new */

/* =========================================================================
   MX_TOF_Init
   ========================================================================= */
void MX_TOF_Init(void)
{
  MX_53L8A1_SimpleRanging_Init();
  UART_StartReceive_IT();   /* FIX 3: arm interrupt-driven RX right after init */
}

/* =========================================================================
   MX_TOF_Process
   ========================================================================= */
void MX_TOF_Process(void)
{
  MX_53L8A1_SimpleRanging_Process();
}

/* =========================================================================
   Init
   ========================================================================= */
static void MX_53L8A1_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Sensor reset */
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

  printf("\033[2H\033[2J");
  printf("53L8A1 Simple Ranging demo application\n");
  printf("Sensor initialization...\n");

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L8A1_RANGING_SENSOR_Init failed\n");
    while (1);
  }
}

/* =========================================================================
   FIX 3 — Interrupt-driven UART RX
   =========================================================================
   Arms a single-byte interrupt receive. Every time one byte arrives, the
   HAL fires HAL_UART_RxCpltCallback below — REGARDLESS of what the main
   loop is doing (GetDistance, Send_GUI_Data, HAL_Delay, etc.). This makes
   byte loss essentially impossible under normal command rates, which is
   exactly what the old com_has_data()-polling approach could not guarantee.
   ========================================================================= */
static void UART_StartReceive_IT(void)
{
  /* FIX 5 (NEW): BSP_COM_Init() configures the USART2 peripheral itself,
   * but it never calls HAL_NVIC_EnableIRQ(USART2_IRQn) — it was written
   * for polling-mode use. Without this, HAL_UART_Receive_IT() arms the
   * peripheral's RXNEIE flag and a byte does land in the data register,
   * but the NVIC never wakes the CPU into USART2_IRQHandler(), so
   * HAL_UART_RxCpltCallback() is never invoked. This is the actual root
   * cause of THR:/SENSOR:/CMD: bytes being silently dropped while DIST:
   * streaming (pure TX) kept working fine.
   */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  rx_index      = 0;
  rx_line_ready = 0;
  HAL_UART_Receive_IT(&hcom_uart[COM1], &rx_byte, 1);
}

/* Called automatically by the HAL ISR whenever one byte has been received.
 * Must be re-armed every time, or RX silently stops after the first byte. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == hcom_uart[COM1].Instance)
  {
    char c = (char)rx_byte;

    /* Original single-key debug commands ('r'/'s'/'c') still work
       immediately, exactly as in the previous version, as long as they
       arrive at the very start of a line. */
    if (rx_index == 0 && (c == 'r' || c == 's' || c == 'c'))
    {
      handle_cmd((uint8_t)c);
    }
    else if (c == '\n' || c == '\r')
    {
      if (rx_index > 0)
      {
        rx_line[rx_index] = '\0';
        rx_line_ready = 1;   /* hand off to main loop — never parse inside an ISR */
      }
    }
    else if (rx_index < (RX_LINE_MAX - 1))
    {
      rx_line[rx_index++] = c;
    }
    else
    {
      rx_index = 0;   /* overflow guard — discard line */
    }

    /* Re-arm for the next byte — CRITICAL, otherwise RX stops after 1 byte */
    HAL_UART_Receive_IT(&hcom_uart[COM1], &rx_byte, 1);
  }
}

/* FIX 4 — clear RX error flags so one overrun can never permanently wedge
 * reception. Also re-arms RX, since a HAL error aborts the active request. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == hcom_uart[COM1].Instance)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    rx_index = 0;   /* discard whatever partial line we had */
    HAL_UART_Receive_IT(&hcom_uart[COM1], &rx_byte, 1);
  }
}

/* =========================================================================
   Process — main loop
   FIX 2: loop checks sensor_reconfig_pending on every iteration and
   applies a fresh Stop -> ConfigProfile -> Start cycle when a SENSOR:
   command has arrived.
   FIX 3: loop now checks rx_line_ready (set by the ISR) instead of
   polling com_has_data() / reading bytes one at a time itself.
   ========================================================================= */
static void MX_53L8A1_SimpleRanging_Process(void)
{
  uint32_t Id;

  VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
  VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);

  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget   = TIMING_BUDGET_DEFAULT;
  Profile.Frequency      = RANGING_FREQUENCY_DEFAULT;
  Profile.EnableAmbient  = 0;
  Profile.EnableSignal   = 0;

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);

  status = VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER,
                                          RS_MODE_BLOCKING_CONTINUOUS);
  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L8A1_RANGING_SENSOR_Start failed\n");
    while (1);
  }

  while (1)
  {
    /* ── FIX 3: process a complete line received by the ISR, if any ── */
    if (rx_line_ready)
    {
      rx_line_ready = 0;
      Process_GUI_Line(rx_line);
      rx_index = 0;
    }

    /* ── FIX 2: apply any pending sensor reconfiguration ── */
    if (sensor_reconfig_pending)
    {
      Apply_Pending_Sensor_Config();
      sensor_reconfig_pending = 0;
    }

    /* polling mode */
    status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER,
                                                  &Result);
    if (status == BSP_ERROR_NONE)
    {
      /* ── Find nearest object across all zones ── */
      uint32_t min_distance = 9999;

      for (int z = 0; z < Result.NumberOfZones; z++)
      {
        if (Result.ZoneResult[z].NumberOfTargets > 0)
        {
          if (Result.ZoneResult[z].Distance[0] < min_distance)
          {
            min_distance = Result.ZoneResult[z].Distance[0];
          }
        }
      }

      /* ── Buzzer logic — reads live threshold variables (FIX 1) ── */
      static uint32_t last_toggle = 0;
      static uint8_t  buzzer_state = 0;

      if (min_distance < dist_near)
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
      }
      else if (min_distance < dist_far_1)
      {
        if (HAL_GetTick() - last_toggle >= 150)
        {
          last_toggle  = HAL_GetTick();
          buzzer_state = !buzzer_state;
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                                 buzzer_state ? 500 : 0);
        }
      }
      else if (min_distance < dist_far_2)
      {
        if (HAL_GetTick() - last_toggle >= 500)
        {
          last_toggle  = HAL_GetTick();
          buzzer_state = !buzzer_state;
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                                 buzzer_state ? 500 : 0);
        }
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      }

      /* ── Send data to Python GUI over UART ── */
      Send_GUI_Data(min_distance);

      /* ── Print to serial terminal (original) ── */
      print_result(&Result);
    }

    HAL_Delay(POLLING_PERIOD);
  }
}

/* =========================================================================
   Process_GUI_Line
   FIX 3: this is what Process_GUI_RX used to do, but now it runs on a
   COMPLETE, already-assembled line (handed off from the ISR via
   rx_line_ready), instead of trying to read+dispatch one byte per call
   while racing against blocking UART TX elsewhere in the loop.
   ========================================================================= */
static void Process_GUI_Line(const char *line)
{
  if (strncmp(line, "THR:", 4) == 0)
  {
    Apply_Thresholds(line + 4);
  }
  else if (strncmp(line, "SENSOR:", 7) == 0)
  {
    Parse_Sensor_Config(line + 7);
  }
  else if (strncmp(line, "CMD:START", 9) == 0)
  {
    printf("CMD: Start ranging requested\n");
  }
  else if (strncmp(line, "CMD:STOP", 8) == 0)
  {
    printf("CMD: Stop ranging requested\n");
  }
  else if (strncmp(line, "CMD:BUZZ_TEST", 13) == 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
    HAL_Delay(300);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  }
}

/* =========================================================================
   Apply_Thresholds — unchanged logic from previous version
   ========================================================================= */
static void Apply_Thresholds(const char *line)
{
  uint32_t d, w, c;
  char buf[48];
  char *tok;

  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  tok = strtok(buf, ",");
  if (tok == NULL) { printf("THR parse error\n"); return; }
  d = (uint32_t)strtoul(tok, NULL, 10);

  tok = strtok(NULL, ",");
  if (tok == NULL) { printf("THR parse error\n"); return; }
  w = (uint32_t)strtoul(tok, NULL, 10);

  tok = strtok(NULL, ",");
  if (tok == NULL) { printf("THR parse error\n"); return; }
  c = (uint32_t)strtoul(tok, NULL, 10);

  if (!(d < w && w < c))
  {
    printf("THR rejected (invalid order): D=%lu W=%lu C=%lu\n",
           (unsigned long)d, (unsigned long)w, (unsigned long)c);
    return;
  }

  dist_near  = d;
  dist_far_1 = w;
  dist_far_2 = c;

  printf("THR applied: Danger<%lu  Warning<%lu  Caution<%lu\n",
         (unsigned long)dist_near,
         (unsigned long)dist_far_1,
         (unsigned long)dist_far_2);

  /* Acknowledge back to the GUI so it can confirm the apply succeeded */
  char ack[64];
  int n = snprintf(ack, sizeof(ack), "ACK:THR,%lu,%lu,%lu\r\n",
                   (unsigned long)dist_near,
                   (unsigned long)dist_far_1,
                   (unsigned long)dist_far_2);
  HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)ack, n, 100);
}

/* =========================================================================
   Parse_Sensor_Config — unchanged logic from previous version
   ========================================================================= */
static void Parse_Sensor_Config(const char *line)
{
  char buf[48];
  char *tok;
  char res_tok[16] = {0};
  char timing_tok[16] = {0};
  char freq_tok[16] = {0};

  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  tok = strtok(buf, ",");
  if (tok != NULL) strncpy(res_tok, tok, sizeof(res_tok) - 1);

  tok = strtok(NULL, ",");
  if (tok != NULL) strncpy(timing_tok, tok, sizeof(timing_tok) - 1);

  tok = strtok(NULL, ",");
  if (tok != NULL) strncpy(freq_tok, tok, sizeof(freq_tok) - 1);

  /* ── Resolution: match leading digit ── */
  if (res_tok[0] == '4')
  {
    pending_profile = RS_PROFILE_4x4_CONTINUOUS;
  }
  else if (res_tok[0] == '8')
  {
    pending_profile = RS_PROFILE_8x8_CONTINUOUS;
  }
  else
  {
    printf("SENSOR: unrecognized resolution '%s' — keeping current\n", res_tok);
    pending_profile = Profile.RangingProfile;
  }

  /* ── Timing budget: strip trailing "ms" ── */
  pending_timing_ms = (uint32_t)strtoul(timing_tok, NULL, 10);
  if (pending_timing_ms < 5)   pending_timing_ms = 5;
  if (pending_timing_ms > 100) pending_timing_ms = 100;

  /* ── Frequency: strip trailing "Hz" ── */
  pending_freq_hz = (uint32_t)strtoul(freq_tok, NULL, 10);
  if (pending_freq_hz < 1)  pending_freq_hz = 1;
  if (pending_freq_hz > 30) pending_freq_hz = 30;

  printf("SENSOR config queued: res=%s timing=%lums freq=%luHz\n",
         res_tok,
         (unsigned long)pending_timing_ms,
         (unsigned long)pending_freq_hz);

  sensor_reconfig_pending = 1;
}

/* =========================================================================
   Apply_Pending_Sensor_Config — unchanged logic from previous version
   ========================================================================= */
static void Apply_Pending_Sensor_Config(void)
{
  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);

  Profile.RangingProfile = pending_profile;
  Profile.TimingBudget   = pending_timing_ms;
  Profile.Frequency      = pending_freq_hz;

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);

  status = VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER,
                                          RS_MODE_BLOCKING_CONTINUOUS);
  if (status != BSP_ERROR_NONE)
  {
    printf("SENSOR reconfigure: Start failed, status=%ld\n", (long)status);
  }
  else
  {
    printf("SENSOR reconfigure applied: profile=%d timing=%lums freq=%luHz\n",
           (int)Profile.RangingProfile,
           (unsigned long)Profile.TimingBudget,
           (unsigned long)Profile.Frequency);
  }
}

/* =========================================================================
   Send_GUI_Data — unchanged logic from previous version
   ========================================================================= */
static void Send_GUI_Data(uint32_t dist_mm)
{
  const char *zone_str;
  const char *alert_str;

  if (dist_mm < dist_near)
  {
    zone_str  = "DANGER";
    alert_str = "CONTINUOUS";
  }
  else if (dist_mm < dist_far_1)
  {
    zone_str  = "WARNING";
    alert_str = "FAST_BEEP";
  }
  else if (dist_mm < dist_far_2)
  {
    zone_str  = "CAUTION";
    alert_str = "SLOW_BEEP";
  }
  else
  {
    zone_str  = "SAFE";
    alert_str = "OFF";
  }

  sprintf(gui_buf,
    "DIST:%lu,ZONE:%s,ALERT:%s,STATUS:RUNNING\r\n",
    dist_mm, zone_str, alert_str);

  HAL_UART_Transmit(&hcom_uart[COM1],
                    (uint8_t *)gui_buf,
                    strlen(gui_buf),
                    100);
}

/* =========================================================================
   print_result — original terminal output
   ========================================================================= */
static void print_result(RANGING_SENSOR_Result_t *Result)
{
  int8_t i, j, k, l;
  uint8_t zones_per_line;

  zones_per_line = ((Profile.RangingProfile == RS_PROFILE_8x8_AUTONOMOUS) ||
                    (Profile.RangingProfile == RS_PROFILE_8x8_CONTINUOUS)) ? 8 : 4;

  display_commands_banner();

  printf("Cell Format :\n\n");
  for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
  {
    printf(" \033[38;5;10m%20s\033[0m : %20s\n",
           "Distance [mm]", "Status");
    if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
    {
      printf(" %20s : %20s\n",
             "Signal [kcps/spad]", "Ambient [kcps/spad]");
    }
  }
  printf("\n\n");

  for (j = 0; j < Result->NumberOfZones; j += zones_per_line)
  {
    for (i = 0; i < zones_per_line; i++)
      printf(" -----------------");
    printf("\n");

    for (i = 0; i < zones_per_line; i++)
      printf("|                 ");
    printf("|\n");

    for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
    {
      for (k = (zones_per_line - 1); k >= 0; k--)
      {
        if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                 (long)Result->ZoneResult[j + k].Distance[l],
                 (long)Result->ZoneResult[j + k].Status[l]);
        else
          printf("| %5s  :  %5s ", "X", "X");
      }
      printf("|\n");

      if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
      {
        for (k = (zones_per_line - 1); k >= 0; k--)
        {
          if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          {
            if (Profile.EnableSignal != 0)
              printf("| %5ld  :  ",
                     (long)Result->ZoneResult[j + k].Signal[l]);
            else
              printf("| %5s  :  ", "X");

            if (Profile.EnableAmbient != 0)
              printf("%5ld ",
                     (long)Result->ZoneResult[j + k].Ambient[l]);
            else
              printf("%5s ", "X");
          }
          else
            printf("| %5s  :  %5s ", "X", "X");
        }
        printf("|\n");
      }
    }
  }

  for (i = 0; i < zones_per_line; i++)
    printf(" -----------------");
  printf("\n");
}

/* =========================================================================
   Helper functions — original
   ========================================================================= */
static void toggle_resolution(void)
{
  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);

  switch (Profile.RangingProfile)
  {
    case RS_PROFILE_4x4_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_AUTONOMOUS;  break;
    case RS_PROFILE_4x4_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;  break;
    case RS_PROFILE_8x8_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_AUTONOMOUS;  break;
    case RS_PROFILE_8x8_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_CONTINUOUS;  break;
    default: break;
  }

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER,
                                 RS_MODE_BLOCKING_CONTINUOUS);
}

static void toggle_signal_and_ambient(void)
{
  VL53L8A1_RANGING_SENSOR_Stop(VL53L8A1_DEV_CENTER);
  Profile.EnableAmbient = (Profile.EnableAmbient) ? 0U : 1U;
  Profile.EnableSignal  = (Profile.EnableSignal)  ? 0U : 1U;
  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER,
                                 RS_MODE_BLOCKING_CONTINUOUS);
}

static void clear_screen(void)
{
  printf("%c[2J", 27);
}

static void display_commands_banner(void)
{
  printf("%c[2H", 27);
  printf("53L8A1 Simple Ranging demo application\n");
  printf("--------------------------------------\n\n");
  printf("Use the following keys to control application\n");
  printf(" 'r' : change resolution\n");
  printf(" 's' : enable signal and ambient\n");
  printf(" 'c' : clear screen\n");
  printf("\n");
}

static void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'r': toggle_resolution();        clear_screen(); break;
    case 's': toggle_signal_and_ambient(); clear_screen(); break;
    case 'c': clear_screen();                              break;
    default:  break;
  }
}

/* NOTE: get_key()/com_has_data() are kept only so the rest of the file
 * still compiles/links if anything else references them — they are no
 * longer used by the main RX path (FIX 3 replaced that with interrupts). */
static uint8_t get_key(void)
{
  uint8_t cmd = 0;
  HAL_UART_Receive(&hcom_uart[COM1], &cmd, 1, HAL_MAX_DELAY);
  return cmd;
}

static uint32_t com_has_data(void)
{
  return __HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE);
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  PushButtonDetected = 1;
}

#ifdef __cplusplus
}
#endif
