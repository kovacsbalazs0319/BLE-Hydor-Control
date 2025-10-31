#include "em_gpio.h"
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include "app_log.h"
#include <stdbool.h>
#include <stdint.h>

// ---- Pin kiosztás ----------------------------------------------------------
#define PUMP_PORT         gpioPortD
#define PUMP_PIN_PWM      3   // D3 : I1A  -> PWM
#define PUMP_PIN_LOW      2   // D2 : I1B  -> fix LOW

#define FLOW_PORT         gpioPortC
#define FLOW_PIN          0   // C0 : Flow_data (rising edge count)

// ---- Átfolyásszenzor konstansok (YF-S201 tipikus) --------------------------
#define FLOW_HZ_PER_LPM   5.71f   // 5.71 Hz == 1 L/min  (Q[L/min] = F[Hz] / 7.5)

// ---- PWM paraméterek -------------------------------------------------------
#define PWM_FREQ_HZ       1000u  // 1 kHz PWM
#define PWM_DUTY_NUM      2u     // 1/8 kitöltés számláló
#define PWM_DUTY_DEN      16u

// ---- Belső állapot ---------------------------------------------------------
static volatile uint32_t s_flow_pulse_count = 0;

static bool     s_inited = false;
static uint32_t s_pwm_period_ticks = 0;
static uint32_t s_pwm_high_ticks   = 0;
static uint32_t s_pwm_epoch_ticks  = 0;

static uint32_t s_last_sample_ticks = 0;
static uint32_t s_last_sample_count = 0;

// ---- Segédfüggvények -------------------------------------------------------
static void flow_gpio_init(void)
{
  // Óra engedélyezés (általában már megy, de nem árt)
  CMU_ClockEnable(cmuClock_GPIO, true);

  // C0 bemenet: pullup + filter javasolt a tisztább jelhez
  GPIO_PinModeSet(FLOW_PORT, FLOW_PIN, gpioModeInputPullFilter, 1);

  // Külső megszakítás konfigurálása: rising edge
  // extint = pin száma (0..15), port független
  GPIO_ExtIntConfig(FLOW_PORT, FLOW_PIN, FLOW_PIN, true, false, true);

  // NVIC engedély
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);   // pin 0 -> EVEN
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

static void pump_gpio_init(void)
{
  // D2: fix LOW (ellenirányú félhíd letiltva)
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_LOW, gpioModePushPull, 0);

  // D3: PWM szoftveresen vezérelve
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0);
}

static void timing_init(void)
{
  // Sleeptimer frekvencia
  uint32_t tick_hz = sl_sleeptimer_get_timer_frequency();

  // PWM tick periódus és high idő
  s_pwm_period_ticks = tick_hz / PWM_FREQ_HZ;
  if (s_pwm_period_ticks == 0) s_pwm_period_ticks = 1;

  // 1/8 kitöltés
  s_pwm_high_ticks = (s_pwm_period_ticks * PWM_DUTY_NUM) / PWM_DUTY_DEN;
  if (s_pwm_high_ticks == 0) s_pwm_high_ticks = 1;

  s_pwm_epoch_ticks  = sl_sleeptimer_get_tick_count();
  s_last_sample_ticks = s_pwm_epoch_ticks;
  s_last_sample_count = 0;
}

void app_init_once(void)
{
  pump_gpio_init();
  flow_gpio_init();
  timing_init();

  app_log("Flow test init done. PWM=%lu Hz, duty=%u/%u, YF-S201 K=%.2f Hz per L/min\r\n",
          (unsigned long)PWM_FREQ_HZ, PWM_DUTY_NUM, PWM_DUTY_DEN, (double)FLOW_HZ_PER_LPM);
  s_inited = true;
}

// ---- GPIO IRQ-k ------------------------------------------------------------
// C0 -> even line (0)
void GPIO_EVEN_IRQHandler(void)
{
  uint32_t iflags = GPIO_IntGetEnabled();
  if (iflags & (1u << FLOW_PIN)) {
    GPIO_IntClear(1u << FLOW_PIN);
    s_flow_pulse_count++;
  }
  // További even pinekhez itt lehet bővíteni
}

// ---- Folyton futó rész: hívd az app_process_action()-ból -------------------
void app_main_loop_step(void)
{
  if (!s_inited) {
    app_init_once();
  }

  // ---- Szoftveres PWM D3-on ------------------------------------------------
  uint32_t now = sl_sleeptimer_get_tick_count();
  uint32_t phase = (now - s_pwm_epoch_ticks) % s_pwm_period_ticks;

  if (phase < s_pwm_high_ticks) {
    GPIO_PinOutSet(PUMP_PORT, PUMP_PIN_PWM);
  } else {
    GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM);
  }

  // ---- Átfolyás számítás és log (1 s ablak) --------------------------------
  const uint32_t sample_window_ms = 1000;
  uint32_t tick_hz = sl_sleeptimer_get_timer_frequency();
  uint32_t window_ticks = (tick_hz * sample_window_ms) / 1000;

  if ((now - s_last_sample_ticks) >= window_ticks) {
    // Biztonságos olvasás (IRQ-val versenyhelyzet elkerülése)
    __disable_irq();
    uint32_t pulses = s_flow_pulse_count;
    __enable_irq();

    uint32_t dpulses = pulses - s_last_sample_count;     // ablakban mért impulzus
    s_last_sample_count = pulses;
    s_last_sample_ticks = now;

    // Frekvencia Hz-ben (dpulses / 1s)
    float freq_hz = (float)dpulses;
    // Liter/perc becslés
    float lpm = freq_hz / FLOW_HZ_PER_LPM;

    app_log("Flow: pulses=%lu, freq=%.2f Hz, Q=%.3f L/min\r\n",
            (unsigned long)dpulses, (double)freq_hz, (double)lpm);
  }
}
