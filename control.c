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

// ---- Átfolyásszenzor konstansok (YF-S201) --------------------------
#define FLOW_HZ_PER_LPM   5.71f   // 5.71 Hz == 1 L/min  (Q[L/min] = F[Hz] / 7.5)

// ---- PWM paraméterek -------------------------------------------------------
#define PWM_FREQ_HZ       1000u  // 1 kHz PWM
#define PWM_DUTY_NUM      2u     // 1/8 kitöltés számláló
#define PWM_DUTY_DEN      16u

// ---- Belső állapot ---------------------------------------------------------
static volatile uint32_t s_pulses = 0;
static uint32_t s_last_ticks = 0;
static uint32_t s_last_pulses = 0;
static float    s_lpm = 0.0f;

static bool     s_enabled = false;
static uint8_t  s_error   = 0;

static float    s_min_lpm_after = 0.2f; // pl. 0.2 L/min alatt hibának vesszük...
static uint8_t  s_min_after_s   = 3;    // ...3 mp-vel bekapcs után

static hydro_sink_t      s_sink = 0;
static void             *s_sink_user = 0;
static sl_sleeptimer_timer_handle_t s_sample_tmr;

// ---- Segédfüggvények -------------------------------------------------------
static void pump_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_LOW, gpioModePushPull, 0); // I1B = 0
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0); // I1A = 0 (off)
}


static void flow_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(FLOW_PORT, FLOW_PIN, gpioModeInputPullFilter, 1);
  GPIO_ExtIntConfig(FLOW_PORT, FLOW_PIN, FLOW_PIN, true, false, true);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

// C0 -> even line (0)
// ---- IRQ ----
void GPIO_EVEN_IRQHandler(void)
{
  uint32_t iflags = GPIO_IntGetEnabled();
  if (iflags & (1u << FLOW_PIN)) {
    GPIO_IntClear(1u << FLOW_PIN);
    s_pulses++;
  }
}

// ---- kiszámolja a L/perc-et és meghívja a sink-et ----
static void sample_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle; (void)data;

  // biztonságos olvasás
  __disable_irq();
  uint32_t p = s_pulses;
  __enable_irq();

  uint32_t dp = p - s_last_pulses;
  s_last_pulses = p;

  s_lpm = ((float)dp) / FLOW_HZ_PER_LPM;

  // egyszerű dry-run hiba: csak a bekapcsolás utáni n. másodperctől figyeljük
  static uint8_t seconds_since_on = 0;
  if (s_enabled) {
    if (seconds_since_on < 250) seconds_since_on++;
  } else {
    seconds_since_on = 0;
  }

  if (s_enabled && seconds_since_on >= s_min_after_s && s_lpm < s_min_lpm_after) {
    s_error = 1; // dry
  } else if (!s_enabled) {
    s_error = 0;
  }

  if (s_sink) {
    s_sink(s_lpm, p, s_error, s_sink_user);
  }
}

// ---- PUBLIC ----
void hydro_init(void)
{
  static bool inited = false;
  if (inited) return;
  pump_gpio_init();
  flow_gpio_init();
  s_last_ticks = sl_sleeptimer_get_tick_count();
  inited = true;
}

void hydro_enable(bool on)
{
  hydro_init();
  if (on == s_enabled) return;

  s_enabled = on;
  pump_set_on(on);

  if (on) {
    // mintavételi időzítő indítása 1000 ms periódussal
    (void)sl_sleeptimer_start_periodic_timer_ms(&s_sample_tmr, 1000, sample_cb, NULL, 0, 0);
    // induló állapot nullázás
    s_last_pulses = s_pulses;
    s_error = 0;
  } else {
    (void)sl_sleeptimer_stop_timer(&s_sample_tmr);
  }
}

bool hydro_is_enabled(void) { return s_enabled; }

float hydro_get_flow_lpm(void) { return s_lpm; }
uint32_t hydro_get_pulse_count(void) { return s_pulses; }

void hydro_set_sink(hydro_sink_t cb, void *user)
{
  s_sink = cb;
  s_sink_user = user;
}
