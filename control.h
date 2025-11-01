#pragma once
#include <stdint.h>
#include <stdbool.h>

// Egysoros "sink" callback típus (app.c adja meg).
// Minden új mintánál hívjuk: lpm, pulses, error_code (0=OK, !0 hiba)
typedef void (*hydro_sink_t)(float lpm, uint32_t pulses, uint8_t error_code, void *user);

// Init: GPIO + IRQ + belső állapot
void hydro_init(void);

// Pumpa engedélyezés/tiltás (elindítja/leállítja a belső mintavételt)
void hydro_enable(bool on);
bool hydro_is_enabled(void);

// Aktuális értékek lekérdezése
float    hydro_get_flow_lpm(void);
uint32_t hydro_get_pulse_count(void);

// App oldali "1 soros" GATT küldés beregisztrálása
void hydro_set_sink(hydro_sink_t cb, void *user);
