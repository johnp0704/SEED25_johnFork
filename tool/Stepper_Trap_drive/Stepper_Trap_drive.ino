#include <Arduino.h>
#include "driver/rmt.h"

// ------------------- Pins -------------------
#define STEP_GPIO    18
#define DIR_GPIO     4    // D4
#define EN_GPIO      19   // D19 (active HIGH)

// ------------------- RMT --------------------
#define RMT_CHANNEL  RMT_CHANNEL_0
#define RMT_CLK_DIV  80   // 1 µs tick

// ---------------- Motion tuning -------------
#define LOW_RPM        10
#define HIGH_RPM       120
#define STEPS_PER_REV  200 //* 27  // 1.8deg with 1:27 gear ratio
#define ACCEL_STEPS    100       // steps used to ramp up/down
#define TOTAL_STEPS    7500

// --------------------------------------------

static inline uint32_t rpmToFreq(uint32_t rpm)
{
    return (rpm * STEPS_PER_REV) / 60;
}

void rmtStepperInit()
{
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)STEP_GPIO;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;

    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void stepBlock(uint32_t steps, uint32_t freq_hz)
{
    uint32_t period_us = 1000000UL / freq_hz;
    uint32_t half = period_us / 2;

    rmt_item32_t item;
    item.level0 = 1;
    item.duration0 = half;
    item.level1 = 0;
    item.duration1 = half;

    for (uint32_t i = 0; i < steps; i++) {
        rmt_write_items(RMT_CHANNEL, &item, 1, true);
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

void stepperMoveTrapezoidal(uint32_t total_steps)
{
    uint32_t accel_steps = min((uint32_t)ACCEL_STEPS, total_steps / 2);
    uint32_t cruise_steps = total_steps - 2 * accel_steps;

    uint32_t f_low  = rpmToFreq(LOW_RPM);
    uint32_t f_high = rpmToFreq(HIGH_RPM);

    // ---------- Ramp up ----------
    for (uint32_t i = 0; i < accel_steps; i++) {
        uint32_t f = f_low + (f_high - f_low) * i / accel_steps;
        stepBlock(1, f);
    }

    // ---------- Cruise ----------
    if (cruise_steps > 0) {
        stepBlock(cruise_steps, f_high);
    }

    // ---------- Ramp down ----------
    for (uint32_t i = accel_steps; i > 0; i--) {
        uint32_t f = f_low + (f_high - f_low) * i / accel_steps;
        stepBlock(1, f);
    }
}

void setup()
{
    pinMode(DIR_GPIO, OUTPUT);
    pinMode(EN_GPIO, OUTPUT);

    digitalWrite(EN_GPIO, HIGH);  // enable driver
    digitalWrite(DIR_GPIO, HIGH); // forward

    rmtStepperInit();
    delay(10);
}

void loop()
{
    // Forward
    digitalWrite(DIR_GPIO, HIGH);
    delayMicroseconds(5); // DIR setup time
    stepperMoveTrapezoidal(TOTAL_STEPS);

    delay(1000);

    // Backward
    digitalWrite(DIR_GPIO, LOW);
    delayMicroseconds(5);
    stepperMoveTrapezoidal(TOTAL_STEPS);

    delay(1000);
}
