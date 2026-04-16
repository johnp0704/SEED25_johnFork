#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#define STEP_GPIO           16
#define DIR_GPIO            17
#define RMT_CHANNEL         RMT_CHANNEL_0

#define RMT_CLK_DIV         80          // 80 MHz / 80 = 1 MHz (1 µs tick)
#define STEP_FREQ_HZ        10000       // 10 kHz default
#define STEP_COUNT          2000

// Acceleration profile in hz
#define ACC                 25
#define LOW_SPEED           500
#define HIGH_SPEED          2000
#define ACC_DISC_INTERVAL   10          // How many steps before changing velocity

// Helper: integer min
static inline uint32_t u32_min(uint32_t a, uint32_t b) { return a < b ? a : b; }

// ── GPIO direction pin ───────────────────────────────────────────────────────

static bool s_dir_state = true;

static void dir_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DIR_GPIO, s_dir_state ? 1 : 0);
}

static void dir_toggle(void)
{
    s_dir_state = !s_dir_state;
    gpio_set_level(DIR_GPIO, s_dir_state ? 1 : 0);
}

// ── RMT stepper init ─────────────────────────────────────────────────────────

static void rmt_stepper_init(void)
{
    rmt_config_t config = {
        .rmt_mode               = RMT_MODE_TX,
        .channel                = RMT_CHANNEL,
        .gpio_num               = (gpio_num_t)STEP_GPIO,
        .clk_div                = RMT_CLK_DIV,
        .mem_block_num          = 1,
        .tx_config = {
            .loop_en            = false,
            .carrier_en         = false,
            .idle_output_en     = true,
            .idle_level         = RMT_IDLE_LEVEL_LOW,
        },
    };
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

// ── Constant-speed move ──────────────────────────────────────────────────────

static void stepper_move(uint32_t steps, uint32_t freq_hz)
{
    uint32_t half_period = (1000000UL / freq_hz) / 2;

    rmt_item32_t step = {
        .level0    = 0,
        .duration0 = half_period,
        .level1    = 1,
        .duration1 = half_period,
    };

    for (uint32_t i = 0; i < steps; i++) {
        rmt_write_items(RMT_CHANNEL, &step, 1, true);
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

// ── Trapezoidal acceleration move ────────────────────────────────────────────

static void stepper_move_acc_trap(uint32_t steps)
{
    uint16_t f           = LOW_SPEED;
    uint32_t curve_length = steps / 2;

    // Number of discrete velocity steps we can fit in the acceleration phase
    uint32_t max_vel_steps = (HIGH_SPEED - LOW_SPEED) / ACC * ACC_DISC_INTERVAL;
    uint32_t acc_steps     = u32_min(curve_length, max_vel_steps);
    uint32_t vel_change_steps = acc_steps / ACC_DISC_INTERVAL;

    // Coasting steps (account for integer rounding in acc/dec phases)
    uint32_t coast_steps = steps - 2 * (vel_change_steps * ACC_DISC_INTERVAL);

    rmt_item32_t step;

    // ── Acceleration ────────────────────────────────────────────
    for (uint32_t i = 0; i < vel_change_steps; i++) {
        uint32_t half_period = (1000000UL / f) / 2;
        step.level0    = 0;
        step.duration0 = half_period;
        step.level1    = 1;
        step.duration1 = half_period;

        for (uint32_t j = 0; j < ACC_DISC_INTERVAL; j++) {
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
        }
        f += ACC;
    }

    // ── Coast ────────────────────────────────────────────────────
    {
        uint32_t half_period = (1000000UL / f) / 2;
        step.level0    = 0;
        step.duration0 = half_period;
        step.level1    = 1;
        step.duration1 = half_period;

        for (uint32_t i = 0; i < coast_steps; i++) {
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
        }
    }

    // ── Deceleration ─────────────────────────────────────────────
    for (uint32_t i = 0; i < vel_change_steps; i++) {
        uint32_t half_period = (1000000UL / f) / 2;
        step.level0    = 0;
        step.duration0 = half_period;
        step.level1    = 1;
        step.duration1 = half_period;

        for (uint32_t j = 0; j < ACC_DISC_INTERVAL; j++) {
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
        }
        f -= ACC;
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

// ── Entry point ──────────────────────────────────────────────────────────────

void app_main(void)
{
    dir_gpio_init();
    rmt_stepper_init();

    vTaskDelay(pdMS_TO_TICKS(10));

    // Initial move (mirrors Arduino setup())
    stepper_move(STEP_COUNT, STEP_FREQ_HZ);

    // Loop equivalent
    while (true) {
        dir_toggle();
        vTaskDelay(pdMS_TO_TICKS(1000));

        stepper_move_acc_trap(10000);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}