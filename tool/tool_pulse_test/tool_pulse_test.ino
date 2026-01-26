
#include <Arduino.h>
#include "driver/rmt.h"

#define STEP_GPIO      18
#define DIR_GPIO     4
#define RMT_CHANNEL    RMT_CHANNEL_0

#define RMT_CLK_DIV    80          // 80 MHz / 80 = 1 MHz (1 µs tick)
#define STEP_FREQ_HZ   10000       // 10 kHz default
#define STEP_COUNT     2000

void rmtStepperInit(){
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

void stepperMove(uint32_t steps, uint32_t freq_hz){
    uint32_t period_us = 1000000UL / freq_hz;
    uint32_t half_period = period_us / 2;

    rmt_item32_t step;
    step.level0 = 0;
    step.duration0 = half_period;
    step.level1 = 1;
    step.duration1 = half_period;

    for (uint32_t i = 0; i < steps; i++) {
        rmt_write_items(RMT_CHANNEL, &step, 1, true);
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

void setup()
{
    pinMode(DIR_GPIO, OUTPUT);
    digitalWrite(DIR_GPIO, HIGH);   // Set direction

    rmtStepperInit();

    delay(10);

    // Example move
    stepperMove(STEP_COUNT, STEP_FREQ_HZ);
}

void loop()
{
    // Example alternating direction


    digitalWrite(DIR_GPIO, !digitalRead(DIR_GPIO));
    
    delay(100);

    stepperMove(100, 500);  // 0.5 
    stepperMove(100, 1000);  // 1 kHz
    stepperMove(2000, 2000);  // 2 kHz
    stepperMove(100, 1000);  // 1 kHz
    stepperMove(100, 500);  // 0.5 kHz

    delay(1000);


}
