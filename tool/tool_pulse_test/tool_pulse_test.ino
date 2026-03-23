#include <Arduino.h>
#include "driver/rmt.h"

#define STEP_GPIO      18
#define DIR_GPIO       4
#define RMT_CHANNEL    RMT_CHANNEL_0

#define RMT_CLK_DIV    80          // 80 MHz / 80 = 1 MHz (1 µs tick)
#define STEP_FREQ_HZ   10000       // 10 kHz default
#define STEP_COUNT     2000


//Acceleration profile in hz
#define ACC 25
#define LOW_SPEED 500
#define HIGH_SPEED 5000
#define ACC_DISC_INTERVAL 10 // How many steps to go before changing velocity


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



void stepperMoveAccTrap(uint32_t steps){
    uint16_t f = LOW_SPEED;
    uint32_t period_us = 1000000UL / f;
    uint32_t half_period = period_us / 2;
    uint32_t curve_length = steps/2;

    rmt_item32_t step;
    step.level0 = 0;
    step.duration0 = half_period;
    step.level1 = 1;
    step.duration1 = half_period;

    // Make sure we have time to stop
    uint32_t acc_steps = min((int)curve_length, (HIGH_SPEED-LOW_SPEED)/(ACC/ACC_DISC_INTERVAL));


    uint32_t vel_change_steps = acc_steps/ACC_DISC_INTERVAL; //How many velocity steeps
    uint32_t coast_steps = steps - 2*(vel_change_steps*ACC_DISC_INTERVAL); //recalculate acc steps to account for int rounding
 

    for (uint16_t i = 0; i < vel_change_steps; i++) {
        for (uint16_t i = 0; i < ACC_DISC_INTERVAL; i++){
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
            
        }
        //Update speed at increment
        f += ACC;
        half_period = (1000000UL / f) /2;

        step.duration0 = half_period;
        step.duration1 = half_period;   
    }

  
    for (uint16_t i = 0; i < coast_steps; i++){
        rmt_write_items(RMT_CHANNEL, &step, 1, true);
        
    }
  
    

    for (uint16_t i = 0; i < vel_change_steps; i++) {
        for (uint16_t i = 0; i < ACC_DISC_INTERVAL; i++){
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
            
        }
        //Update speed at increment
        f -= ACC;
        half_period = (1000000UL / f) /2;

        step.duration0 = half_period;
        step.duration1 = half_period;   
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

    // stepperMove(1000, 500);  // 0.5 
    stepperMoveAccTrap(20000);

    delay(1000);


}
