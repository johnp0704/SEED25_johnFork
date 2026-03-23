#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/ledc.h"


//PWM -------------------------
#define PWM_GPIO_CW    26
#define PWM_GPIO_CCW   27

#define PWM_FREQ       1000        // ~1 kHz
#define PWM_RES_BITS   8          // 10-bit resolution (0–256)
#define PWM_DUTY       25         // 50% duty (adjust this)

#define PWM_CH_CW       LEDC_CHANNEL_0
#define PWM_CH_CCW      LEDC_CHANNEL_1



// Stepper -------------------

#define STEP_GPIO      16
#define DIR_GPIO       17
#define RMT_CHANNEL    RMT_CHANNEL_0

#define RMT_CLK_DIV    80          // 80 MHz / 80 = 1 MHz (1 µs tick)
#define STEP_FREQ_HZ   10000       // 10 kHz default
#define STEP_COUNT     2000


//Acceleration profile in hz
#define ACC 25
#define LOW_SPEED 500
#define HIGH_SPEED 2000
#define ACC_DISC_INTERVAL 10 // How many steps to go before changing velocity


void pwmInit() {

    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // CW channel
    ledc_channel_config_t chCW = {
        .gpio_num   = PWM_GPIO_CW,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = PWM_CH_CW,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chCW);

    // CCW channel
    ledc_channel_config_t chCCW = {
        .gpio_num   = PWM_GPIO_CCW,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = PWM_CH_CCW,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chCCW);

    Serial.println("ALL OFF!");

}



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
    Serial.begin(115200);

    

    pinMode(DIR_GPIO, OUTPUT);
    digitalWrite(DIR_GPIO, HIGH);   // Set direction

    rmtStepperInit();
    pwmInit();


    delay(10);
    ledcWrite(PWM_CH_CW, PWM_DUTY); //TEST
    Serial.println("Running!");
}

void loop()
{
    // Example alternating direction


    digitalWrite(DIR_GPIO, !digitalRead(DIR_GPIO));
    
    delay(1000);
    // ledcWrite(PWM_CH_CW, PWM_DUTY);

    stepperMoveAccTrap(10000);
    // ledcWrite(PWM_CH_CW, 0);

    delay(1000);
}
