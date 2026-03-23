#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/ledc.h"

// PWM -------------------------
#define EN_PIN        32
#define PWM_GPIO_CW   26
#define PWM_GPIO_CCW  27
#define PWM_FREQ      1000
#define PWM_RES_BITS  8
#define PWM_DUTY_P    10
#define PWM_CH_CW     LEDC_CHANNEL_0
#define PWM_CH_CCW    LEDC_CHANNEL_1
#define CM            15000

const uint8_t PWM_DUTY = PWM_DUTY_P * (pow(2, PWM_RES_BITS)) / 100;

// Stepper -------------------
#define STEP_GPIO      16
#define DIR_GPIO       17
#define RMT_CHANNEL    RMT_CHANNEL_0

#define RMT_CLK_DIV    80
#define STEP_FREQ_HZ   10000
#define STEP_COUNT     2000

// Acceleration profile
#define ACC              25
#define LOW_SPEED        500
#define HIGH_SPEED       4500
#define ACC_DISC_INTERVAL 10


// PWM helpers

void setDuty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void bothOff() {
    setDuty(PWM_CH_CW,  0);
    setDuty(PWM_CH_CCW, 0);
}

// Drive PWM in whichever direction the DIR pin is currently set
void pwmOn() {
    bothOff();
    delay(1); // dead-time before switching
    if (digitalRead(DIR_GPIO) == HIGH) {
        setDuty(PWM_CH_CW, PWM_DUTY);
    } else {
        setDuty(PWM_CH_CCW, PWM_DUTY);
    }
}


// Init

void pwmInit() {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

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

    Serial.println("H-Bridge PWM ready");
}

void rmtStepperInit() {
    rmt_config_t config = {};
    config.rmt_mode      = RMT_MODE_TX;
    config.channel       = RMT_CHANNEL;
    config.gpio_num      = (gpio_num_t)STEP_GPIO;
    config.clk_div       = RMT_CLK_DIV;
    config.mem_block_num = 1;

    config.tx_config.loop_en          = false;
    config.tx_config.carrier_en       = false;
    config.tx_config.idle_output_en   = true;
    config.tx_config.idle_level       = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}


// Stepper motion

void stepperMove(uint32_t steps, uint32_t freq_hz) {
    uint32_t half_period = (1000000UL / freq_hz) / 2;

    rmt_item32_t step;
    step.level0    = 0;
    step.duration0 = half_period;
    step.level1    = 1;
    step.duration1 = half_period;

    for (uint32_t i = 0; i < steps; i++) {
        rmt_write_items(RMT_CHANNEL, &step, 1, true);
    }
    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}

void stepperMoveAccTrap(uint32_t steps) {
    uint16_t f = LOW_SPEED;
    uint32_t half_period = (1000000UL / f) / 2;
    uint32_t curve_length = steps / 2;

    rmt_item32_t step;
    step.level0    = 0;
    step.duration0 = half_period;
    step.level1    = 1;
    step.duration1 = half_period;

    uint32_t acc_steps       = min((int)curve_length, (HIGH_SPEED - LOW_SPEED) / (ACC / ACC_DISC_INTERVAL));
    uint32_t vel_change_steps = acc_steps / ACC_DISC_INTERVAL;
    uint32_t coast_steps     = steps - 2 * (vel_change_steps * ACC_DISC_INTERVAL);

    // Ramp up
    for (uint32_t i = 0; i < vel_change_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) {
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
        }
        f += ACC;
        half_period    = (1000000UL / f) / 2;
        step.duration0 = half_period;
        step.duration1 = half_period;
    }

    // Coast
    for (uint32_t i = 0; i < coast_steps; i++) {
        rmt_write_items(RMT_CHANNEL, &step, 1, true);
    }

    // Ramp down
    for (uint32_t i = 0; i < vel_change_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) {
            rmt_write_items(RMT_CHANNEL, &step, 1, true);
        }
        f -= ACC;
        half_period    = (1000000UL / f) / 2;
        step.duration0 = half_period;
        step.duration1 = half_period;
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
}


// Setup & loop

void setup() {
    Serial.begin(115200);

    Serial.println("begining initalization");

    pinMode(EN_PIN,  OUTPUT);
    pinMode(DIR_GPIO, OUTPUT);
    digitalWrite(EN_PIN,  HIGH);
    digitalWrite(DIR_GPIO, HIGH);

    rmtStepperInit();
    pwmInit();

    Serial.println("Counting!");
    for(int i = 5; i>0; i--){
        Serial.println(i);
        delay(1000);
    }

    Serial.println("Ready!");
}

void loop() {
    // --- CW move ---
    digitalWrite(DIR_GPIO, HIGH);
    pwmOn();                         // CW PWM on
    Serial.println("Stepper back, PWM CW");
    stepperMoveAccTrap(9.5*CM);
    bothOff();
    Serial.println("OFF");

    delay(1000);

    // --- CCW move ---
    digitalWrite(DIR_GPIO, LOW);
    pwmOn();                         // CCW PWM on
    Serial.println("Stepper forward, PWM CCW");
    stepperMoveAccTrap(9.5*CM);
    bothOff();
    Serial.println("OFF");

    delay(1000);
}