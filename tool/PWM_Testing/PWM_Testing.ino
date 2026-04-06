#include <Arduino.h>
#include "driver/ledc.h"

#define EN_PIN        32
#define PWM_GPIO_CW   26
#define PWM_GPIO_CCW  27
#define PWM_FREQ      1000
#define PWM_RES_BITS  8
#define PWM_DUTY_P    30
#define PWM_CH_CW     LEDC_CHANNEL_0
#define PWM_CH_CCW    LEDC_CHANNEL_1

const uint8_t PWM_DUTY = PWM_DUTY_P*(pow(2,PWM_RES_BITS))/100;

void setDuty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void bothOff() {
    setDuty(PWM_CH_CW, 0);
    setDuty(PWM_CH_CCW, 0);
}

void setup() {

    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    Serial.println("Motors free");
    Serial.print("PWM total bits: ");
    Serial.println((pow(2, PWM_RES_BITS)));
    Serial.print("PWM on bits: ");
    Serial.println(PWM_DUTY);


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

void loop() {
    digitalWrite(EN_PIN, HIGH);

    // CW on, CCW guaranteed off first
    bothOff();
    delay(1);                        // tiny dead-time before switching
    setDuty(PWM_CH_CW, PWM_DUTY);
    Serial.println("CW ON");
    delay(3500);

    // Dead-time gap
    bothOff();
    Serial.println("OFF");
    delay(1500);

    // CCW on, CW guaranteed off first
    bothOff();
    delay(1);                        // tiny dead-time before switching
    setDuty(PWM_CH_CCW, PWM_DUTY);
    Serial.println("CCW ON");
    delay(3500);

    // Dead-time gap
    bothOff();
    Serial.println("OFF");
    delay(1500);
}