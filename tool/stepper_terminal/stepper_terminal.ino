#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/ledc.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X distanceSensor;

// ============================================================
//  PIN / PWM CONFIG
// ============================================================
#define EN_PIN        32
#define PWM_GPIO_CW   26
#define PWM_GPIO_CCW  27
#define HOME_PIN      25        // bump switch, active LOW (internal pull-up)
#define PWM_FREQ      1000
#define PWM_RES_BITS  8
#define PWM_CH_CW     LEDC_CHANNEL_0
#define PWM_CH_CCW    LEDC_CHANNEL_1

// ============================================================
//  STEPPER CONFIG
// ============================================================
#define STEP_GPIO          16
#define DIR_GPIO           17
#define RMT_CHANNEL        RMT_CHANNEL_0
#define RMT_CLK_DIV        80

#define LOW_SPEED          500    // Hz  (start / end of ramp)
#define HIGH_SPEED         6500   // Hz  (cruise speed)
#define ACC                25     // Hz gained per discrete interval
#define ACC_DISC_INTERVAL  10     // steps between speed updates

#define HOMING_SPEED       1000    // Hz  (fast approach)
#define BACKOFF_SPEED      500    // Hz  (slow back-off / creep)

#define FB_TOL             15     // mm of +- tol on movement (for tool-stuck checks only)

// ============================================================
//  EEPROM CONFIG
// ============================================================
// Memory map  (big-endian uint32_t / float, 4 bytes each):
//   0x00  magic          -- 0xDEADBEEF  range cal validity sentinel
//   0x04  offset         -- steps from switch trigger to software zero
//   0x08  range          -- usable travel in steps from software zero
//   0x0C  fb_magic       -- 0xCAFEF00D  feedback cal validity sentinel
//   0x10  fb_m  (float)  -- slope  of linear fit  (steps -> sensor units)
//   0x14  fb_b  (float)  -- intercept of linear fit
//
#define EEPROM_SIZE            24
#define EEPROM_ADDR_MAGIC       0
#define EEPROM_ADDR_OFFSET      4
#define EEPROM_ADDR_RANGE       8
#define EEPROM_ADDR_FB_MAGIC   12
#define EEPROM_ADDR_FB_M       16
#define EEPROM_ADDR_FB_B       20
#define EEPROM_MAGIC           0xDEADBEEF
#define EEPROM_FB_MAGIC        0xCAFEF00D


// ============================================================
//  RETURN CODES
// ============================================================
#define RC_OK              0
#define RC_MOVE_UNSAFE     1
// codes 2-9 reserved

// ============================================================
//  DRILL CONFIG
// ============================================================
#define DRILL_STARTUP_MS      200   // ms to spin up/down drill before/after move
#define DRILL_DUTY_PCT        12//25     // PWM duty cycle for drill motor (0-100)
#define DRILLING_START        30000 // position at which to start drilling
#define DRILLING_MOVE_SPEED   2500  //speed at which to move drill



// ============================================================
//  CALIBRATION DATA
// ============================================================
struct CalData {
    uint32_t      offset;   // steps: switch trigger -> software zero
    unsigned long range;    // steps: software zero  -> end of travel
};

struct FbCal {
    float m;                // slope     (sensor_reading = m * steps + b)
    float b;                // intercept
};

CalData calData  = { 0, 0 };
bool    calValid = false;

FbCal   fbCal    = { 0.0f, 0.0f };
bool    fbCalValid = false;

// ============================================================
//  MACHINE STATE
// ============================================================
volatile long currentPos = 0;
bool          isHomed    = false;

// ============================================================
//  EEPROM HELPERS
// ============================================================
void eepromWriteU32(int addr, uint32_t val) {
    EEPROM.writeByte(addr + 0, (val >> 24) & 0xFF);
    EEPROM.writeByte(addr + 1, (val >> 16) & 0xFF);
    EEPROM.writeByte(addr + 2, (val >>  8) & 0xFF);
    EEPROM.writeByte(addr + 3, (val >>  0) & 0xFF);
}

uint32_t eepromReadU32(int addr) {
    uint32_t val = 0;
    val |= (uint32_t)EEPROM.readByte(addr + 0) << 24;
    val |= (uint32_t)EEPROM.readByte(addr + 1) << 16;
    val |= (uint32_t)EEPROM.readByte(addr + 2) <<  8;
    val |= (uint32_t)EEPROM.readByte(addr + 3) <<  0;
    return val;
}

// Store a float as its raw IEEE-754 bits.
void eepromWriteFloat(int addr, float val) {
    uint32_t bits;
    memcpy(&bits, &val, 4);
    eepromWriteU32(addr, bits);
}

float eepromReadFloat(int addr) {
    uint32_t bits = eepromReadU32(addr);
    float val;
    memcpy(&val, &bits, 4);
    return val;
}

// ============================================================
//  CALIBRATION LOAD / SAVE / CLEAR / PRINT
// ============================================================
void loadCalibration() {
    // -- range calibration --
    uint32_t magic = eepromReadU32(EEPROM_ADDR_MAGIC);
    if (magic == EEPROM_MAGIC) {
        calData.offset = eepromReadU32(EEPROM_ADDR_OFFSET);
        calData.range  = (unsigned long)eepromReadU32(EEPROM_ADDR_RANGE);
        calValid = true;
        Serial.printf("  Range cal loaded   |  offset = %lu  |  range = %lu\n",
                      calData.offset, calData.range);
    } else {
        calValid = false;
        Serial.println("  No range calibration found in EEPROM.");
    }

    // -- feedback calibration --
    uint32_t fbMagic = eepromReadU32(EEPROM_ADDR_FB_MAGIC);
    if (fbMagic == EEPROM_FB_MAGIC) {
        fbCal.m    = eepromReadFloat(EEPROM_ADDR_FB_M);
        fbCal.b    = eepromReadFloat(EEPROM_ADDR_FB_B);
        fbCalValid = true;
        Serial.printf("  Feedback cal loaded|  m = %.6f  |  b = %.4f\n",
                      fbCal.m, fbCal.b);
    } else {
        fbCalValid = false;
        Serial.println("  No feedback calibration found in EEPROM.");
    }
}

void saveCalibration() {
    eepromWriteU32(EEPROM_ADDR_MAGIC,  EEPROM_MAGIC);
    eepromWriteU32(EEPROM_ADDR_OFFSET, calData.offset);
    eepromWriteU32(EEPROM_ADDR_RANGE,  (uint32_t)calData.range);
    EEPROM.commit();
    calValid = true;
    Serial.printf("  Range cal saved    |  offset = %lu  |  range = %lu\n",
                  calData.offset, calData.range);
}

void saveFbCalibration() {
    eepromWriteU32  (EEPROM_ADDR_FB_MAGIC, EEPROM_FB_MAGIC);
    eepromWriteFloat(EEPROM_ADDR_FB_M,     fbCal.m);
    eepromWriteFloat(EEPROM_ADDR_FB_B,     fbCal.b);
    EEPROM.commit();
    fbCalValid = true;
    Serial.printf("  Feedback cal saved |  m = %.6f  |  b = %.4f\n",
                  fbCal.m, fbCal.b);
}

void clearCalibration() {
    for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.writeByte(i, 0x00);
    EEPROM.commit();
    calData    = { 0, 0 };
    fbCal      = { 0.0f, 0.0f };
    calValid   = false;
    fbCalValid = false;
    Serial.println("  All EEPROM calibration erased (range + feedback).");
}

void printCalibration() {
    Serial.println("  -- Range calibration --");
    if (!calValid) {
        Serial.println("    NOT SET");
    } else {
        Serial.printf("    offset = %lu steps   (switch trigger -> software zero)\n",
                      calData.offset);
        Serial.printf("    range  = %lu steps   (software zero  -> end of travel)\n",
                      calData.range);
    }

    Serial.println("  -- Feedback calibration --");
    if (!fbCalValid) {
        Serial.println("    NOT SET");
    } else {
        Serial.printf("    m = %.6f   (slope,     steps -> sensor units)\n", fbCal.m);
        Serial.printf("    b = %.4f   (intercept, steps -> sensor units)\n", fbCal.b);
    }
}

// ============================================================
//  PWM HELPERS
// ============================================================
void setDuty(ledc_channel_t ch, uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void bothOff() {
    setDuty(PWM_CH_CW,  0);
    setDuty(PWM_CH_CCW, 0);
}

void pwmSetChannel(ledc_channel_t ch, uint8_t pct) {
    uint32_t duty  = (uint32_t)(pct * (1 << PWM_RES_BITS) / 100);
    ledc_channel_t other = (ch == PWM_CH_CW) ? PWM_CH_CCW : PWM_CH_CW;
    setDuty(other, 0);
    delay(1);
    setDuty(ch, duty);
}

// ============================================================
//  HOME SWITCH
// ============================================================
bool homeTriggered() {
    return digitalRead(HOME_PIN) == LOW;
}

// ============================================================
//  HARDWARE INIT
// ============================================================
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
}

void rmtStepperInit() {
    rmt_config_t config = {};
    config.rmt_mode      = RMT_MODE_TX;
    config.channel       = RMT_CHANNEL;
    config.gpio_num      = (gpio_num_t)STEP_GPIO;
    config.clk_div       = RMT_CLK_DIV;
    config.mem_block_num = 1;

    config.tx_config.loop_en        = false;
    config.tx_config.carrier_en     = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void feedbackSensorInit() {
    Wire.begin(21, 22);

    if (distanceSensor.begin() != 0) {
        Serial.println("VL53L1X not detected");
        // while (1);
    }

    distanceSensor.setDistanceModeShort();
    distanceSensor.setTimingBudgetInMs(50);
    distanceSensor.setIntermeasurementPeriod(55);
    distanceSensor.setROI(4, 4, 199);
    distanceSensor.startRanging();

    Serial.println("VL53L1X ready (narrow FOV)");
}

// ============================================================
//  LOW-LEVEL STEP PRIMITIVE
// ============================================================
void sendStep(uint32_t freq_hz) {
    uint32_t half = (1000000UL / freq_hz) / 2;
    if (half < 1) half = 1;

    rmt_item32_t item;
    item.level0    = 0;
    item.duration0 = half;
    item.level1    = 1;
    item.duration1 = half;

    rmt_write_items(RMT_CHANNEL, &item, 1, true);
}

// ============================================================
//  MEDIAN HELPER
// ============================================================
double calculateMedian(float arr[], int n) {
    float tempArr[n];
    for (int i = 0; i < n; i++) tempArr[i] = arr[i];
    std::sort(tempArr, tempArr + n);

    if (n % 2 != 0) {
        return (double)tempArr[n / 2];
    } else {
        return ((double)tempArr[(n - 1) / 2] + (double)tempArr[n / 2]) / 2.0;
    }
}

// ============================================================
//  FEEDBACK CALIBRATION
// ============================================================
// Sweeps from calData.offset to calData.range in 1000-step increments,
// takes a median reading at each point, fits y = m*x + b via ordinary
// least squares, stores m and b to EEPROM, and reports R^2.
//
// x = step position,  y = sensor reading (mm)
//

int stepsToMm(unsigned long steps){
    return steps * fbCal.m + fbCal.b;
}

bool toolNotStuck(unsigned long pos){
    //Cross refrence too ideal position to what sensor sees (checks if stuck)
    float idealMM = stepsToMm(pos);

    // Serial.print("Expected: ");
    // Serial.println(idealMM);
    // Serial.print("Actual: ");
    float dist = distanceSensor.getDistance();
    // Serial.println(dist);
    // Serial.println(abs((float)dist - idealMM) < FB_TOL);


    return abs(dist - idealMM) < FB_TOL;



}


void calibrateFeedback() {
    const int NUM_READINGS = 20;
    const int STEP_SIZE    = 1000;

    if (!isHomed) {
        Serial.println("Warning: not homed! Please home first.");
        return;
    }
    if (!calValid) {
        Serial.println("ERROR: no range calibration. Set offset and range first.");
        return;
    }

    // Count how many samples the sweep will produce
    int n = 0;
    for (long pos = (long)calData.offset; pos < (long)calData.range; pos += STEP_SIZE) n++;
    if (n < 2) {
        Serial.println("ERROR: range too small for a linear fit (need at least 2 points).");
        return;
    }

    // Allocate sample buffers on the heap to avoid stack overflow
    double *xs = new double[n];
    double *ys = new double[n];
    if (!xs || !ys) {
        Serial.println("ERROR: memory allocation failed.");
        delete[] xs; delete[] ys;
        return;
    }

    Serial.println("Press enter to start feedback calibration sweep...");
    while (!Serial.available()) delay(5);
    while (Serial.available()) Serial.read();

    // --- Sweep and collect medians ---
    int idx = 0;
    for (long pos = (long)calData.offset; pos < (long)calData.range; pos += STEP_SIZE) {
        moveAbsolute(pos, false);

        float readings[NUM_READINGS];
        for (int r = 0; r < NUM_READINGS; r++) {
            delay(25);
            readings[r] = (float)distanceSensor.getDistance();
        }

        xs[idx] = (double)pos;
        ys[idx] = calculateMedian(readings, NUM_READINGS);
        Serial.println(ys[idx]);
        idx++;
    }

    // --- Ordinary least squares: y = m*x + b ---
    double sum_x  = 0, sum_y  = 0;
    double sum_xx = 0, sum_xy = 0;
    for (int i = 0; i < n; i++) {
        sum_x  += xs[i];
        sum_y  += ys[i];
        sum_xx += xs[i] * xs[i];
        sum_xy += xs[i] * ys[i];
    }
    double denom = (double)n * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 1e-12) {
        Serial.println("ERROR: degenerate fit (all x values identical?).");
        delete[] xs; delete[] ys;
        return;
    }

    double m = ((double)n * sum_xy - sum_x * sum_y) / denom;
    double b = (sum_y - m * sum_x) / (double)n;

    // --- R^2 ---
    double y_mean  = sum_y / (double)n;
    double ss_tot  = 0, ss_res = 0;
    for (int i = 0; i < n; i++) {
        double y_hat = m * xs[i] + b;
        ss_res += (ys[i] - y_hat)   * (ys[i] - y_hat);
        ss_tot += (ys[i] - y_mean)  * (ys[i] - y_mean);
    }
    double r2 = (ss_tot < 1e-12) ? 0.0 : (1.0 - ss_res / ss_tot);

    delete[] xs;
    delete[] ys;

    // --- Store and report ---
    fbCal.m    = (float)m;
    fbCal.b    = (float)b;
    saveFbCalibration();

    Serial.printf("  m   = %.6f\n", fbCal.m);
    Serial.printf("  b   = %.4f\n", fbCal.b);
    Serial.printf("  R^2 = %.6f\n", r2);

    //Move home
    moveAbsolute(calData.offset, false);
    
}

// ============================================================
//  FEEDBACK SENSOR MONITOR
// ============================================================
void feedbackSensor() {
    Serial.println("Monitoring feedback sensor  --  press any key to exit.");

    while (!Serial.available()) {
        int reading = distanceSensor.getDistance();
        Serial.println(reading);
        delay(10);
    }
    while (Serial.available()) Serial.read();
    Serial.println("Exited feedback sensor monitor.");
}



// ============================================================
//  TRAPEZOIDAL RELATIVE MOVE  (stepper only, no PWM)
// ============================================================
void moveRelative(long steps) {
    if (steps == 0) return;

    bool     cw = (steps > 0);
    uint32_t n  = (uint32_t)abs(steps);

    digitalWrite(DIR_GPIO, cw ? HIGH : LOW);

    uint16_t f            = LOW_SPEED;
    uint32_t curve_length = n / 2;
    uint32_t acc_steps    = min((int)curve_length,
                                (HIGH_SPEED - LOW_SPEED) / (ACC / ACC_DISC_INTERVAL));
    uint32_t vel_steps    = acc_steps / ACC_DISC_INTERVAL;
    uint32_t coast_steps  = n - 2 * (vel_steps * ACC_DISC_INTERVAL);

    for (uint32_t i = 0; i < vel_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f += ACC;
    }
    for (uint32_t i = 0; i < coast_steps; i++) sendStep(f);
    for (uint32_t i = 0; i < vel_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f -= ACC;
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
    currentPos += cw ? -(long)n : (long)n;
}



void moveRelativeSpeed(long steps, int speed) {
    if (steps == 0) return;

    bool     cw = (steps > 0);
    uint32_t n  = (uint32_t)abs(steps);

    digitalWrite(DIR_GPIO, cw ? HIGH : LOW);

    uint16_t f            = LOW_SPEED;
    uint32_t curve_length = n / 2;
    uint32_t acc_steps    = min((int)curve_length,
                                (speed - LOW_SPEED) / (ACC / ACC_DISC_INTERVAL));
    uint32_t vel_steps    = acc_steps / ACC_DISC_INTERVAL;
    uint32_t coast_steps  = n - 2 * (vel_steps * ACC_DISC_INTERVAL);

    for (uint32_t i = 0; i < vel_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f += ACC;
    }
    for (uint32_t i = 0; i < coast_steps; i++) sendStep(f);
    for (uint32_t i = 0; i < vel_steps; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f -= ACC;
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
    currentPos += cw ? -(long)n : (long)n;
}

// ============================================================
//  ABSOLUTE MOVE
// ============================================================


bool moveAbsoluteCheck(long targetPos, bool verbose) {
    if (!isHomed) {
        Serial.println("ERROR: machine not homed. Run 'home' first.");
        return false;
    }
    long delta = targetPos - currentPos;
    if (delta == 0) {
        if (verbose) Serial.printf("Already at position %ld\n", currentPos);
        return false;
    }
    if ((unsigned long)targetPos > calData.range) {
        Serial.printf("Value out of range! Max is: %lu\n", calData.range);
        return false;
    }
    if (verbose) {
        Serial.printf("Moving  %ld  ->  %ld  (%+ld steps)\n",
                      currentPos, targetPos, delta);
    }
    moveRelative(-delta);
    currentPos = targetPos;

    delay(50);
    bool stuck = !toolNotStuck(currentPos);

    if(stuck && verbose){
        Serial.println("TOOL STUCK");
        

        // abs(dist - idealMM) < FB_TOL

    } else if(!stuck && verbose){
        Serial.println("Move good!");
    }

    //return move status (stuck = bad move = false)
    return !stuck;
}





bool moveAbsoluteCheckSpeed(long targetPos, int speed) {
    bool verbose = false;
    if (!isHomed) {
        Serial.println("ERROR: machine not homed. Run 'home' first.");
        return false;
    }
    long delta = targetPos - currentPos;
    if (delta == 0) {
        if (verbose) Serial.printf("Already at position %ld\n", currentPos);
        return false;
    }
    if ((unsigned long)targetPos > calData.range) {
        Serial.printf("Value out of range! Max is: %lu\n", calData.range);
        return false;
    }
    if (verbose) {
        Serial.printf("Moving  %ld  ->  %ld  (%+ld steps)\n",
                      currentPos, targetPos, delta);
    }
    moveRelativeSpeed(-delta, speed);
    currentPos = targetPos;

    delay(50);
    bool stuck = !toolNotStuck(currentPos);

    if(stuck && verbose){
        Serial.println("TOOL STUCK");
        

        // abs(dist - idealMM) < FB_TOL

    } else if(!stuck && verbose){
        Serial.println("Move good!");
    }

    //return move status (stuck = bad move = false)
    return !stuck;
}




void moveAbsolute(long targetPos, bool verbose) {
    if (!isHomed) {
        Serial.println("ERROR: machine not homed. Run 'home' first.");
        return;
    }
    long delta = targetPos - currentPos;
    if (delta == 0) {
        if (verbose) Serial.printf("Already at position %ld\n", currentPos);
        return;
    }
    if ((unsigned long)targetPos > calData.range) {
        Serial.printf("Value out of range! Max is: %lu\n", calData.range);
        return;
    }
    if (verbose) {
        Serial.printf("Moving  %ld  ->  %ld  (%+ld steps)\n",
                      currentPos, targetPos, delta);
    }
    moveRelative(-delta);
    currentPos = targetPos;
}


void moveAbsoluteSpeed(long targetPos, int speed) {
    if (!isHomed) {
        Serial.println("ERROR: machine not homed. Run 'home' first.");
        return;
    }
    long delta = targetPos - currentPos;
    if (delta == 0) {
        // if (verbose) Serial.printf("Already at position %ld\n", currentPos);
        return;
    }
    if ((unsigned long)targetPos > calData.range) {
        Serial.printf("Value out of range! Max is: %lu\n", calData.range);
        return;
    }
    // if (verbose) {
    //     Serial.printf("Moving  %ld  ->  %ld  (%+ld steps)\n",
    //                   currentPos, targetPos, delta);
    // }
    moveRelativeSpeed(-delta, speed);
    currentPos = targetPos;
}

// ============================================================
//  HOMING  (stepper only, no PWM)
// ============================================================
void doHome() {
    Serial.println("Homing: moving CW toward switch...");
    digitalWrite(DIR_GPIO, HIGH);
    while (!homeTriggered()) sendStep(HOMING_SPEED);
    Serial.println("Switch triggered. Backing off...");

    delay(50);

    digitalWrite(DIR_GPIO, LOW);
    while (homeTriggered()) sendStep(BACKOFF_SPEED);

    delay(50);

    digitalWrite(DIR_GPIO, HIGH);
    while (!homeTriggered()) sendStep(BACKOFF_SPEED);

    delay(50);

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);

    currentPos = 0;
    isHomed    = true;
    Serial.println("Home set. currentPos = 0  (raw switch point).");

    if (calValid) {
        moveAbsolute(calData.offset, true);
    }
}

// ============================================================
//  SWITCH MONITOR
// ============================================================
void monitorSwitch() {
    Serial.println("Monitoring home switch  --  press any key to exit.");
    bool last = homeTriggered();
    Serial.printf("  [%s]\n", last ? "TRIGGERED" : "open     ");

    while (!Serial.available()) {
        bool state = homeTriggered();
        if (state != last) {
            Serial.printf("  [%s]\n", state ? "TRIGGERED" : "open     ");
            last = state;
        }
        delay(10);
    }
    while (Serial.available()) Serial.read();
    Serial.println("Exited switch monitor.");
}





// ============================================================
//  PWM TEST MODE
// ============================================================
void pwmTest() {
    bothOff();
    Serial.println("PWM test mode.");
    Serial.println("  cw  <0-100>  : run CW channel at duty %");
    Serial.println("  ccw <0-100>  : run CCW channel at duty %");
    Serial.println("  off          : both channels off");
    Serial.println("  exit         : leave PWM test (turns off)");
    Serial.print("pwm> ");

    String buf = "";

    while (true) {
        while (!Serial.available()) delay(5);

        char c = Serial.read();
        if (c == '\r') continue;

        if (c == '\n') {
            Serial.println();
            buf.trim();

            if (buf.equalsIgnoreCase("exit") || buf.equalsIgnoreCase("quit")) {
                bothOff();
                Serial.println("PWM off. Exited PWM test.");
                buf = "";
                return;
            } else if (buf.equalsIgnoreCase("off")) {
                bothOff();
                Serial.println("  Both channels off.");
            } else if (buf.startsWith("cw ") || buf.startsWith("CW ")) {
                int pct = buf.substring(3).toInt();
                if (pct < 0 || pct > 100) {
                    Serial.println("  ERROR: duty must be 0-100.");
                } else {
                    pwmSetChannel(PWM_CH_CW, (uint8_t)pct);
                    Serial.printf("  CW at %d%%\n", pct);
                }
            } else if (buf.startsWith("ccw ") || buf.startsWith("CCW ")) {
                int pct = buf.substring(4).toInt();
                if (pct < 0 || pct > 100) {
                    Serial.println("  ERROR: duty must be 0-100.");
                } else {
                    pwmSetChannel(PWM_CH_CCW, (uint8_t)pct);
                    Serial.printf("  CCW at %d%%\n", pct);
                }
            } else if (buf.length() > 0) {
                Serial.printf("  Unknown: '%s'  (cw / ccw / off / exit)\n", buf.c_str());
            }

            buf = "";
            Serial.print("pwm> ");
        } else {
            Serial.print(c);
            buf += c;
        }
    }
}



// ============================================================
//  DRILL SEQUENCE
// ============================================================
// Returns RC_OK (0) on success, RC_MOVE_UNSAFE (1) if stuck at any point.
//
// Sequence:
//   1. Spin up drill (CW) for DRILL_STARTUP_MS
//   2. Feed to max range (checked move) -- abort on stuck
//   3. Reverse drill (CCW) for DRILL_STARTUP_MS
//   4. Return to home offset (checked move) -- abort on stuck
//   5. Stop drill
//





int doDrill() {
    if (!isHomed || !calValid) return RC_MOVE_UNSAFE;


    // 0. Go to ready position
    moveAbsolute(DRILLING_START, true);


    // 1. Spin up drill CW
    pwmSetChannel(PWM_CH_CCW, DRILL_DUTY_PCT);
    delay(DRILL_STARTUP_MS);

    // 2. Feed to max range
    bool ok = moveAbsoluteCheckSpeed(calData.range, DRILLING_MOVE_SPEED);
    if (!ok) {
        bothOff();
        return RC_MOVE_UNSAFE;
    }

    // 3. Reverse drill CCW
    bothOff();
    delay(DRILL_STARTUP_MS);
    pwmSetChannel(PWM_CH_CW, DRILL_DUTY_PCT/3);
    delay(DRILL_STARTUP_MS);

    // 4. Return to home offset
    ok = moveAbsoluteCheck(DRILLING_START, true);
    if (!ok) {
        bothOff();
        return RC_MOVE_UNSAFE;
    }

    // 5. Stop drill
    bothOff();

    //6.  move home
    ok = moveAbsoluteCheck(calData.offset, true);
    if (!ok) {
        bothOff();
        return RC_MOVE_UNSAFE;
    }

    return RC_OK;
}




// ============================================================
//  HELP MENU
// ============================================================
#define L(txt) Serial.println("||  " txt "  ||")
#define SEP()  Serial.println("||                                                                      ||")

void printHelp() {
    Serial.println();
    Serial.println("++======================================================================++");
    Serial.println("||       S T E P P E R   T E R M I N A L   --   C O M M A N D S        ||");
    Serial.println("++======================================================================++");
    SEP();
    L("MOTION                                                                ");
    L("  home                 run homing sequence                            ");
    L("  pos                  print current position (steps)                 ");
    L("  sw                   monitor home switch  (any key = exit)          ");
    SEP();
    L("  step+                single step CW   (toward home / up)           ");
    L("  step-                single step CCW  (away from home / down)      ");
    SEP();
    L("  rel  <steps>         relative move  (+ away from home, - toward)   ");
    L("  abs  <steps>         absolute move to position  (requires home)    ");
    SEP();
    L("RANGE CALIBRATION                                                     ");
    L("  cal                  show all stored calibration values             ");
    L("  set offset <steps>   offset: switch trigger -> software zero        ");
    L("  set range  <steps>   range : software zero  -> end of travel        ");
    L("  cal save             write offset + range to EEPROM                 ");
    L("  cal clear            erase ALL calibration from EEPROM             ");
    SEP();
    L("FEEDBACK CALIBRATION                                                  ");
    L("  fb                   live feedback sensor monitor  (any key = exit) ");
    L("  cal-fb               sweep and fit linear model  (saves to EEPROM) ");
    SEP();
    L("PWM TEST                                                              ");
    L("  pwm                  enter PWM test mode  (cw/ccw/off/exit)        ");
    L("  doDrill              test full drill cycle                        ");
    SEP();
    L("  help                 show this menu                                 ");
    SEP();
    Serial.println("++======================================================================++");
    Serial.println();
}

#undef L
#undef SEP

// ============================================================
//  COMMAND PARSER
// ============================================================
void handleCommand(const String &raw) {
    String cmd = raw;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("home")) {
        doHome();

    } else if (cmd.equalsIgnoreCase("pos")) {
        Serial.printf("currentPos  : %ld steps%s\n",
                      currentPos, isHomed ? "" : "  (NOT HOMED)");
        if (calValid && isHomed) {
            long swZero = currentPos - (long)calData.offset;
            Serial.printf("sw-zero pos : %ld steps  (after offset)\n", swZero);
        }

    } else if (cmd.equalsIgnoreCase("sw")) {
        monitorSwitch();

    } else if (cmd.equalsIgnoreCase("step+")) {
        if (homeTriggered()) {
            Serial.println("ERROR: home switch already active. Cannot step CW.");
        } else {
            Serial.println("Single step CW");
            digitalWrite(DIR_GPIO, HIGH);
            sendStep(LOW_SPEED);
            currentPos--;
        }

    } else if (cmd.equalsIgnoreCase("step-")) {
        Serial.println("Single step CCW");
        digitalWrite(DIR_GPIO, LOW);
        sendStep(LOW_SPEED);
        currentPos++;

    } else if (cmd.startsWith("rel ") || cmd.startsWith("REL ")) {
        String arg = cmd.substring(4);
        arg.trim();
        long steps = arg.toInt();
        if (steps == 0 && arg != "0") {
            Serial.println("ERROR: invalid argument.  Usage:  rel <steps>");
        } else {
            Serial.printf("Relative move: %+ld steps\n", steps);
            moveRelative(-steps);
        }

    } else if (cmd.startsWith("abs ") || cmd.startsWith("ABS ")) {
        String arg = cmd.substring(4);
        arg.trim();
        long target = arg.toInt();
        if (target < 0) {
            Serial.println("ERROR: absolute position must be >= 0.");
        } else {
            moveAbsolute(target, true);
        }


    } else if (cmd.startsWith("cabs ") || cmd.startsWith("CABS ")) {
        String arg = cmd.substring(4);
        arg.trim();
        long target = arg.toInt();
        if (target < 0) {
            Serial.println("ERROR: absolute position must be >= 0.");
        } else {
            moveAbsoluteCheck(target, true);
        }

    } else if (cmd.equalsIgnoreCase("cal")) {
        printCalibration();

    } else if (cmd.equalsIgnoreCase("cal save")) {
        saveCalibration();

    } else if (cmd.equalsIgnoreCase("cal clear")) {
        clearCalibration();

    } else if (cmd.startsWith("set offset ") || cmd.startsWith("SET OFFSET ")) {
        String arg = cmd.substring(11);
        arg.trim();
        long val = arg.toInt();
        if (val < 0 || (val == 0 && arg != "0")) {
            Serial.println("ERROR: offset must be >= 0.  Usage:  set offset <steps>");
        } else {
            calData.offset = (uint32_t)val;
            Serial.printf("offset = %lu  (not yet saved -- run 'cal save')\n",
                          calData.offset);
        }

    } else if (cmd.startsWith("set range ") || cmd.startsWith("SET RANGE ")) {
        String arg = cmd.substring(10);
        arg.trim();
        unsigned long val = strtoul(arg.c_str(), nullptr, 10);
        if (val == 0 && arg != "0") {
            Serial.println("ERROR: range must be > 0.  Usage:  set range <steps>");
        } else {
            calData.range = val;
            Serial.printf("range  = %lu  (not yet saved -- run 'cal save')\n",
                          calData.range);
        }

    } else if (cmd.equalsIgnoreCase("cal-fb")) {
        calibrateFeedback();

    } else if (cmd.equalsIgnoreCase("fb")) {
        feedbackSensor();

    } else if (cmd.equalsIgnoreCase("pwm")) {
        pwmTest();

    } else if (cmd.equalsIgnoreCase("help")) {
        printHelp();

    }else if (cmd.equalsIgnoreCase("doDrill")) {
        Serial.println(doDrill());

    } else {
        Serial.printf("Unknown command: '%s'  --  type 'help' for options.\n",
                      cmd.c_str());
    }
}

// ============================================================
//  SETUP & LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    EEPROM.begin(EEPROM_SIZE);

    pinMode(EN_PIN,   OUTPUT);
    pinMode(DIR_GPIO, OUTPUT);
    pinMode(HOME_PIN, INPUT_PULLUP);

    digitalWrite(EN_PIN,   HIGH);
    digitalWrite(DIR_GPIO, HIGH);

    rmtStepperInit();
    pwmInit();
    bothOff();
    feedbackSensorInit();

    Serial.println("\n\nStepper Terminal  --  Calibration Edition");
    Serial.printf("Home switch : %s\n", homeTriggered() ? "TRIGGERED" : "open");
    loadCalibration();
    printHelp();
    Serial.print("> ");
}

String inputBuf = "";

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            Serial.println();
            handleCommand(inputBuf);
            inputBuf = "";
            Serial.print("> ");
        } else {
            Serial.print(c);
            inputBuf += c;
        }
    }
}
