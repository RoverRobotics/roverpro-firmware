#include "flipper.h"
#include "main.h"
#include "motor.h"
#include "analog.h"

// invalid flipper pot thresholds.  These are very wide because the flipper pots are on a
// different 3.3V supply than the PIC If the flipper pot is below this threshold, it is invalid
#define LOW_POT_THRESHOLD 33
// If the flipper pot is above this threshold, it is invalid
#define HIGH_POT_THRESHOLD 990

#define FLIPPER_POT_OFFSET (-55)

static uint16_t get_flipper_angle(uint16_t pot_1_value, uint16_t pot_2_value);
static uint16_t return_combined_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value);
/// returns the given angle, normalized to between 0 and 360
uint16_t wrap_angle(int16_t value) { return (uint16_t)((value % 360 + 360) % 360); }

void flipper_feedback_calibrate() {
    MotorChannel c;

    // Coast all the motors
    for (EACH_MOTOR_CHANNEL(c))
        motor_update(c, MOTOR_FLAG_COAST, 0);

    g_settings.flipper.is_calibrated = true;
    g_settings.flipper.angle_offset = return_combined_pot_angle(g_state.analog.flipper_sensors[0],
                                                                g_state.analog.flipper_sensors[1]);

    Settings s = settings_load();
    s.flipper = g_settings.flipper;
    settings_save(&s);

    // don't do anything again ever
    while (1) {
        __builtin_clrwdt();
    }
}

void tick_flipper_feedback() {
    uint16_t pot1 = g_state.analog.flipper_sensors[0];
    uint16_t pot2 = g_state.analog.flipper_sensors[1];
    // update flipper motor position
    g_state.drive.flipper_angle = get_flipper_angle(pot1, pot2);
}

uint16_t return_combined_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value) {
    int combined_pot_angle = 0;
    int pot_angle_1 = 0;
    int pot_angle_2 = 0;
    int temp1 = 0;
    int temp2 = 0;
    float scale_factor = 0;
    int temp_pot1_value = 0;

    // correct for pot 2 turning the opposite direction
    pot_2_value = 1023 - pot_2_value;

    // if both pot values are invalid
    if (((pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD)) &&
        ((pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD))) {
        return 0xffff;
    }
    // if pot 1 is out of linear range
    else if ((pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD)) {
        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        combined_pot_angle = pot_2_value * .326 + 13.35;
    }
    // if pot 2 is out of linear range
    else if ((pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD)) {
        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        // 13.35 degrees + 45 degrees = 58.35 degrees
        combined_pot_angle = (int)pot_1_value * .326 + 13.35 + FLIPPER_POT_OFFSET;

    }
    // if both pot 1 and pot 2 values are valid
    else {
        // figure out which one is closest to the end of range
        temp1 = pot_1_value - 512;
        temp2 = pot_2_value - 512;

        // offset, so that both pot values should be the same
        // FLIPPER_POT_OFFSET/333.33*1023 = 168.8 for 55 degrees
        temp_pot1_value = pot_1_value - 168.8;

        pot_angle_1 = wrap_angle(temp_pot1_value * .326 + 13.35);
        pot_angle_2 = wrap_angle(pot_2_value * .326 + 13.35);

        // if pot1 is closer to the end of range
        if (abs(temp1) > abs(temp2)) {
            scale_factor = (512 - abs(temp1)) / 512.0;
            // combined_pot_value = (pot_1_value*scale_factor + pot_2_value*(1-scale_factor));
            combined_pot_angle = pot_angle_1 * scale_factor + pot_angle_2 * (1 - scale_factor);

        }
        // if pot2 is closer to the end of range
        else {
            scale_factor = (512 - abs(temp2)) / 512.0;
            // combined_pot_value = (pot_2_value*scale_factor + pot_1_value*(1-scale_factor));
            combined_pot_angle = pot_angle_2 * scale_factor + pot_angle_1 * (1 - scale_factor);
        }

        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        // combined_pot_angle = combined_pot_value*.326+13.35;
    }

    combined_pot_angle = wrap_angle(combined_pot_angle);

    return (unsigned int)combined_pot_angle;
}

uint16_t get_flipper_angle(uint16_t pot_1_value, uint16_t pot_2_value) {
    uint16_t combined_pot_angle = 0;
    uint16_t calibrated_pot_angle = 0;

    combined_pot_angle = return_combined_pot_angle(pot_1_value, pot_2_value);

    // special case -- invalid reading
    if (combined_pot_angle == 10000)
        return 10000;
    else if (combined_pot_angle == 0xffff)
        return 0xffff;

    // if calibration didn't work right, return angle with no offset
    if (!g_settings.flipper.is_calibrated) {
        return combined_pot_angle;
    } else {
        uint16_t flipper_angle_offset = g_settings.flipper.angle_offset;
        calibrated_pot_angle = wrap_angle(combined_pot_angle - flipper_angle_offset);
        return calibrated_pot_angle;
    }
}
