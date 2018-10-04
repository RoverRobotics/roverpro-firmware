#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2c.h"
#include "interrupt_switch.h"
#include "testing.h"
#include "debug_uart.h"

void switched_sensor_wires(void);
void pulse_power_bus(void);
void force_overcurrent(void);
void current_display(void);
void int_to_decimal_string(unsigned int input, char *output);

// gets called after board is initialized.=
void test_function(void) {

    // force_overcurrent();
    // current_display();
}

// try to drive the motors as hard as possible
// without killing the battery.
void motor_stress_test(void) {

    unsigned int i;
    unsigned char direction_flag = 0;
    block_ms(5000);

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);

    M1_COAST = Clear_ActiveLO;
    M1_DIR = HI;
    M1_BRAKE = Clear_ActiveLO;
    M1_MODE = 1;

    M2_COAST = Clear_ActiveLO;
    M2_DIR = HI;
    M2_BRAKE = Clear_ActiveLO;
    M2_MODE = 1;

    M3_COAST = Clear_ActiveLO;
    M3_DIR = HI;
    M3_BRAKE = Clear_ActiveLO;
    M3_MODE = 1;

    while (1) {
        /*	M2_DIR=LO;
                OC1R=2000;
                OC2R=2000;
                block_ms(1000);
                OC1R=0;
                OC2R=0;
                block_ms(2000);*/
        if (direction_flag) {
            direction_flag = 0;
            M1_DIR = HI;
            M2_DIR = HI;
            M3_DIR = HI;
        } else {
            direction_flag = 1;
            M1_DIR = LO;
            M2_DIR = LO;
            M3_DIR = LO;
        }

        for (i = 0; i <= 2000; i += 100) {
            OC1R = i;
            OC2R = i;
            OC3R = i;
            block_ms(30);
            ClrWdt();
        }

        if (direction_flag) {
            M1_DIR = HI;
            M2_DIR = HI;
            M3_DIR = HI;
        } else {
            M1_DIR = LO;
            M2_DIR = LO;
            M3_DIR = LO;
        }

        // for 2 motors installed, with tracks, 10ms doesn't reset, 20ms does, 15ms does
        block_ms(20);
        OC1R = 0;
        OC2R = 0;
        OC3R = 0;
    }
}

void pulse_power_bus(void) {

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);

    M1_COAST = Clear_ActiveLO;
    M1_DIR = HI;
    M1_BRAKE = Clear_ActiveLO;
    M1_MODE = 1;

    M2_COAST = Clear_ActiveLO;
    M2_DIR = HI;
    M2_BRAKE = Clear_ActiveLO;
    M2_MODE = 1;

    M3_COAST = Clear_ActiveLO;
    M3_DIR = HI;
    M3_BRAKE = Clear_ActiveLO;
    M3_MODE = 1;

    OC1R = 0;
    OC2R = 0;
    OC3R = 0;

    block_ms(1000);

    OC1R = 2000;
    OC2R = 2000;
    OC3R = 2000;

    while (1) {
        Cell_Ctrl(Cell_A, Cell_ON);
        Cell_Ctrl(Cell_B, Cell_ON);

        block_ms(15);

        Cell_Ctrl(Cell_A, Cell_OFF);
        Cell_Ctrl(Cell_B, Cell_OFF);

        block_ms(1000);
    }
}

void switched_sensor_wires(void) {
    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);

    M1_COAST = Clear_ActiveLO;
    M1_DIR = HI;
    M1_BRAKE = Clear_ActiveLO;
    M1_MODE = 1;

    M2_COAST = Clear_ActiveLO;
    M2_DIR = HI;
    M2_BRAKE = Clear_ActiveLO;
    M2_MODE = 1;

    M3_COAST = Clear_ActiveLO;
    M3_DIR = HI;
    M3_BRAKE = Clear_ActiveLO;
    M3_MODE = 1;

    while (1) {

        OC1R = 2000;
        OC2R = 2000;
        OC3R = 2000;

        // 10ms doesn't kill the battery, but 20ms does
        block_ms(10);

        OC1R = 0;
        OC2R = 0;
        OC3R = 0;

        block_ms(60);
    }
}

void int_to_decimal_string(unsigned int input, char *output) {
    unsigned char ten_thousands, thousands, hundreds, tens, ones = 0;

    ten_thousands = input / 10000;
    thousands = (input - ten_thousands * 10000) / 1000;
    hundreds = (input - ten_thousands * 10000 - thousands * 1000) / 100;
    tens = (input - ten_thousands * 10000 - thousands * 1000 - hundreds * 100) / 10;
    ones = (input - ten_thousands * 10000 - thousands * 1000 - hundreds * 100 - tens * 10);

    output[0] = ten_thousands + 48;
    output[1] = thousands + 48;
    output[2] = hundreds + 48;
    output[3] = tens + 48;
    output[4] = ones + 48;
}

void current_display(void) {
    // Note:  CELL A, for the purpose of the display, is the cell closest to the edge of the board
    char hex_string_A[6], hex_string_B[6];
    char percentage_string[5];
    char current_display[26] = "zzzzzz   zzzzzz  zzzzz%\r\n";
    long temp1, temp2;
    unsigned int i;
    unsigned int A_Current, B_Current = 0;
    unsigned int percentage;

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);
    /*  block_ms(100);
            Cell_Ctrl(Cell_A,Cell_ON);
      block_ms(100);
            Cell_Ctrl(Cell_A,Cell_OFF);*/

    M1_COAST = Clear_ActiveLO;
    M1_DIR = HI;
    M1_BRAKE = Clear_ActiveLO;
    M1_MODE = 1;

    M2_COAST = Clear_ActiveLO;
    M2_DIR = HI;
    M2_BRAKE = Clear_ActiveLO;
    M2_MODE = 1;

    M3_COAST = Clear_ActiveLO;
    M3_DIR = HI;
    M3_BRAKE = Clear_ActiveLO;
    M3_MODE = 1;

    block_ms(1000);
    ClrWdt();

    temp1 = 0;
    temp2 = 0;
    for (i = 0; i < SampleLength; i++) {
        temp1 += Cell_A_Current[i];
        temp2 += Cell_B_Current[i];
    }

    A_Current = temp1 / SampleLength;
    B_Current = temp2 / SampleLength;

    send_debug_uart_string("Baseline current: ", 18);
    block_ms(10);

    int_to_string(A_Current, (char *)hex_string_A);
    int_to_string(B_Current, (char *)hex_string_B);

    for (i = 0; i < 6; i++) {
        current_display[i] = hex_string_A[i];
        current_display[i + 9] = hex_string_B[i];
    }

    send_debug_uart_string(current_display, 17);
    block_ms(10);
    send_debug_uart_string("\r\n", 2);
    block_ms(10);

    OC1R = 2000;
    OC2R = 2000;
    OC3R = 2000;

    while (1) {
        ClrWdt();
        temp1 = 0;
        temp2 = 0;
        for (i = 0; i < SampleLength; i++) {
            temp1 += Cell_A_Current[i];
            temp2 += Cell_B_Current[i];
        }

        A_Current = temp1 / SampleLength;
        B_Current = temp2 / SampleLength;

        percentage = abs(A_Current - B_Current) * 100 / ((A_Current + B_Current) / 2);

        int_to_string(A_Current, (char *)hex_string_A);
        int_to_string(B_Current, (char *)hex_string_B);
        int_to_decimal_string(percentage, (char *)percentage_string);

        for (i = 0; i < 6; i++) {
            current_display[i] = hex_string_A[i];
            current_display[i + 9] = hex_string_B[i];
        }

        for (i = 0; i < 5; i++) {
            current_display[i + 17] = percentage_string[i];
        }

        send_debug_uart_string(current_display, 26);
        block_ms(500);
    }
}

void force_overcurrent(void) {
    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);

    M1_COAST = Clear_ActiveLO;
    M1_DIR = HI;
    M1_BRAKE = Clear_ActiveLO;
    M1_MODE = 1;

    M2_COAST = Clear_ActiveLO;
    M2_DIR = HI;
    M2_BRAKE = Clear_ActiveLO;
    M2_MODE = 1;

    M3_COAST = Clear_ActiveLO;
    M3_DIR = HI;
    M3_BRAKE = Clear_ActiveLO;
    M3_MODE = 1;
    OC1R = 0;
    OC2R = 0;
    OC3R = 0;

    ClrWdt();
    OC1R = 200;
    OC2R = 200;
    block_ms(5000);
    ClrWdt();

    OC1R = 2000;
    OC2R = 2000;

    block_ms(1000);

    M1_DIR = LO;
    M2_DIR = LO;

    while (1) {
        ClrWdt();
    }
}
