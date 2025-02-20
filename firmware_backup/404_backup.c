#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/adc/adc1.h"
#include "mcc_generated_files/pwm/sccp1.h"
#include "mcc_generated_files/pwm/sccp3.h"
#include "mcc_generated_files/pwm/sccp2.h"
#include "mcc_generated_files/pwm/sccp4.h"
#include "mcc_generated_files/pwm/sccp5.h"
#include "mcc_generated_files/pwm/sccp6.h"

#define FCY 4000000UL  // Assuming Fosc = 8 MHz, FCY = Fosc / 2

#include <libpic30.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define TABLE_SIZE 512  // Define the number of points in the sine table
#define PWM_MAX 4095    // Max PWM value for 12-bit resolution
#define M_PI 3.14159265358979323846
#define PWM_FREQUENCY 1000  // Frequency that the PWM was set to (Hertz)

uint16_t potValue;
uint16_t dutyCycle;
bool system_running = false; // System starts in "stopped" mode

uint16_t sine_table[TABLE_SIZE];
uint16_t phase_offset_a_high = 0;
uint16_t phase_offset_b_high = TABLE_SIZE / 3;
uint16_t phase_offset_c_high = (2 * TABLE_SIZE) / 3;
uint16_t phase_offset_a_low;
uint16_t phase_offset_b_low;
uint16_t phase_offset_c_low;

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (uint16_t)(((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);
}

// Function to initialize the sine table
void initialize_sine_table() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float angle = (2 * M_PI * i) / TABLE_SIZE;
        sine_table[i] = (uint16_t)((sin(angle) * 0.5 + 0.5) * PWM_MAX);
    }
}

// Function to calculate step increment
uint16_t calculate_step_increment(uint16_t potValue) {
    uint16_t target_freq = map(potValue, 0, 4096, 10, 60);
    printf("Target Frequency: %u\n\r", target_freq);
    return (uint16_t)((TABLE_SIZE * target_freq) / PWM_FREQUENCY);
}

// Function to set three-phase PWM signals
void set_three_phase_pwm(uint16_t potValue) {
    uint16_t duty_a = sine_table[phase_offset_a_high % TABLE_SIZE];
    uint16_t duty_b = sine_table[phase_offset_b_high % TABLE_SIZE];
    uint16_t duty_c = sine_table[phase_offset_c_high % TABLE_SIZE];

    uint16_t duty_a_low = sine_table[phase_offset_a_low % TABLE_SIZE];
    uint16_t duty_b_low = sine_table[phase_offset_b_low % TABLE_SIZE];
    uint16_t duty_c_low = sine_table[phase_offset_c_low % TABLE_SIZE];

    duty_a = (uint16_t)(0.8 * duty_a + 0.1 * PWM_MAX);
    duty_b = (uint16_t)(0.8 * duty_b + 0.1 * PWM_MAX);
    duty_c = (uint16_t)(0.8 * duty_c + 0.1 * PWM_MAX);

    duty_a_low = (uint16_t)(0.8 * duty_a_low + 0.1 * PWM_MAX);
    duty_b_low = (uint16_t)(0.8 * duty_b_low + 0.1 * PWM_MAX);
    duty_c_low = (uint16_t)(0.8 * duty_c_low + 0.1 * PWM_MAX);

    PWM1_DutyCycleSet(system_running ? duty_a : 0);
    PWM2_DutyCycleSet(system_running ? duty_b : 0);
    PWM3_DutyCycleSet(system_running ? duty_c : 0);
    PWM4_DutyCycleSet(system_running ? duty_a_low : 0);
    PWM5_DutyCycleSet(system_running ? duty_b_low : 0);
    PWM6_DutyCycleSet(system_running ? duty_c_low : 0);

    uint16_t step_increment = calculate_step_increment(potValue);
    phase_offset_a_high = (phase_offset_a_high + step_increment) % TABLE_SIZE;
    phase_offset_b_high = (phase_offset_b_high + step_increment) % TABLE_SIZE;
    phase_offset_c_high = (phase_offset_c_high + step_increment) % TABLE_SIZE;
    phase_offset_a_low = (phase_offset_a_low + step_increment) % TABLE_SIZE;
    phase_offset_b_low = (phase_offset_b_low + step_increment) % TABLE_SIZE;
    phase_offset_c_low = (phase_offset_c_low + step_increment) % TABLE_SIZE;
}

// Function to check button press and toggle system state
void check_button_press() {
    static bool button_pressed = false;
    
    if (BUTTON1_GetValue() == 0) {  // Button pressed (active-low)
        __delay_ms(50);  // Debounce delay
        if (BUTTON1_GetValue() == 0 && !button_pressed) {
            system_running = !system_running;  // Toggle system state
            button_pressed = true;  // Prevent multiple toggles
            printf("System %s\n\r", system_running ? "Started" : "Stopped");
        }
    } else {
        button_pressed = false;  // Reset flag when button is released
    }
}

int main(void) {
    SYSTEM_Initialize();
    initialize_sine_table();

    phase_offset_a_low = phase_offset_a_high + (TABLE_SIZE / 2);
    phase_offset_b_low = phase_offset_b_high + (TABLE_SIZE / 2);
    phase_offset_c_low = phase_offset_c_high + (TABLE_SIZE / 2);

    __delay_ms(25);

    printf("Hello, World!\r\n");

    while (1) {
        check_button_press();  // Check if button is pressed to toggle system

        if (system_running) {
            if (ADC1_IsConversionComplete(POT_PIN)) {
                potValue = ADC1_ConversionResultGet(POT_PIN);
                printf("Pot: %d, ", potValue);
                set_three_phase_pwm(potValue);
            } else {
                printf("ADC conversion not complete.\r\n");
            }
        }
    }
}

