/*
 * A.E.G.I.S. firmware configuration: pin map and tuned constants.
 * Electrical values come from engineering/AEGIS_Design_Calculations.md.
 *
 * Pin map rules for ESP32-S3-DevKitC-1 (WROOM-1 module):
 *   - GPIO 19/20 are USB, 43/44 are the UART0 console: unused.
 *   - GPIO 26-37 are flash / octal PSRAM on N8R8 / N16R8 modules: unused.
 *   - GPIO 0 and 45 are boot straps: unused.
 *   - GPIO 46 is a strap with an internal pull-down. It drives TB6612 STBY, so
 *     both drivers stay in standby (motors off) through reset and boot.
 *   - TB6612 PWMA/PWMB are tied to 3V3 on the harness; speed is PWM'd on
 *     AIN1/AIN2 (drive/brake, slow decay), which saves 4 GPIOs.
 *   - The right ToF has XSHUT tied high; only front and left need a GPIO.
 */
#pragma once

#include "driver/gpio.h"
#include "aegis_params.h"   /* tunables shared with the simulator */

/* ---- I2S: two INMP441 on one bus (L/R pin: left mic GND, right mic VDD) */
#define PIN_I2S_BCLK        GPIO_NUM_4
#define PIN_I2S_WS          GPIO_NUM_5
#define PIN_I2S_DIN         GPIO_NUM_6

/* ---- I2C: 3x VL53L0X + MPU-6050, 400 kHz, 2.2 k pull-ups on the harness */
#define PIN_I2C_SDA         GPIO_NUM_8
#define PIN_I2C_SCL         GPIO_NUM_9
#define I2C_FREQ_HZ         400000
#define PIN_TOF_XSHUT_FRONT GPIO_NUM_10
#define PIN_TOF_XSHUT_LEFT  GPIO_NUM_11
#define TOF_ADDR_FRONT      0x30
#define TOF_ADDR_LEFT       0x31
#define TOF_ADDR_RIGHT      0x32
#define MPU6050_ADDR        0x68

/* ---- Motors: TB6612 IN1/IN2 per wheel (index order FL, RL, FR, RR) ------ */
enum { WHEEL_FL = 0, WHEEL_RL = 1, WHEEL_FR = 2, WHEEL_RR = 3, WHEEL_COUNT = 4 };
#define PIN_MOTOR_IN1       {GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_18}
#define PIN_MOTOR_IN2       {GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_17, GPIO_NUM_21}
#define MOTOR_INVERT        {false, false, true, true}   /* right side mirrored */
#define PIN_MOTOR_STBY      GPIO_NUM_46
#define MOTOR_PWM_HZ        20000                         /* above audio band */

/* ---- Encoders (A, B) ---------------------------------------------------- */
#define PIN_ENC_A           {GPIO_NUM_38, GPIO_NUM_40, GPIO_NUM_42, GPIO_NUM_48}
#define PIN_ENC_B           {GPIO_NUM_39, GPIO_NUM_41, GPIO_NUM_47, GPIO_NUM_7}

/* ---- Gripper servo (MG946R on the 5.0 V servo rail) --------------------- */
#define PIN_SERVO           GPIO_NUM_3

/* ---- ESP32-CAM link (UART1, RX only) ------------------------------------ */
#define PIN_CAM_RX          GPIO_NUM_2
#define CAM_UART_BAUD       115200

/* ---- Battery sense: 100k / 22k divider on GPIO1 (ADC1_CH0) ------------- */
#define PIN_VBAT_ADC        GPIO_NUM_1
#define VBAT_DIVIDER        ((100.0f + 22.0f) / 22.0f)
