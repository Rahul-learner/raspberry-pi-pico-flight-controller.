// Include necessary Raspberry Pi Pico SDK headers
#include <stdio.h>        // For standard input/output functions like printf
#include <math.h>         // For mathematical functions like atan2, asin, sqrt
#include <string.h>       // For memset
#include "pico/stdlib.h"  // For general Pico standard library functions (gpio_init, sleep_ms, etc.)
#include "hardware/gpio.h" // For GPIO (General Purpose Input/Output) functions and interrupts
#include "pico/time.h"    // For time-related functions like sleep_us, make_timeout_time_ms
#include "hardware/i2c.h"  // For I2C (Inter-Integrated Circuit) functions
#include "hardware/clocks.h" // For setting CPU frequency (overclocking)
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "PwmIn.pio.h"
#include "stdint.h"

// Define a small epsilon for floating point comparisons
#define PID_EPSILON 1e-6f

// --- Define the GPIO pin for the onboard LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define UART_ID uart0 // For UART connection
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BUAD_RATE 74880

#define MPU6050_I2C_PORT i2c0      // Using I2C0 peripheral
#define MPU6050_SDA_PIN 20         // SDA on GP20 as requested
#define MPU6050_SCL_PIN 21         // SCL on GP21 as requested
#define MPU6050_BAUD_RATE 400000   // Fast mode I2C (400 kHz)

// --- I2C Helper Functions (shared for MPU6050 and HMC5883L) ---
void i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    i2c_write_blocking(MPU6050_I2C_PORT, device_addr, buf, 2, false);
}

void i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    i2c_write_blocking(MPU6050_I2C_PORT, device_addr, &reg_addr, 1, true); // True for repeated start
    i2c_read_blocking(MPU6050_I2C_PORT, device_addr, data, len, false); // False for no repeated start on end
}

//-- PWM signal config --
#define NUM_CHANNELS 4
const uint16_t channel_gpio_pins[NUM_CHANNELS] = {13, 11, 9, 7};
volatile uint32_t channel[4] = {1500, 1500, 1000, 1500};
uint32_t initial_channel[4] = {1500, 1500, 1000, 1500};
const uint64_t PWM_READ_INTERVAL_US = 1000000/50;
uint64_t last_pwm_update = 0;

const float CONTROL_OUT_MIN = -30.0f;
const float CONTROL_OUT_MAX = 30.0f;

float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pwm_in_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = PwmIn_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_clkdiv(&c, 2.0f);  // 250 MHz system clock / 1.0 = full speed

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false); // input

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

float ticks_to_us(uint32_t delta_ticks) {
    // Each tick is 1 cycle at 125MHz = 8ns
    return (int)(0xFFFFFFFF - delta_ticks) * (2.0f / 125.0f); // convert to microseconds
}

// --- MPU6050 I2C Configuration ---

// --- MPU6050 Interrupt Pin ---
#define MPU6050_INT_PIN 15         // Interrupt pin on GP15 as requested

// --- MPU6050 Register Addresses ---
#define MPU6050_ADDR 0x68          // MPU6050 I2C address (AD0 low)
#define MPU6050_PWR_MGMT_1 0x6B    // Power Management 1 register
#define MPU6050_SMPLRT_DIV 0x19    // Sample Rate Divider register
#define MPU6050_CONFIG 0x1A        // Configuration register (DLPF)
#define MPU6050_GYRO_CONFIG 0x1B   // Gyroscope Configuration register
#define MPU6050_ACCEL_CONFIG 0x1C  // Accelerometer Configuration register
#define MPU6050_INT_PIN_CFG 0x37   // Interrupt Pin Configuration register
#define MPU6050_INT_ENABLE 0x38    // Interrupt Enable register
#define MPU6050_ACCEL_XOUT_H 0x3B  // Start of accelerometer data registers
#define MPU6050_GYRO_XOUT_H 0x43   // Start of gyroscope data registers

// --- MPU6050 Sensitivity Scales (based on CONFIG registers) ---
// For MPU6050_ACCEL_CONFIG = 0x00 (2g scale)
#define ACCEL_SCALE_FACTOR 16384.0f // LSB/g for +/- 2g range
// For MPU6050_GYRO_CONFIG = 0x00 (250 deg/s scale)
#define GYRO_SCALE_FACTOR 131.0f    // LSB/(deg/s) for +/- 250 deg/s range

// Global variables for MPU6050 raw data
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

const float GYRO_X_OFFSET = -86.5546f;
const float GYRO_Y_OFFSET = 128.2100f;
const float GYRO_Z_OFFSET = 137.5134f;

// Global variables for filtered data
float roll = 0.0f, pitch = 0.0f, yaw_rate = 0.0f; // Added yaw
volatile bool new_data_ready = false; // Flag set by interrupt handler

// Global variable for tracking time for filter
static uint64_t last_update_us = 0;

// --- MPU6050 Initialization ---
void mpu6050_init() {
    // Wake up MPU6050
    i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    sleep_ms(100); // Give time to power up

    // Set DLPF to 0 (widest bandwidth, 8kHz gyro, 4kHz accel internal sample rate)
    i2c_write_byte(MPU6050_ADDR, MPU6050_CONFIG, 0x00); // DLPF_CFG = 0

    // Set sample rate to (Gyro Output Rate) / (1 + SMPLRT_DIV)
    // To get 2000Hz from 8kHz gyro: SMPLRT_DIV = 3 (0x03)
    i2c_write_byte(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x03); // Set to 3 for 2000Hz output

    // Set Gyroscope Full Scale Range: +/- 250 deg/s
    i2c_write_byte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00); // FS_SEL = 0

    // Set Accelerometer Full Scale Range: +/- 2g
    i2c_write_byte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00); // AFS_SEL = 0

    // Configure interrupt: Data Ready interrupt, active high, push-pull, clear on read
    i2c_write_byte(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x00); // Clear on read for INT_STATUS

    // Enable Data Ready Interrupt
    i2c_write_byte(MPU6050_ADDR, MPU6050_INT_ENABLE, 0x01); // DATA_RDY_EN
}

// --- MPU6050 Read Raw Data ---
void mpu6050_read_raw_data() {
    uint8_t data[14];
    i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);

    // Accelerometer data
    accel_x_raw = (int16_t)(data[0] << 8 | data[1]);
    accel_y_raw = (int16_t)(data[2] << 8 | data[3]);
    accel_z_raw = (int16_t)(data[4] << 8 | data[5]);

    // Temperature data (unused in this example)
    // int16_t temp_raw = (int16_t)(data[6] << 8 | data[7]);

    // Gyroscope data
    gyro_x_raw = (int16_t)(data[8] << 8 | data[9]);
    gyro_y_raw = (int16_t)(data[10] << 8 | data[11]);
    gyro_z_raw = (int16_t)(data[12] << 8 | data[13]);
}

// --- MPU6050 Interrupt Handler ---
void mpu6050_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        new_data_ready = true; // Set flag to process data in main loop
    }
}
//---------------------------------------------------------------------------------------------
// --- MOTOR CONFIG ---
//gpio_motor1 = 2 front left, clockwise
//gpio_motor2 = 3 front right, counter clockwise
//gpio_motor3 = 4 rear left, counter clockwise
//gpio_motor4 = 5 rear right, clockwise
#define MOTOR1_PWM_PIN 2 // Example: GP2 (PWM Slice 0, Channel A)
#define MOTOR2_PWM_PIN 3 // Example: GP3 (PWM Slice 0, Channel B)
#define MOTOR3_PWM_PIN 4 // Example: GP4 (PWM Slice 1, Channel A)
#define MOTOR4_PWM_PIN 5 // Example: GP5 (PWM Slice 1, Channel B)
#define PWM_FREQUENCY_HZ 490
#define PWM_WRAP_VALUE 2040 // For 490Hz, Period = 1,000,000 / 490 = 2040
#define ESC_MIN_PULSE_US 1000
#define ESC_MAX_PULSE_US 2000
const uint64_t MOTOR_PWM_INTERVAL_US = 1000000 / PWM_FREQUENCY_HZ;
uint64_t last_motor_pwm_update = 0;

// Function to set the PWM pulse width for an ESC in microseconds
void set_esc_pulse_us(uint gpio, uint32_t pulse_us) {
    // Ensure pulse_us in within the valid ESC range
    if (pulse_us < ESC_MIN_PULSE_US) {
        pulse_us = ESC_MIN_PULSE_US;
    }else if (pulse_us > ESC_MAX_PULSE_US) {
        pulse_us = ESC_MAX_PULSE_US;
    }
    // This function efficiently updates the PWM duty cycle for the specified GPIO.
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio), pulse_us);
}

// Function to initialize a single PWM channel for ESC
void init_esc_pwm_channel(uint gpio_pin) {
    // Get the slice and channel for the chosen GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel_num = pwm_gpio_to_channel(gpio_pin);

    // Tell GPIO pin to connect to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Configure PWM for this slice (only needs to be done once peer slice)
    pwm_config config = pwm_get_default_config();

    // Calculate the clock divider
    // clk_div = System_Clock / (PWM_Frequency * (Wrap_Value + 1))
    // We want the PWM_WRAP_VALUE to directly represent microseconds,
    // so the PWM_FREQUENCY_HZ * (PWM_WRAP_VALUE + 1) should be 1,000,000 (1 MHz)
    // This makes each count of the PWM counter equal to 1 microsecond.
    // So, clk_div = System_Clock / 1,000,000
    float clock_divider = (float)clock_get_hz(clk_sys) / 1000000.0f; // Each tick = 1us
    pwm_config_set_clkdiv(&config, clock_divider);

    // Set the wrap value (TOP)
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE); // Counter goes from 0 to PWM_WRAP_VALUE

    // Initialize PWM slice with the configured settings
    // Only enable if this is the first channel on this slice being initialized
    // or if you want to explicitly enable it here.
    pwm_init(slice_num, &config, true); // true to enable the slice immediately

    printf("PWM configured on GP%d (Slice %d, Channel %d) at %d Hz.\n",
           gpio_pin, slice_num, channel_num, PWM_FREQUENCY_HZ);
}

// Function to update the MOTOR PWM
void update_motors_pwm(uint throttle, float roll_control_output, float pitch_control_output, float yaw_control_outptut) {
    uint32_t motor1_speed = throttle + roll_control_output - pitch_control_output + yaw_control_outptut;
    uint32_t motor2_speed = throttle - roll_control_output - pitch_control_output - yaw_control_outptut;
    uint32_t motor3_speed = throttle + roll_control_output + pitch_control_output - yaw_control_outptut;
    uint32_t motor4_speed = throttle - roll_control_output + pitch_control_output + yaw_control_outptut;

    set_esc_pulse_us(MOTOR1_PWM_PIN, motor1_speed);
    set_esc_pulse_us(MOTOR2_PWM_PIN, motor2_speed);
    set_esc_pulse_us(MOTOR3_PWM_PIN, motor3_speed);
    set_esc_pulse_us(MOTOR4_PWM_PIN, motor4_speed);
}
// --- PID Class ------------------------------------------------------------------------------

float desired_roll = 0.0f, desired_pitch = 0.0f, desired_yaw = 0.0f;
float roll_control_output = 0.0f, pitch_control_output = 0.0f, yaw_control_output = 0.0f;

class PIDController {
public:
    // PID gains
    float Kp;
    float Ki;
    float Kd;

    // Output limits
    float output_min;
    float output_max;

    // Integral term limits (anti-windup)
    float integral_min;
    float integral_max;

    // Previous error for derivative calculation
    float prev_error;
    // Accumulated integral error
    float integral_sum;

    // Flag to indicate if the controller has been initialized with a previous error
    bool first_run;

    /**
     * @brief Constructor for the PIDController.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param out_min Minimum allowed output value.
     * @param out_max Maximum allowed output value.
     * @param integral_limit Maximum absolute value for the integral sum (for anti-windup).
     */
    PIDController(float p, float i, float d, float out_min, float out_max, float integral_limit)
        : Kp(p), Ki(i), Kd(d),
          output_min(out_min), output_max(out_max),
          integral_min(-fabs(integral_limit)), integral_max(fabs(integral_limit)),
          prev_error(0.0f), integral_sum(0.0f),
          first_run(true) {
    }

    /**
     * @brief Resets the integral sum and previous error.
     * Call this when the system state changes significantly or on startup.
     */
    void reset() {
        integral_sum = 0.0f;
        prev_error = 0.0f;
        first_run = true;
    }

    void setPID(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    /**
     * @brief Calculates the PID control output.
     * @param setpoint The desired target value.
     * @param process_variable The current measured value from the system.
     * @param dt The time difference (in seconds) since the last calculation.
     * @return The calculated control output.
     */
    float calculate(float setpoint, float process_variable, float dt) {
        // Calculate the error
        float error = setpoint - process_variable;

        // --- Proportional Term ---
        float proportional_term = Kp * error;

        // --- Integral Term ---
        // Only integrate if dt is meaningful and within limits.
        // Also, implement anti-windup: only integrate if output is not saturated
        // or if the error is moving away from the saturation limit.
        if (dt > PID_EPSILON) {
            integral_sum += error * dt;

            // Clamp the integral sum to prevent windup
            if (integral_sum > integral_max) {
                integral_sum = integral_max;
            } else if (integral_sum < integral_min) {
                integral_sum = integral_min;
            }
        }
        float integral_term = Ki * integral_sum;


        // --- Derivative Term ---
        float derivative_term = 0.0f;
        if (first_run) {
            // On the first run, we don't have a previous error, so derivative is zero.
            first_run = false;
        } else if (dt > PID_EPSILON) {
            // Standard derivative on error
            derivative_term = Kd * (error - prev_error) / dt;

            // Alternative: Derivative on Process Variable (DPV)
            // This avoids "derivative kick" if the setpoint changes suddenly.
            // derivative_term = Kd * (process_variable - prev_pv) / dt; // Requires storing prev_pv

            // You might add a low-pass filter to the derivative term if your sensor is noisy
            // e.g., using an exponential moving average or similar.
        }
        prev_error = error; // Store current error for next derivative calculation

        // --- Combine Terms ---
        float control_output = proportional_term + integral_term + derivative_term;

        // --- Clamp Output ---
        if (control_output > output_max) {
            control_output = output_max;
        } else if (control_output < output_min) {
            control_output = output_min;
        }

        return control_output;
    }
};

//---------------------------------------------------------------------------------------------------
// Definitions
#define betaDef		0.041f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//-----------------------------------------------------------------------------------------------------------------------

void toEuler(float& roll_deg, float& pitch_deg) {
    // Pitch (y-axis rotation)
    roll_deg = atan2(2.0f*((q0*q1)+(q2*q3)), 1.0f-(2.0f*((q1*q1)+(q2*q2)))) * (180.0f / M_PI);
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1)
        pitch_deg = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI); // Use 90 degrees if out of range
    else
        pitch_deg = asinf(sinp) * (180.0f / M_PI);
    
}
// Core1 Settings ----------------------------------------------------------------------------
// --- Core1 Serial Command Buffer ---
#define UART_BUFFER_SIZE 103
char uart_buffer[UART_BUFFER_SIZE];
int uart_index = 0;
bool pid_changed = false;
#define TX_BUFFER_SIZE 300
char tx_buffer[TX_BUFFER_SIZE];
const uint64_t TX_SEND_INTERVAL_US = 1000000/20;
uint64_t last_tx_send_us = 0;

float Rp = 0.0f;float Ri = 0.0f;float Rd = 0.0f;
float Pp = 0.0f;float Pi = 0.0f;float Pd = 0.0f;
float Yp = 0.0f;float Yi = 0.0f;float Yd = 0.0f;

/**
 * @brief Processes a complete serial line received by Core1.
 * Parses PID gain values from a specific string format.
 * This function runs on Core1.
 */
void process_received_line_core1() {
    uart_buffer[uart_index] = '\0'; // Null-terminate the received string
    printf("Core1: Received line: '%s'\n", uart_buffer);

    // Check if the command starts with "SET_PID:"
    if (strncmp(uart_buffer, "SET_PID:", 8) == 0) {
        // Create a mutable copy of the buffer because strtok modifies the string
        char temp_buffer[UART_BUFFER_SIZE];
        strncpy(temp_buffer, uart_buffer, UART_BUFFER_SIZE);
        temp_buffer[UART_BUFFER_SIZE - 1] = '\0'; // Ensure null termination

        char *data_start = temp_buffer + 8; // Pointer to the start of the data (after "SET_PID:")
        char *token;
        char *rest = data_start; // Use a temporary pointer for strtok_r (re-entrant strtok)

        printf("Core1: Parsing PID values...\n");

        // Split the string by commas (e.g., "RP:0.0500", "RI:0.0000")
        while ((token = strtok_r(rest, ",", &rest))) {
            char *key_value_pair = token;
            char *key = strtok(key_value_pair, ":");    // Split by colon to get key
            char *value_str = strtok(NULL, ":"); // Get value string

            if (key != NULL && value_str != NULL) {
                float value = atof(value_str); // Convert value string to float

                // Assign the parsed value to the corresponding global variable
                if (strcmp(key, "RP") == 0) { Rp = value; }
                else if (strcmp(key, "RI") == 0) { Ri = value; }
                else if (strcmp(key, "RD") == 0) { Rd = value; }
                else if (strcmp(key, "PP") == 0) { Pp = value; }
                else if (strcmp(key, "PI") == 0) { Pi = value; }
                else if (strcmp(key, "PD") == 0) { Pd = value; }
                else if (strcmp(key, "YP") == 0) { Yp = value; }
                else if (strcmp(key, "YI") == 0) { Yi = value; }
                else if (strcmp(key, "YD") == 0) { Yd = value; }
                else {
                    printf("Core1: Unknown PID key: %s\n", key);
                }
            }
        }
        printf("Core1: PID values updated:\n");
        printf("  RP: %.4f, RI: %.4f, RD: %.4f\n", Rp, Ri, Rd);
        printf("  PP: %.4f, PI: %.4f, PD: %.4f\n", Pp, Pi, Pd);
        printf("  YP: %.4f, YI: %.4f, YD: %.4f\n", Yp, Yi, Yd);
        pid_changed = true;

    } else {
        printf("Core1: Unrecognized command format. Expected 'SET_PID:...' \n");
    }

    // Reset buffer for the next liner[uart_index]= 0;
    memset(uart_buffer, 0, UART_BUFFER_SIZE); // Clear buffer
    uart_index = 0;
}

/**
 * @brief Entry point function for Core1.
 * this function runs on Core1.
 */
void core1_entry() {
    sleep_ms(3000);
    // Initialize UART for Core1
    uart_init(UART_ID, BUAD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true); // Optional: for better responsiveness
    printf("Core1: UART initialized on GP%d (TX) and GP%d (RX) at %d buad.\n", UART_TX_PIN, UART_RX_PIN, BUAD_RATE);
    printf("Core1: Waiting for input...\n");

    // Main loop for Core1: Continuously read UART input
    float current_time_us = time_us_64();
    while (true) {
        if (uart_is_readable(UART_ID)) {
            // Example:SET_PID:RP:0.0500,RI:0.0000,RD:0.0000,PP:0.2030,PI:-1.2050,PD:0.0000,YP:0.0000,YI:0.0000,YD:0.0000
            while (uart_index < UART_BUFFER_SIZE - 1) {
                char c = uart_getc(UART_ID);
                if (c == '\n') {
                    break;
                }
                uart_buffer[uart_index++] = c;
            }
            process_received_line_core1();

        }
        if (channel[2] > 1005) {
            uint64_t current_us = time_us_64();
            if (current_us - last_tx_send_us > TX_SEND_INTERVAL_US) {
                float time = (current_us-current_time_us)/1000000.0f;
                int len = snprintf(tx_buffer, TX_BUFFER_SIZE,
                "Roll:%.4f,Pitch:%.4f,Yaw:%.4f,Desired_Roll:%.2f,Desired_Pitch:%.2f,Desired_Yaw:%.2f,PID_Roll:%.4f,PID_Pitch:%.4f,PID_Yaw:%.4f,T:%f;No\n",
                roll,pitch,yaw_rate,desired_roll,desired_pitch,desired_yaw,roll_control_output,pitch_control_output,yaw_control_output,time);
                if (len < 0 || len >= TX_BUFFER_SIZE) {
                    printf("Warning: UART TX buffer overflow or error!\n");
                }
                // Send it to the uart
                uart_puts(UART_ID, tx_buffer);
                last_tx_send_us = current_us;
            }
        } else {
            current_time_us = time_us_64();
        }

        sleep_ms(1);
    }
}
uint64_t last_print_us = 0;
const uint64_t PRINT_INTERVAL_US = 1000000 / 10;
float roll_bias = 0.0f;
float pitch_bias = 0.0f;
float yaw_bias = 0.0f;
bool first_throttle_on = true;
int main() {
    sleep_ms(100);
    // --- Overclock the CPU to 250 MHz ---
    set_sys_clock_khz(250000, true); // Set system clock to 250 MHz, and wait for it to stabilize

    // Initialize standard I/O for USB serial
    stdio_init_all();
    sleep_ms(3000);
    printf("Started...\n");
    printf("Pico MPU6050 Reader with Mahony started!\n");
    printf("CPU Frequency: %lu kHz\n", clock_get_hz(clk_sys) / 1000);

    // --- Launch Core1 ---
    multicore_launch_core1(core1_entry);
    printf("Core0: Core1 launched.\n");

    // --- Initialize onboard LED ---
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // --- Initializing the PIO PWM reader ---
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &PwmIn_program);
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        pwm_in_program_init(pio, i, offset, channel_gpio_pins[i]);
    }

    // --- Initialize all ESC PWM channels ---
    init_esc_pwm_channel(MOTOR1_PWM_PIN);
    init_esc_pwm_channel(MOTOR2_PWM_PIN);
    init_esc_pwm_channel(MOTOR3_PWM_PIN);
    init_esc_pwm_channel(MOTOR4_PWM_PIN);
    printf("Sending MIN throttle to arm ESCs...\n");
    set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
    set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);

    // --- Initializing PID ---
    PIDController roll_pid(0.0f, 0.0f, 0.0f, -500.0f, 500.0f, 10.0f);
    PIDController pitch_pid(0.0f, 0.0f, 0.0f, -500.0f, 500.0f, 10.0f);
    PIDController yaw_pid(0.0f, 0.0f, 0.0f, -500.0f, 500.0f, 10.0f);

    // --- Initialize I2C ---
    i2c_init(MPU6050_I2C_PORT, MPU6050_BAUD_RATE);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);
    printf("I2C initialized on SDA GP%d, SCL GP%d at %d Hz.\n", MPU6050_SDA_PIN, MPU6050_SCL_PIN, MPU6050_BAUD_RATE);

    // --- Initialize MPU6050 ---
    mpu6050_init();
    printf("MPU6050 initialized.\n");

    // --- Configure MPU6050 Interrupt Pin ---
    gpio_init(MPU6050_INT_PIN);
    gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU6050_INT_PIN); // MPU6050 INT pin is active low, so pull-up is needed
    gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu6050_interrupt_handler);
    printf("MPU6050 Interrupt configured on GP%d.\n", MPU6050_INT_PIN);

    // Perform an initial I2C scan (optional, but good for debugging connections)
    printf("\nPerforming I2C scan to confirm MPU6050 presence...\n");
    printf("Scanning for devices...\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (int addr = 0x00; addr < 0x80; addr++) {
        if (addr % 16 == 0) {
            printf("%02x: ", addr);
        }
        uint8_t rxdata;
        int ret = i2c_write_blocking_until(MPU6050_I2C_PORT, addr, &rxdata, 0, false, make_timeout_time_ms(5));
        if (ret >= 0) {
            printf("%02x ", addr);
        } else {
            printf("-- ");
        }
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    printf("I2C Scan Complete.\n\n");

    // Re-initialize last_update_us after settling
    last_update_us = time_us_64();
    last_pwm_update = time_us_64();
    // --- Main Loop ---
    while (true) {
        // Only process data if the MPU6050 interrupt indicates new data is ready
        if (new_data_ready) {
            // Reset the flag immediately to avoid missing subsequent interrupts
            new_data_ready = false;

            // Read the PWM receiver signal from FIFO
            uint64_t current_pwm_read_time_us = time_us_64();
            if (current_pwm_read_time_us - last_pwm_update >= PWM_READ_INTERVAL_US) {
            for (int i = 0; i < NUM_CHANNELS; ++i) {
                PIO p = pio;
                uint sm = i;
                if (!pio_sm_is_rx_fifo_empty(p, sm)) {
                    uint32_t period_ticks = pio_sm_get(p, sm);
                    float period_us = ticks_to_us(period_ticks);
                    channel[i] = period_us;
                } else {
                    channel[i] = initial_channel[i];
                }

            }
                last_pwm_update = current_pwm_read_time_us;
            }

            // check if the throttle is on
            if (channel[2] > 1005) {
                if (first_throttle_on) {
                    while (channel[2] > 1051) {
                        gpio_put(PICO_DEFAULT_LED_PIN, 1);
                        sleep_ms(200);
                        PIO p = pio;
                        uint sm = 2;
                        if (!pio_sm_is_rx_fifo_empty(p, sm)) {
                            uint32_t period_ticks = pio_sm_get(p, sm);
                            float period_us = ticks_to_us(period_ticks);
                            channel[2] = period_us;
                        } else {
                            channel[2] = 1000;
                        }
                        set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
                        set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
                        set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
                        set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
                        gpio_put(PICO_DEFAULT_LED_PIN, 0);
                        printf("Throttle is high! Throttle:%i\n", channel[2]);
                        sleep_ms(200);
                    }
                    first_throttle_on = false;
                    last_update_us = time_us_64();
                } else {
                    // Read raw sensor data
                    mpu6050_read_raw_data();
                    // Calculate time difference for filter (in seconds)
                    uint64_t current_time_us = time_us_64();
                    float dt_s = (float)(current_time_us - last_update_us) / 1000000.0f;
                    last_update_us = current_time_us;
                    // Convert raw accelerometer data to g's
                    float ax = (float)accel_x_raw / ACCEL_SCALE_FACTOR;
                    float ay = (float)accel_y_raw / ACCEL_SCALE_FACTOR;
                    float az = (float)accel_z_raw / ACCEL_SCALE_FACTOR;

                    // Apply offsets to raw gyroscope data and convert to rad/s
                    float gx = (((float)gyro_x_raw - GYRO_X_OFFSET) / GYRO_SCALE_FACTOR) * (M_PI / 180.0f);
                    float gy = (((float)gyro_y_raw - GYRO_Y_OFFSET) / GYRO_SCALE_FACTOR) * (M_PI / 180.0f);
                    float gz = (((float)gyro_z_raw - GYRO_Z_OFFSET) / GYRO_SCALE_FACTOR) * (M_PI / 180.0f);
                    // Update the filter (call each time you have new sensor data)
                    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt_s);
                    // Get Roll, Pitch, Yaw angles from the filter's estimated quaternion
                    toEuler(roll, pitch);
                    roll += roll_bias;
                    pitch += pitch_bias;
                    yaw_rate = (gz*180.0f/M_PI) + yaw_bias;
                    // calculating PID values
                    desired_roll = map_value(channel[0], ESC_MIN_PULSE_US, ESC_MAX_PULSE_US, CONTROL_OUT_MIN, CONTROL_OUT_MAX) * -1.0f;
                    desired_pitch = map_value(channel[1], ESC_MIN_PULSE_US, ESC_MAX_PULSE_US, CONTROL_OUT_MIN, CONTROL_OUT_MAX);
                    desired_yaw = map_value(channel[3], ESC_MIN_PULSE_US, ESC_MAX_PULSE_US, CONTROL_OUT_MIN, CONTROL_OUT_MAX);
                    roll_control_output = roll_pid.calculate(desired_roll, roll, dt_s);
                    pitch_control_output = pitch_pid.calculate(desired_pitch, pitch, dt_s);
                    yaw_control_output = yaw_pid.calculate(desired_yaw, yaw_rate, dt_s);
                    // updating motor PWM
                    if (current_time_us - last_motor_pwm_update >= MOTOR_PWM_INTERVAL_US) {
                        if (channel[2] >= 1050) {
                            gpio_put(PICO_DEFAULT_LED_PIN, 1);
                            update_motors_pwm(channel[2], roll_control_output, pitch_control_output, yaw_control_output);
                        } else {
                            gpio_put(PICO_DEFAULT_LED_PIN, 0);
                            set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
                            set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
                            set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
                            set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
                        }

                        // Always update after processing
                        last_motor_pwm_update = current_time_us;
                    }
                    // Print the results to serial less frequently to avoid bottlenecking
                    if (current_time_us - last_print_us > PRINT_INTERVAL_US) {
                        printf("dts:%.6f, Roll:%.4f, Pitch:%.4f, Yaw_rate:%.4f, throttle:%i\n", dt_s, roll, pitch, yaw_rate, channel[2]);
                        if (pid_changed == true) {
                            roll_pid.setPID(Rp, Ri, Rd);
                            pitch_pid.setPID(Rp, Ri, Rd);
                            yaw_pid.setPID(Yp, Yi, Yd);
                            roll_bias = Pp;
                            pitch_bias = Pi;
                            yaw_bias = Pd;
                            printf("Core0: PID values updated:\n");
                            printf("  RP: %.4f, RI: %.4f, RD: %.4f\n", Rp, Ri, Rd);
                            printf("  PP: %.4f, PI: %.4f, PD: %.4f\n", Pp, Pi, Pd);
                            printf("  YP: %.4f, YI: %.4f, YD: %.4f\n", Yp, Yi, Yd);
                            pid_changed = false;
                        }
                        last_print_us = current_time_us;
                    }
                }
            } else {
                q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
                first_throttle_on = true;
                roll_pid.reset();
                pitch_pid.reset();
                yaw_pid.reset();
                set_esc_pulse_us(MOTOR1_PWM_PIN, ESC_MIN_PULSE_US);
                set_esc_pulse_us(MOTOR2_PWM_PIN, ESC_MIN_PULSE_US);
                set_esc_pulse_us(MOTOR3_PWM_PIN, ESC_MIN_PULSE_US);
                set_esc_pulse_us(MOTOR4_PWM_PIN, ESC_MIN_PULSE_US);
            }

        } else {
            // If no new data, give up CPU for a short while to avoid busy-waiting
            sleep_us(1); // Sleep even less to poll the flag more aggressively
        }
    }

    return 0; // Not reached in typical embedded applications
}
