#include <Arduino.h>

// DEFINE MODEL CONSTANTS
#define ENCODER_TICKS_P_REV (408*4)
#define SAMPLE_RATE_MS 5
#define MAX_PWM 255
#define MAX_MOTOR_RPS 2.5

float rps_conversion(long encoder_steps, long steps_per_rev, long dt);
long rps_to_period_conversion(float rps, long steps_per_rev, long dt);