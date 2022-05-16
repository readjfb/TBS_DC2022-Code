#include <Arduino.h>
#include <Encoder.h>

// DEFINE MODEL CONSTANTS
#define ENCODER_TICKS_P_REV (408*4)
#define SAMPLE_RATE_MS 5
#define MAX_PWM 255
#define MAX_MOTOR_RPS 2.5

// DEFINE ROBOT PARAMETERS
# define wheel_diameter_mm 65
# define wheel_distance_mm 110

float rps_conversion(long encoder_steps, long steps_per_rev, long dt);
long rps_to_period_conversion(float rps, long steps_per_rev, long dt);