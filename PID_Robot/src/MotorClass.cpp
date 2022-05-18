#include <Arduino.h>
#include <Encoder.h>
#include <MotorClass.h>
#include <PID_MotorControl.h>

// Default Constructor
MOTOR::MOTOR()
{
    this->pwm_pin = 0;
    this->dir_pin = 0;
    this->encoderA_pin = 0;
    this->encoderB_pin = 0;
}

MOTOR::MOTOR(int pwm_pin, int dir_pin, int encoderA_pin, int encoderB_pin)
{
    this->pwm_pin = pwm_pin;
    this->dir_pin = dir_pin;
    this->encoderA_pin = encoderA_pin;
    this->encoderB_pin = encoderB_pin;
    this->encoder = new Encoder(encoderA_pin, encoderB_pin);
}

void MOTOR::setup_motor_pinModes()
{
    pinMode(this->pwm_pin, OUTPUT);
    pinMode(this->dir_pin, OUTPUT);
    pinMode(this->encoderA_pin, INPUT);
    pinMode(this->encoderB_pin, INPUT);
}

void MOTOR::set_pid_parameters(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

// Accepts values from -255 to +255, caps input out of range
void MOTOR::send_pwm() {
    bool dir_ = (this->pwm_speed > 0);
    int speed_ = min(MAX_PWM, abs(this->pwm_speed));

    // if (this->invert) {
    //     dir_ = !dir_;
    // }

    if (dir_)
    {
        digitalWrite(this->dir_pin, HIGH);
    }
    else
    {
        digitalWrite(this->dir_pin, LOW);
    }

    analogWrite(this->pwm_pin, speed_);

    this->active = pwm_speed != 0;
}

int MOTOR::pid_control(float dt, float setpoint, float pos) {
    if (dt == 0)
        return 0;

    float error = setpoint - pos;

    this->error_sum += error * dt;
    this->error_sum = min(this->error_sum, 10);
    this->error_sum = max(this->error_sum, -10);

    float P = (this->kp * error);
    float I = (this->ki * this->error_sum);
    float D = (this->kd * (error - this->last_error) / dt);
    this->last_error = error;

    int out = static_cast<int>(P + I + D);

    return out;
}

void MOTOR::set_velocity(float setpoint_rps, float dt) {
    if (this->active & (this->rps == 0)) {
        this->ERROR = true;
    }

    long motor_pos = this->encoder->read();

    if (this->invert) {
        setpoint_rps = -setpoint_rps;
    }

    long setpoint_intermediate = rps_to_period_conversion(setpoint_rps, ENCODER_TICKS_P_REV, dt);
    float setpoint_inter = rps_conversion(setpoint_intermediate, ENCODER_TICKS_P_REV, dt);

    this->rps = rps_conversion(motor_pos - this->prevPos, ENCODER_TICKS_P_REV, dt);

    // if (this->invert) {
    //     this->rps = -this->rps;
    // }

    this->prevPos = motor_pos;

    if (setpoint_rps == 0)
    {
        this->pwm_speed = 0;
    }
    else
    {
        this->pwm_speed += pid_control(dt, setpoint_inter, this->rps);
        this->pwm_speed = min(this->pwm_speed, MAX_PWM);
        this->pwm_speed = max(this->pwm_speed, -MAX_PWM);
    }

    this->send_pwm();
}

void MOTOR::set_invert(bool invert) {
    this->invert = invert;
}

float MOTOR::get_abs_encoder_pos() {
    if (this->invert) {
        return -this->prevPos;
    }

    return this->prevPos;
}

bool MOTOR::get_invert() {
    return this->invert;
}