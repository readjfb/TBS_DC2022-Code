#include <Arduino.h>
#include <Robot.h>
#include <PID_MotorControl.h>
#include <PIN_MAP.cpp>

ROBOT::ROBOT()
{
    this->motorL = new MOTOR(motorL_PWM, motorL_dir, motorL_encoderA, motorL_encoderB);
    this->motorR = new MOTOR(motorR_PWM, motorR_dir, motorR_encoder_A, motorR_encoder_B);

    this->motorL->set_invert(false);
    this->motorR->set_invert(true);

    this->motorL->set_pid_parameters(15, 0.15, 0.0);
    this->motorR->set_pid_parameters(15, 0.15, 0.0);

    this->motorL->set_velocity(0, 1);
    this->motorR->set_velocity(0, 1);

    this->clock = 0;
}

void ROBOT::update_motors(float dt)
{
    this->motorL->set_velocity(this->motorL_target_rps, dt);
    this->motorR->set_velocity(this->motorR_target_rps, dt);
}

void ROBOT::setup_robot_pinModes()
{
    this->motorL->setup_motor_pinModes();
    this->motorR->setup_motor_pinModes();
}

void ROBOT::set_stop()
{
    this->mode = STOP;
}

// Sets the robot to drive in a line for distance distance (in cm)
// Max speed is max_velocity, given in meters / second
void ROBOT::set_linear_drive(float distance, float max_velocity)
{
    this->mode = STRAIGHT;
    this->target_distance_mm = distance * 10;

    this->max_velocity_rps = (max_velocity * 1000) / this->wheel_circumference_mm;

    this->move_time = 0;
    this->possible_done_time = 0;

    this->move_start_distanceL_mm = this->get_total_distanceL_mm();
    this->move_start_distanceR_mm = this->get_total_distanceR_mm();
}

void ROBOT::set_rotational_drive(float heading)
{
    this->mode = TURN;
    this->target_heading_angle = heading;
}

void ROBOT::update_robot()
{
    u_int64_t time = clock;
    float dt = (time - prev_time);
    prev_time = time;

    this->update_motors(dt);

    switch (this->mode)
    {
        case STOP:
            this->update_stop(dt);
            break;
        case STRAIGHT:
            this->update_linear_drive(dt);
            break;
        case TURN:
            this->update_rotational_drive(dt);
            break;
    }
}

void ROBOT::update_stop(float dt)
{
    this->motorL_target_rps = 0;
    this->motorR_target_rps = 0;
}

void ROBOT::update_linear_drive(float dt)
{
    float adjusted_distance = this->get_total_distanceL_mm() - this->move_start_distanceL_mm;

    float target_speed = this->linear_velocity_pid(this->target_distance_mm, adjusted_distance, dt);
    target_speed = constrain(target_speed, -this->max_velocity_rps, this->max_velocity_rps);
    target_speed = constrain(target_speed, -MAX_MOTOR_RPS, MAX_MOTOR_RPS);

    Serial.println(target_speed);

    // Constrain the target_speed within the acceleration
    if (abs(target_speed) > abs(this->prev_rps) + (this->max_accel_rpss * (dt / 1000))) {

        if (target_speed > 0) {
            target_speed = this->prev_rps + (this->max_accel_rpss * (dt / 1000));
        } else {
            target_speed = this->prev_rps - (this->max_accel_rpss * (dt / 1000));
        }
    }

    this->prev_rps = target_speed;

    this->target_robot_speed_rps = target_speed;
    this->motorL_target_rps = this->target_robot_speed_rps;
    this->motorR_target_rps = this->target_robot_speed_rps;

    // STILL VERY MUCH NEED TO DO HEADING ADJUSTMENT


    // Logic to detect if move is done
    if ((abs(adjusted_distance - this->target_distance_mm) < 1) && (abs(this->target_robot_speed_rps) < 0.15))
    {
        if (this->possible_done_time < 0)
            this->possible_done_time = this->clock;
    } else {
        this->possible_done_time = -1;
    }
    if ((this->possible_done_time > 0) && (this->clock - this->possible_done_time > 1000))
    {
       this->set_stop();
    }

}

float ROBOT::linear_velocity_pid(float setpoint, float current, float dt)
{
    if (dt == 0)
        return 0;

    float error = setpoint - current;

    this->linear_pid.error_sum += error * dt;
    this->linear_pid.error_sum = constrain(this->linear_pid.error_sum, -100, 100);

    float P = this->linear_pid.kp * error;
    float I = this->linear_pid.ki * this->linear_pid.error_sum;
    float D = this->linear_pid.kd * (error - this->linear_pid.last_error) / dt;

    this->linear_pid.last_error = error;

    return P + I + D;
}

void ROBOT::update_rotational_drive(float dt)
{
    // TBD
    return;
}


float ROBOT::get_total_distanceL_mm()
{
    float abs_encoder_position = this->motorL->get_abs_encoder_pos();

    float abs_encoder_revs = abs_encoder_position / ENCODER_TICKS_P_REV;
    float abs_pos_mm = abs_encoder_revs * this->wheel_circumference_mm;

    return abs_pos_mm;
}

float ROBOT::get_total_distanceR_mm()
{
    float abs_encoder_position = this->motorR->get_abs_encoder_pos();

    float abs_encoder_revs = abs_encoder_position / ENCODER_TICKS_P_REV;
    float abs_pos_mm = abs_encoder_revs * this->wheel_circumference_mm;

    return abs_pos_mm;
}

float ROBOT::get_distanceL_mm()
{
    return this->get_total_distanceL_mm() - this->move_start_distanceL_mm;
}

float ROBOT::get_distanceR_mm()
{
    return this->get_total_distanceR_mm() - this->move_start_distanceR_mm;
}

int ROBOT::get_state()
{
    return this->mode;
}

float ROBOT::get_target_speed_rps()
{
    return this->target_robot_speed_rps;
}

float ROBOT::get_target_speed_ms()
{
    return (this->target_robot_speed_rps * this->wheel_circumference_mm) / 1000;
}