#include <Arduino.h>
#include <MotorClass.h>

// DEFINE ROBOT PARAMETERS
#define WHEEL_DIAMETER_MM 40
#define WHEEL_DISTANCE_MM 110

#define STOP 0
#define STRAIGHT 1
#define TURN 2

struct PID_DATA
{
    float kp;
    float ki;
    float kd;
    float error_sum;
    float last_error;
};

class ROBOT {
    public:
        MOTOR* motorL = new MOTOR();
        MOTOR* motorR = new MOTOR();

    private:
        elapsedMillis clock;

        u_int64_t prev_time = 0;

        float wheel_diameter_mm = WHEEL_DIAMETER_MM;
        float wheel_circumference_mm = (WHEEL_DIAMETER_MM * 3.14159);

        float max_accel_rpss = 2;
        float prev_rps;

        float heading_angle = 0;
        float target_heading_angle = 0;

        float target_distance_mm = 0;
        float move_start_distanceL_mm = 0;
        float move_start_distanceR_mm = 0;
        elapsedMillis move_time = 0;

        int possible_done_time = 0;

        float max_velocity_rps = 0;

        float target_robot_speed_rps = 0;
        float motorR_target_rps = 0;
        float motorL_target_rps = 0;

        u_int8_t mode = STOP;

        PID_DATA linear_pid = PID_DATA{0.009, 0.0012, 0.085};

    // Methods
    public:
        ROBOT();

        void setup_robot_pinModes();
        void set_stop();
        void set_linear_drive(float distance, float max_velocity);
        void set_rotational_drive(float heading);

        void update_robot();

        float get_distanceL_mm();
        float get_distanceR_mm();
        int get_state();
        float get_target_speed_rps();
        float get_target_speed_ms();

    private:
        float get_total_distanceL_mm();
        float get_total_distanceR_mm();

        void update_motors(float dt);

        void update_rotational_drive(float dt);
        void update_linear_drive(float dt);
        void update_stop(float dt);

        float linear_velocity_pid(float setpoint, float current, float dt);
};