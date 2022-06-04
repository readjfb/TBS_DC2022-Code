#include <Arduino.h>
#include <MotorClass.h>
#include <HeadingClass.h>
#include <Dispenser.h>

// DEFINE ROBOT PARAMETERS
#define WHEEL_DIAMETER_MM 78
#define WHEEL_DISTANCE_MM 176.5

#define STOP 0
#define STRAIGHT 1
#define TURN 2
#define WAIT 3

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

        Dispenser* dispenser = new Dispenser();

        HEADING_6050* heading_sensor = new HEADING_6050();

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

        u_int32_t milis_wait = 0;

        u_int8_t mode = STOP;

        // PID_DATA linear_pid = PID_DATA{0.009, 0,0};
        PID_DATA linear_pid = PID_DATA{0.01, 0.00092, 0.0};

        int last_token_time = clock;

        int step = -1;
        bool moving = false;

    // Methods
    public:
        ROBOT();

        void setup_robot_pinModes();
        void set_stop();
        void set_linear_drive(float distance, float max_velocity);
        void set_rotational_drive(float heading);
        void set_wait(int milis);

        void update_robot();
        void update_sensors();

        float get_distanceL_mm();
        float get_distanceR_mm();
        int get_state();
        float get_target_speed_rps();
        float get_target_speed_ms();

        float get_heading_angle();

        bool is_moving();

        void go();

    private:
        float get_total_distanceL_mm();
        float get_total_distanceR_mm();

        void update_motors(float dt);

        void update_rotational_drive(float dt);
        void update_linear_drive(float dt);
        void update_stop(float dt);
        void update_wait(float dt);

        float linear_velocity_pid(float setpoint, float current, float dt);

        // TODO: Make a dispenser class
};