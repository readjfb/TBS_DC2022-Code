#include <Arduino.h>
#include <MotorClass.h>

class Robot {
    public:
        MOTOR* motorL = new MOTOR();
        MOTOR* motorR = new MOTOR();

    private:
        float wheel_diameter_mm = 65;
        float wheel_circumference_mm = (wheel_diameter_mm * 3.14159);

        float heading_angle = 0;

        float motorR_target_rps = 0;
        float motorL_target_rps = 0;

        float rotational_kp = 0;
        float rotational_ki = 0;
        float rotational_kd = 0;

        float linear_kp = 0;
        float linear_ki = 0;
        float linear_kd = 0;
        float linear_delta = 0;

    // Methods
    public:
        Robot();

        float get_heading_deg();
        void set_rotational_pid_parameters(float kp, float ki, float kd);


};