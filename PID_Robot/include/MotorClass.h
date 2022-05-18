#include <Arduino.h>
#include <Encoder.h>

class MOTOR {
    // Define the motor's variables
    public:
        float pwm_speed = 0;
        float rps = 0;
        bool active = false;
        bool ERROR = false;
    private:
        int pwm_pin;
        int dir_pin;
        int encoderA_pin;
        int encoderB_pin;
        Encoder* encoder = new Encoder(0, 1);
        long prevPos = 0;
        bool invert = false;
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float error_sum = 0;
        float last_error = 0;

    // Define the motor functions
    public:
        MOTOR();
        MOTOR(int pwm_pin, int dir_pin, int encoderA_pin, int encoderB_pin);

        void setup_motor_pinModes();
        void set_velocity(float setpoint_rps, float dt);

        void set_pid_parameters(float kp, float ki, float kd);

        void set_invert(bool invert);

        float get_abs_encoder_pos();

        bool get_invert();

    private:
        void send_pwm();
        int pid_control(float dt, float setpoint, float pos);
};