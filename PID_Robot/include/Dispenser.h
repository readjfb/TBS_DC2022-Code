#include <Arduino.h>
#include <PWMServo.h>

#define servoMin 500
#define servoMax 3000

#define OPENING 1
#define CLOSING 0

class Dispenser {
    public:
        PWMServo *dispenser_servo = new PWMServo();
        void pulse();
        void setup_servo(int pin);
        void update();

        Dispenser();
        Dispenser(int pin);

        float get_pos();

    private:
        int servo_pos = 0;
        elapsedMillis clock;
        int start_time = 0;
        int state = CLOSING;

        void update_servo();
};