#include <Arduino.h>
#include <Dispenser.h>

Dispenser::Dispenser(int pin) {
    this->dispenser_servo = new PWMServo();
    setup_servo(pin);
    this->clock = 0;
}

Dispenser::Dispenser() {
    this->dispenser_servo = new PWMServo();
    setup_servo(0);
    this->clock = 0;
}

void Dispenser::pulse() {
    this->start_time = clock;
    this->state = OPENING;
}

void Dispenser::setup_servo(int pin) {
    this->dispenser_servo->attach(pin, servoMin, servoMax);
}

void Dispenser::update() {
    if (this->clock - this->start_time > 750) {
        this->state = CLOSING;
    }
    this->update_servo();
}

void Dispenser::update_servo() {
    if (this->state == OPENING) {
        this->dispenser_servo->write(100);
    } else if (this->state == CLOSING) {
        this->dispenser_servo->write(60);
    }
}

float Dispenser::get_pos() {
    return this->dispenser_servo->read();
}