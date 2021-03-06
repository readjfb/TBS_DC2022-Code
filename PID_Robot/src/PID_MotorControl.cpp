#include <Arduino.h>
#include <PIN_MAP.cpp>
#include <PID_MotorControl.h>
#include <Robot.h>
#include <Bounce.h>

#define FT_TO_CM 30.48

elapsedMillis clock;

long motor1_velocity, motor1_prevPos;

long prev_time = 0;

float SETPOINT = 1.7;

bool ERROR = false;

ROBOT robot = ROBOT();

Bounce button = Bounce(buttonPin, 10);

int dir = 1;

// Converts from encoder steps to revolutions per second
float rps_conversion(long encoder_steps, long steps_per_rev, long dt) {
  float m = (static_cast<float>(encoder_steps) / static_cast<float>(dt)) * 1000;
  m /= static_cast<float>(steps_per_rev);

  return m;
}

long rps_to_period_conversion(float rps, long steps_per_rev, long dt) {
  float m = rps * steps_per_rev;
  m /= 1000;
  m *= dt;

  return static_cast<long>(m);
}

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);

  robot.setup_robot_pinModes();
  pinMode(LED_PIN, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  robot.set_stop();

  // robot.set_linear_drive(45.72, 0.5);
}

void loop() {
  // put your main code here, to run repeatedly:
  robot.update_robot();

  if ((clock - prev_time) >= SAMPLE_RATE_MS) {
    digitalWrite(LED_PIN, HIGH);

    prev_time = clock;

    robot.update_sensors();

    Serial.print(robot.get_distanceL_mm() / 10);
    Serial.print(",\t");
    Serial.print(robot.get_distanceR_mm() / 10);
    Serial.print(",\t");
    Serial.print((robot.get_distanceR_mm() - robot.get_distanceL_mm()));
    Serial.print(",\t");
    Serial.print(robot.get_state());
    Serial.print(",\t");
    Serial.print(robot.get_target_speed_ms());
    Serial.print(",\t");
    Serial.print(robot.get_target_speed_rps());
    Serial.print(",\t");
    Serial.print(robot.is_moving());
    Serial.println();

    button.update();

    if (button.risingEdge()) {
      robot.go();
    }
  }

  if (Serial.available()) {
    float new_val = Serial.parseFloat();
    Serial.read();

    if (abs(new_val) <= MAX_MOTOR_RPS) {
      SETPOINT = new_val;
    }
  }
}
