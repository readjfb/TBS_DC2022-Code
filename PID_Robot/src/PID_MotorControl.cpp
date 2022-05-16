#include <Arduino.h>
#include <Encoder.h>
#include <PIN_MAP.cpp>
#include <PID_MotorControl.h>
#include <MotorClass.h>

elapsedMillis clock;

long motor1_velocity, motor1_prevPos;

long prev_time = 0;

float SETPOINT = 1.7;

bool ERROR = false;

MOTOR M1 = MOTOR(motor1_PWM, motor1_dir, motor1_encoderA, motor1_encoderB);
MOTOR M2 = MOTOR(motor2_PWM, motor2_dir, motor2_encoder_A, motor2_encoder_B);

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

  M1.set_pid_parameters(15, 0.15, 0.0);

  M2.setup_motor_pinModes();
  M2.set_pid_parameters(15, 0.15, 0.0);
  M2.set_invert(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((clock - prev_time) >= SAMPLE_RATE_MS) {
    if (M1.ERROR && (clock % 1000 < 500)) {
      digitalWrite(LED_PIN, LOW);
    }
    else {
      digitalWrite(LED_PIN, HIGH);
    }
    M1.ERROR = false;

    long dt = (clock - prev_time);

    prev_time = clock;

    M1.set_velocity(SETPOINT, dt);
    M2.set_velocity(SETPOINT, dt);

    Serial.print(SETPOINT);
    Serial.print(",");
    Serial.print(M1.rps);
    Serial.print(",");
    Serial.print(M2.rps);
    Serial.print(",");
    Serial.print(abs(M1.get_abs_encoder_pos()) - abs(M2.get_abs_encoder_pos()));
    Serial.print(",");
    Serial.println();
  }

  if (Serial.available()) {
    float new_val = Serial.parseFloat();
    Serial.read();

    if (abs(new_val) <= MAX_MOTOR_RPS) {
      SETPOINT = new_val;
    }
  }
}
