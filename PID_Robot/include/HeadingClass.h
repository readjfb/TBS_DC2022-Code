#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

# define calibration_time_ms 5000

class HEADING_6050 {
    public:
        float calibration_Z;

        // Attributes
    private:
        Adafruit_MPU6050 mpu;
        float total_rotation = 0;

        elapsedMillis clock;
        u_int64_t prev_time = 0;

        int num_calibration_points;

        sensors_event_t acc, gyro, temp;

        // Methods
    public:
        HEADING_6050();
        bool init();
        void calibrateZ();
        void update();
        float getHeadingZ_rad();
        float getHeadingZ_deg();
        void reset_heading();

    private:

};