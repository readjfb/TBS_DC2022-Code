# include <Arduino.h>
# include <HeadingClass.h>

HEADING_6050::HEADING_6050()
{
    this->mpu = Adafruit_MPU6050();
    this->total_rotation = 0;
}

bool HEADING_6050::init()
{

    if (!this->mpu.begin()){
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        return false;
    }

    this->mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    this->mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    this->mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    // this->mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    this->clock = 0;
    this->prev_time = 0;
    return true;
}

void HEADING_6050::calibrateZ()
{
    this->clock = 0;
    this->prev_time = 0;
    this->num_calibration_points = 0;
    this->calibration_Z = 0;
    this->total_rotation = 0;

    while (this->clock < calibration_time_ms)
    {
        int t = this->clock;
        float dt = t - this->prev_time;
        this->prev_time = t;

        if (dt >= 1){
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            this->calibration_Z += g.gyro.z;
            this->num_calibration_points++;
        }
    }

    this->calibration_Z /= this->num_calibration_points;

}

void HEADING_6050::update()
{
    int t = this->clock;
    float dt = t - this->prev_time;

    if (dt >= 2){
        sensors_event_t a, g, temp;
        this->mpu.getEvent(&a, &g, &temp);

        this->total_rotation += (g.gyro.z - this->calibration_Z) * (dt / 1000);
        this->prev_time = t;
    }
}

float HEADING_6050::getHeadingZ_rad()
{
    return this->total_rotation;
}

float HEADING_6050::getHeadingZ_deg()
{
    return this->total_rotation * 180 / 3.14159;
}

void HEADING_6050::reset_heading()
{
    this->total_rotation = 0;
}