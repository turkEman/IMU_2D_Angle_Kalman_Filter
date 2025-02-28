#include <Arduino.h>
#include <Wire.h>

#define MPU_addr 0x68 // MPU9250address

#define loop_time 4000
#define loop_rate 250
#define inverse_loop_rate 0.004

int16_t raw_x = 0, raw_y = 0, raw_z = 0; // for reading raw data
float angle_gx = 0, angle_gy = 0;        // extracted angle from gyro data by actan function
float angle_ax = 0, angle_ay = 0;        // extracted angle from accl data by integration

// variables for gyro data
float offset_gx = 0, offset_gy = 0, offset_gz = 0; // for calibration
#define cali_times 3000
int cali_count = 0;           // for loop counter
float gx = 0, gy = 0, gz = 0; // for final data

// variables for accl data
// no need to calibrate
float ax = 0, ay = 0, az = 0; // for final data

// variables for kalman
float angle_kx = 0, angle_ky = 0, angle_kz = 0;                // kalman angles
float uncert_kx = 2 * 2, uncert_ky = 2 * 2, uncert_kz = 2 * 2; // kalman uncertainties

void mpu_connect(void);
bool mpu_connected(void);
void mpu_init(void);
void mpu_cali(void);
void mpu_readData(void);
void kalman_1D(float &state, float &uncertainty, float &input, float &measurement, float &angle_out, float &uncert_out);

void setup()
{
    Serial.begin(115200);
    Serial.println("Connecting.............");
    do
    {
        mpu_connect();
        delay(250);
    } while (!mpu_connected());

    Serial.println("Connected.............");

    Serial.println("MPU initalizing");
    mpu_init();
    Serial.println("MPU initalized");

    Serial.println("Calibrating.............");
    mpu_cali();
    Serial.println("Calibrated.............");
}
uint32_t a, b, c, d;
uint32_t time_data, time_filter, time_end;

void loop()
{
    a = micros();
    mpu_readData();
    b = micros();
    time_data = b - a;

    c = micros();
    kalman_1D(angle_kx, uncert_kx, angle_ax, gx, angle_kx, uncert_kx);
    kalman_1D(angle_ky, uncert_ky, angle_ay, gy, angle_ky, uncert_ky);
    d = micros();
    time_filter = d - c;

    while (micros() - a < 4000)
        ;

    time_end = micros() - a;

    Serial.print("kal x:" + String(angle_kx) + " kal y:" + String(angle_ky) + " ");
    Serial.println("data:" + String(time_data) + " filtr:" + String(time_filter) + " end:" + String(time_end) + " ");
}

void mpu_connect(void)
{
    // Start communication with MPU9250
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock to 400 kHz
    Wire.beginTransmission(MPU_addr);

    // Power management register
    Wire.write(0x6B);

    // Set to 0 to wake up the MPU9250
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void mpu_init(void)
{
    // Turn on low-pass filter (works for both gyro and accel)
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1A); // Access the low-pass filter config register
    Wire.write(0x05); // Set low-pass filter to 10Hz (DLPF value 5)
    Wire.endTransmission();

    // Configure accelerometer output
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1C); // Accelerometer configuration register
    Wire.write(0x10); // Select full-scale range of +/- 8g
    Wire.endTransmission();

    // Configure gyroscope output
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1B); // Gyroscope configuration register
    Wire.write(0x08); // Set sensitivity scale factor to 65.5 LSB/deg/s
    Wire.endTransmission();
}

// bool mpu_connected(void)
// {
//     // Start communication with MPU9250
//     Wire.beginTransmission(MPU_addr);

//     // WHO_AM_I register
//     Wire.write(0x75);
//     Wire.endTransmission();

//     // Request 1 byte of data
//     Wire.requestFrom(MPU_addr, 1);

//     int who_am_i = Wire.read();
//     Serial.print("WHO_AM_I: 0x");
//     Serial.println(who_am_i, HEX);
//     // return (who_am_i == 0x68);
//     if (who_am_i == 0x70)
//         return true;
// }
bool mpu_connected(void)
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x75); // WHO_AM_I register
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_addr, 1);

    if (Wire.available())
    {
        int who_am_i = Wire.read();
        Serial.print("WHO_AM_I: 0x");
        Serial.println(who_am_i, HEX);
        return (who_am_i == 0x68 || who_am_i == 0x70 || who_am_i == 0x71 || who_am_i == 0x73 || who_am_i == 0x75);
    }
    return false;
}

void mpu_cali(void)
{
    // calibrating
    for (cali_count = 0; cali_count < cali_times; cali_count++)
    {

        // finally reading gyro data
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43); // start reading from register 0x43
        Wire.endTransmission();

        Wire.requestFrom(MPU_addr, 6); // requesting 6 bytes of data

        raw_x = Wire.read() << 8 | Wire.read();
        raw_y = Wire.read() << 8 | Wire.read();
        raw_z = Wire.read() << 8 | Wire.read();

        // summing up data
        offset_gx += raw_x;
        offset_gy += raw_y;
        offset_gz += raw_z;

        delay(3);
    }

    offset_gx /= cali_times;
    offset_gy /= cali_times;
    offset_gz /= cali_times;

    Serial.print("Offset x: ");
    Serial.print(offset_gx);
    Serial.print(" Offset y: ");
    Serial.print(offset_gy);
    Serial.print(" Offset z: ");
    Serial.println(offset_gz);
}

void mpu_readData(void)
{
    // finally reading gyro data
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43); // start reading from register 0x43
    Wire.endTransmission();

    Wire.requestFrom(MPU_addr, 6); // requesting 6 bytes of data

    raw_x = Wire.read() << 8 | Wire.read();
    raw_y = Wire.read() << 8 | Wire.read();
    raw_z = Wire.read() << 8 | Wire.read();

    // compensating for offsets (in raw values)
    raw_x -= offset_gx;
    raw_y -= offset_gy;
    raw_z -= offset_gz;

    // converting raw gyro data to deg/s
    gx = (float)raw_x * 0.0152671;
    gy = (float)raw_y * 0.0152671;
    gz = (float)raw_z * 0.0152671; // 0.0152671 = 1/65.5

    // integrating gyro rate to get angle
    angle_gx += raw_x * 0.00006107;
    angle_gy += raw_y * 0.00006107;

// Apply yaw correction based on the current pitch and roll angles
#define scaling_factor 0.000001066
    float temp_gx = angle_gx - angle_gy * sin(raw_z * scaling_factor);
    float temp_gy = angle_gy + angle_gx * sin(raw_z * scaling_factor);

    angle_gx = temp_gx;
    angle_gy = temp_gy;

    raw_x = 0;
    raw_y = 0;
    raw_z = 0;

    // finally reading accl data
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // start reading from register 0x3B
    Wire.endTransmission();

    Wire.requestFrom(MPU_addr, 6); // requesting 6 bytes of data

    raw_x = Wire.read() << 8 | Wire.read();
    raw_y = Wire.read() << 8 | Wire.read();
    raw_z = Wire.read() << 8 | Wire.read();

    // converting raw accl data to g
    ax = (float)raw_x / 4096 - 0.03;
    ay = (float)raw_y / 4096 - 0.025;
    az = (float)raw_z / 4096 + 0.09;

    // extracting angle from accl data
    angle_ax = atan(ay / sqrt(ax * ax + az * az)) * 57.29577951;
    angle_ay = -atan(ax / sqrt(ay * ay + az * az)) * 57.29577951;
}

void kalman_1D(float &state, float &uncertainty, float &input, float &measurement, float &angle_out, float &uncert_out)
{
    // state is k angle, input is accel angle
    state = state + 0.004 * input;
    // calculate uncertainty
    uncertainty = uncertainty + 0.004 * 0.004 * 4 * 4;
    // gain is the weight towards either the state or the measurement
    float gain = uncertainty * 1 / (1 * uncertainty + 3 * 3);
    // agate again?
    state = state + gain * (measurement - state);
    // update uncertainty
    uncertainty = (1 - gain) * uncertainty;
    // put values in variables
    angle_out = state;
    uncert_out = uncertainty;
}