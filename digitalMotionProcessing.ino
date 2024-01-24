#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
int led = 11;

MPU6050 accelgyro; 

unsigned long now, lastTime = 0;
float dt; // Time variable

int16_t ax, ay, az, gx, gy, gz; // Raw data from accelerometer and gyroscope
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0; // Angle variables
long axo = 0, ayo = 0, azo = 0; // Accelerometer offsets
long gxo = 0, gyo = 0, gzo = 0; // Gyroscope offsets

float pi = 3.1415926;
float AcceRatio = 16384.0; // Accelerometer scaling factor
float GyroRatio = 131.0; // Gyroscope scaling factor

uint8_t n_sample = 8; // Number of samples for accelerometer filter algorithm
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0}; // Arrays for storing x, y, z-axis samples
long aax_sum, aay_sum,aaz_sum; // Summation of x, y, z-axis samples

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; // Arrays for calculating accelerometer covariance
float Px=1, Rx, Kx, Sx, Vx, Qx; // Kalman filter variables for x-axis
float Py=1, Ry, Ky, Sy, Vy, Qy; // Kalman filter variables for y-axis
float Pz=1, Rz, Kz, Sz, Vz, Qz; // Kalman filter variables for z-axis

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    pinMode(led, OUTPUT);
    accelgyro.initialize(); // Initialize MPU6050

    unsigned short times = 200; // Number of samples for calibration
    for(int i=0;i<times;i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw six-axis values
        axo += ax; ayo += ay; azo += az; // Accumulate for offset calculation
        gxo += gx; gyo += gy; gzo += gz;
    }
    
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offsets
    gxo /= times; gyo /= times; gzo /= times; // Calculate gyroscope offsets
}

void loop()
{
    unsigned long now = millis(); // Current time in milliseconds
    dt = (now - lastTime) / 1000.0; // Time difference in seconds
    lastTime = now; // Update last sampling time

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw six-axis values again

    float accx = ax / AcceRatio; // Calculate x-axis acceleration
    float accy = ay / AcceRatio; // Calculate y-axis acceleration
    float accz = az / AcceRatio; // Calculate z-axis acceleration

    aax = atan(accy / accz) * (-180) / pi; // Calculate angle of y-axis relative to z-axis
    aay = atan(accx / accz) * 180 / pi; // Calculate angle of x-axis relative to z-axis
    aaz = atan(accz / accy) * 180 / pi; // Calculate angle of z-axis relative to y-axis

    aax_sum = 0; aay_sum = 0; aaz_sum = 0; // Initialize sums for sliding weighted filtering
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i]; // Shift array values
        aax_sum += aaxs[i] * i; // Accumulate weighted sum
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    }
    
    aaxs[n_sample-1] = aax; // Store new value
    aax_sum += aax * n_sample; // Update sum with new value
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; // Calculate angle amplitude adjustment to 0-90°
    aays[n_sample-1] = aay;
    aay_sum += aay * n_sample;
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz;
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; // Calculate x-axis angular velocity
    float gyroy = - (gy-gyo) / GyroRatio * dt; // Calculate y-axis angular velocity
    float gyroz = - (gz-gzo) / GyroRatio * dt; // Calculate z-axis angular velocity
    agx += gyrox; // Integrate to get angular displacement
    agy += gyroy;
    agz += gyroz;

    // Start of Kalman filter calculations
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {
        a_x[i-1] = a_x[i]; // Shift array values
        Sx += a_x[i]; // Calculate average of measurements (acceleration)
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10; // Calculate average x-axis acceleration
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10; // Calculate average y-axis acceleration
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10; // Calculate average z-axis acceleration

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx); // Calculate variance
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    }
    
    Rx = Rx / 9; // Finalize variance calculation
    Ry = Ry / 9;
    Rz = Rz / 9;
  
    Px = Px + 0.0025; // Process noise for x-axis
    Kx = Px / (Px + Rx); // Calculate Kalman gain for x-axis
    agx = agx + Kx * (aax - agx); // Update gyro angle with accelerometer reading
    Px = (1 - Kx) * Px; // Update Px

    Py = Py + 0.0025; // Process noise for y-axis
    Ky = Py / (Py + Ry); // Calculate Kalman gain for y-axis
    agy = agy + Ky * (aay - agy);
    Py = (1 - Ky) * Py;

    Pz = Pz + 0.0025; // Process noise for z-axis
    Kz = Pz / (Pz + Rz); // Calculate Kalman gain for z-axis
    agz = agz + Kz * (aaz - agz);
    Pz = (1 - Kz) * Pz;

    // Level LED control based on x-axis angle
    if ((agx > 5.24) && (agx <5.50)) {
        digitalWrite(led, HIGH); // Turn on LED if angle is within range
    }
    else  {
        digitalWrite(led, LOW); // Turn off LED otherwise
    }

    // End of Kalman filter calculations

    // Serial print section for debugging and monitoring
    Serial.print("x axis angle: "); Serial.println(agx);
    Serial.print("y axis angle: "); Serial.println(agy);
    Serial.print("z axis angle: "); Serial.println(agz);
    Serial.print("x axis acceleration: "); Serial.println(accx);
    Serial.print("y axis acceleration: "); Serial.println(accy);
    Serial.print("z axis acceleration: "); Serial.println(accz);
}
