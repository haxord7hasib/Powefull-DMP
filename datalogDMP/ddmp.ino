#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>

int led = 11;
MPU6050 accelgyro; 

unsigned long now, lastTime = 0;
float dt; // Time variable

int16_t ax, ay, az, gx, gy, gz; // Raw data from accelerometer and gyroscope
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; // Angle variables
long axo = 0, ayo = 0, azo = 0; // Accelerometer offsets
long gxo = 0, gyo = 0, gzo = 0; // Gyroscope offsets

float pi = 3.1415926;
float AcceRatio = 16384.0; // Accelerometer scaling factor
float GyroRatio = 131.0; // Gyroscope scaling factor

uint8_t n_sample = 8; // Number of samples for accelerometer filter algorithm
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0}; // Arrays for storing x, y, z-axis samples
long aax_sum, aay_sum, aaz_sum; // Summation of x, y, z-axis samples

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0}, g_x[10] = {0}, g_y[10] = {0}, g_z[10] = {0}; // Arrays for calculating accelerometer covariance
float Px = 1, Rx, Kx, Sx, Vx, Qx; // Kalman filter variables for x-axis
float Py = 1, Ry, Ky, Sy, Vy, Qy; // Kalman filter variables for y-axis
float Pz = 1, Rz, Kz, Sz, Vz, Qz; // Kalman filter variables for z-axis

File dataFile; // File object for SD card

void setup() {
    Wire.begin();
    Serial.begin(115200);
    pinMode(led, OUTPUT);
    accelgyro.initialize(); // Initialize MPU6050

    // Initialize SD card
    Serial.print("Initializing SD card...");
    if (!SD.begin(10)) { // Assuming chip select pin for SD module is 10
        Serial.println("initialization failed!");
        while (1);
    }
    Serial.println("initialization done.");

    // Open or create a CSV file
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
        // Write the CSV header
        dataFile.println("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
        dataFile.close(); 
    } else {
        Serial.println("Error opening data.csv");
    }

    // Calibrating MPU6050
    unsigned short times = 200; // Number of samples for calibration
    for(int i = 0; i < times; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw six-axis values
        axo += ax; ayo += ay; azo += az; // Accumulate for offset calculation
        gxo += gx; gyo += gy; gzo += gz;
    }
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offsets
    gxo /= times; gyo /= times; gzo /= times; // Calculate gyroscope offsets
}

void loop() {
    now = millis(); // Current time in milliseconds
    dt = (now - lastTime) / 1000.0; // Time difference in seconds
    lastTime = now; // Update last sampling time

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read raw six-axis values again

    // Convert raw data to angles
    float accx = ax / AcceRatio; 
    float accy = ay / AcceRatio; 
    float accz = az / AcceRatio; 

    aax = atan2(accy, accz) * 180 / pi;
    aay = atan2(accx, accz) * 180 / pi;
    aaz = atan2(accz, sqrt(accx * accx + accy * accy)) * 180 / pi;

    // Apply sliding weighted filtering algorithm for accelerometer data
    for (int i = n_sample - 1; i > 0; i--) {
        aaxs[i] = aaxs[i - 1];
        aays[i] = aays[i - 1];
        aazs[i] = aazs[i - 1];
    }
    aaxs[0] = aax;
    aays[0] = aay;
    aazs[0] = aaz;

    aax_sum = 0;
    aay_sum = 0;
    aaz_sum = 0;
    for (int i = 0; i < n_sample; i++) {
        aax_sum += aaxs[i] * (i + 1);
        aay_sum += aays[i] * (i + 1);
        aaz_sum += aazs[i] * (i + 1);
    }
    aax = aax_sum / (n_sample * (n_sample + 1) / 2);
    aay = aay_sum / (n_sample * (n_sample + 1) / 2);
    aaz = aaz_sum / (n_sample * (n_sample + 1) / 2);

    // Gyroscope data processing
    float gyrox = - (gx - gxo) / GyroRatio;
    float gyroy = - (gy - gyo) / GyroRatio;
    float gyroz = - (gz - gzo) / GyroRatio;

    // Update Kalman filter variables and compute angles
    // Kalman filter for x-axis
    Px = Px + Qx;
    Kx = Px / (Px + Rx);
    agx = agx + Kx * (aax - agx);
    Px = (1 - Kx) * Px;

    // Kalman filter for y-axis
    Py = Py + Qy;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy);
    Py = (1 - Ky) * Py;

    // Kalman filter for z-axis
    Pz = Pz + Qz;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz);
    Pz = (1 - Kz) * Pz;

    // LED control based on angle
    if (agx > some_threshold) { // Replace 'some_threshold' with your desired value
        digitalWrite(led, HIGH);
    } else {
        digitalWrite(led, LOW);
    }

    // Open the file in write mode
    dataFile = SD.open("data.csv", FILE_WRITE);

    // If the file is available, write to it
    if (dataFile) {
        dataFile.print(now); dataFile.print(",");
        dataFile.print(ax); dataFile.print(",");
        dataFile.print(ay); dataFile.print(",");
        dataFile.print(az); dataFile.print(",");
        dataFile.print(gx); dataFile.print(",");
        dataFile.print(gy); dataFile.print(",");
        dataFile.println(gz);
        dataFile.close(); // Close the file
    } else {
        Serial.println("Error opening data.csv");
    }

    // Serial printing for debugging
    Serial.print("x axis angle: "); Serial.println(agx);
    Serial.print("y axis angle: "); Serial.println(agy);
    Serial.print("z axis angle: "); Serial.println(agz);
    Serial.print("x axis acceleration: "); Serial.println(accx);
    Serial.print("y axis acceleration: "); Serial.println(accy);
    Serial.print("z axis acceleration: "); Serial.println(accz);
}
