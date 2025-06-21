#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define ALPHA 0.873
#define LOOP_PERIOD 20 // loop 주기 (ms)
#define TRIGGER_THRESHOLD 25.0
#define PI 3.14159265358979323846
#define ignitor_relay_pin 6

MPU6050 mpu;

// 사출
bool ejcet_option = true;
bool ejectionStarted = false;
unsigned long ejectionStartTime = 0;
bool isStabilizing = true;
unsigned long stabilizeStartTime = 0;

int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;

float Yaw_prev = 0.0f;

// 칼만 필터 변수
float KalAngleX = 0.0f, KalBiasX = 0.0f;
float KalAngleY = 0.0f, KalBiasY = 0.0f;
float KalAngleZ = 0.0f, KalBiasZ = 0.0f;
float P_X[2][2] = {{1,0}, {0,1}};
float P_Y[2][2] = {{1,0}, {0,1}};
float P_Z[2][2] = {{1,0}, {0,1}};
float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;

float prev_Kal_Roll = 0.0f, prev_Kal_Pitch = 0.0f, prev_Kal_Yaw = 0.0f;

bool firstRun = true;
unsigned long lastLoopTime = 0;

// 로우패스 필터 변수 (회전각 및 가속도)
float lpf_angularVelocity_Roll = 0.0f;
float lpf_angularVelocity_Pitch = 0.0f;
float lpf_angularVelocity_Yaw = 0.0f;
float lpf_Kal_Roll = 0.0f;
float lpf_Kal_Pitch = 0.0f;
float lpf_Kal_Yaw = 0.0f;

float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 0.0f;

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
    newRate -= bias;
    angle += newRate * dt;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void Parachute_ejection() {
    if (!ejectionStarted) {
        ejectionStarted = true;
        ejectionStartTime = millis();
        digitalWrite(ignitor_relay_pin, LOW);
        Serial.println("\n>>> Deployed!\n");
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU Connection Failed!");
        while (1);
    }

    Serial.println("I2C Connection Succeeded!");
    delay(200);

    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < 100; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(5);
    }

    gx_offset = sumX / 100.0f;
    gy_offset = sumY / 100.0f;
    gz_offset = sumZ / 100.0f;

    pinMode(ignitor_relay_pin, OUTPUT);
    digitalWrite(ignitor_relay_pin, HIGH);

    stabilizeStartTime = millis();
    isStabilizing = true;
    firstRun = true;
    lastLoopTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastLoopTime < LOOP_PERIOD) return;

    float dt = (currentTime - lastLoopTime) / 1000.0f;
    lastLoopTime = currentTime;
    if (dt <= 0) dt = 0.001;

    if (isStabilizing) {
        if (currentTime - stabilizeStartTime >= 3000) {
            isStabilizing = false;
            prev_Kal_Roll = prev_Kal_Pitch = prev_Kal_Yaw = 0.0f;
        } else {
            digitalWrite(ignitor_relay_pin, HIGH);
            Serial.println("Sensor Stabilizing 3s...");
            return;
        }
    }

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int gx_corr = gx - gx_offset;
    int gy_corr = gy - gy_offset;
    int gz_corr = gz - gz_offset;

    float gx_dps = gx_corr / 131.0f;
    float gy_dps = gy_corr / 131.0f;
    float gz_dps = gz_corr / 131.0f;

    float ax_g = ax / 2048.0f;
    float ay_g = ay / 2048.0f;
    float az_g = az / 2048.0f;

    // === 가속도에 로우패스 필터 적용 ===
    lpf_ax = ALPHA * lpf_ax + (1 - ALPHA) * ax_g;
    lpf_ay = ALPHA * lpf_ay + (1 - ALPHA) * ay_g;
    lpf_az = ALPHA * lpf_az + (1 - ALPHA) * az_g;

    // === 회전각 계산 ===
    float Roll = atan2(ay_g, az_g) * 180.0f / PI;
    float denom = sqrt(ay_g * ay_g + az_g * az_g);
    if (denom < 1e-6) denom = 1e-6;
    float Pitch = atan2(-ax_g, denom) * 180.0f / PI;
    float Yaw = Yaw_prev + gz_dps * dt;
    Yaw_prev = Yaw;
    if (Yaw < 0) Yaw += 360.0f;
    if (Yaw >= 360.0f) Yaw -= 360.0f;

    // === 칼만 필터 ===
    if (firstRun) {
        KalAngleX = Roll;
        KalAngleY = Pitch;
        KalAngleZ = Yaw;
        firstRun = false;
    }

    float Kal_Roll = kalmanFilter(Roll, gx_dps, dt, KalAngleX, KalBiasX, P_X);
    float Kal_Pitch = kalmanFilter(Pitch, gy_dps, dt, KalAngleY, KalBiasY, P_Y);
    float Kal_Yaw = kalmanFilter(Yaw, gz_dps, dt, KalAngleZ, KalBiasZ, P_Z);

    float angularVelocity_Roll = (Kal_Roll - prev_Kal_Roll) / dt;
    float angularVelocity_Pitch = (Kal_Pitch - prev_Kal_Pitch) / dt;
    float angularVelocity_Yaw = (Kal_Yaw - prev_Kal_Yaw) / dt;

    lpf_angularVelocity_Roll = ALPHA * lpf_angularVelocity_Roll + (1 - ALPHA) * angularVelocity_Roll;
    lpf_angularVelocity_Pitch = ALPHA * lpf_angularVelocity_Pitch + (1 - ALPHA) * angularVelocity_Pitch;
    lpf_angularVelocity_Yaw = ALPHA * lpf_angularVelocity_Yaw + (1 - ALPHA) * angularVelocity_Yaw;

    lpf_Kal_Roll = ALPHA * lpf_Kal_Roll + (1 - ALPHA) * Kal_Roll;
    lpf_Kal_Pitch = ALPHA * lpf_Kal_Pitch + (1 - ALPHA) * Kal_Pitch;
    lpf_Kal_Yaw = ALPHA * lpf_Kal_Yaw + (1 - ALPHA) * Kal_Yaw;

    bool trigger_X = fabs(lpf_Kal_Roll) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Roll) > TRIGGER_THRESHOLD;
    bool trigger_Y = fabs(lpf_Kal_Pitch) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Pitch) > TRIGGER_THRESHOLD;
    bool trigger_Z = fabs(lpf_Kal_Yaw) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Yaw) > TRIGGER_THRESHOLD;

    if (ejectionStarted && millis() - ejectionStartTime >= 2000) {
        digitalWrite(ignitor_relay_pin, HIGH);
    }

    if ((trigger_X + trigger_Y + trigger_Z) >= 2 && ejcet_option) {
        Parachute_ejection();
    }

    // === 출력 ===
    Serial.print(lpf_Kal_Roll, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Pitch, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Yaw, 2); Serial.print("\t");

    Serial.print(lpf_ax, 2); Serial.print("\t");
    Serial.print(lpf_ay, 2); Serial.print("\t");
    Serial.println(lpf_az, 2);

    prev_Kal_Roll = Kal_Roll;
    prev_Kal_Pitch = Kal_Pitch;
    prev_Kal_Yaw = Kal_Yaw;
}
