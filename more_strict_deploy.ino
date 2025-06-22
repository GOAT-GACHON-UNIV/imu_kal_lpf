#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define ALPHA 0.873
#define LOOP_PERIOD 20 // loop 주기 (ms)

// Roll, Pitch, Yaw 임계각
#define THRESHOLD_ROLL_ANGLE 22.0
#define THRESHOLD_PITCH_ANGLE 18.0
#define THRESHOLD_YAW_ANGLE 35.0
// Roll, Pitch, Yaw 각속도 임계값
#define THRESHOLD_ROLL_RATE 50.0
#define THRESHOLD_PITCH_RATE 40.0
#define THRESHOLD_YAW_RATE 30.0

// 사출 안정성 더하기
#define TRIGGER_HOLD_TIME 200 // 0.2초 유지시 사출
#define TRIGGER_RESET_TIME 500 // 0.5초 이내 릴레이 전압 재인가 방지 
#define PI 3.14159265358979323846
#define ignitor_relay_pin 6

MPU6050 mpu;

// 상태 변수
bool eject_option = true;
bool ejectionStarted = false;
unsigned long ejectionStartTime = 0;

bool isStabilizing = true;
unsigned long stabilizeStartTime = 0;

bool isArming = false;
unsigned long armingStartTime = 0;

// 센서 raw
int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;
float Yaw_prev = 0.0f;

// 칼만 필터
float KalAngleX = 0.0f, KalBiasX = 0.0f;
float KalAngleY = 0.0f, KalBiasY = 0.0f;
float KalAngleZ = 0.0f, KalBiasZ = 0.0f;
float P_X[2][2] = {{1,0}, {0,1}};
float P_Y[2][2] = {{1,0}, {0,1}};
float P_Z[2][2] = {{1,0}, {0,1}};
float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;

// 이전값
float prev_Kal_Roll = 0.0f, prev_Kal_Pitch = 0.0f, prev_Kal_Yaw = 0.0f;
bool firstRun = true;
unsigned long lastLoopTime = 0;

// LPF 변수
float lpf_angularVelocity_Roll = 0.0f;
float lpf_angularVelocity_Pitch = 0.0f;
float lpf_angularVelocity_Yaw = 0.0f;
float lpf_Kal_Roll = 0.0f;
float lpf_Kal_Pitch = 0.0f;
float lpf_Kal_Yaw = 0.0f;

// 칼만 필터 함수
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
    // 추정 단계
    newRate -= bias; // 보정 각가속도의 바이어스를 제거한 값
    //추정 단계 : 보정 각가속도를 적분하여 회전각의 추정값을 구함
    angle += newRate * dt;
    // 오차공분산 (error convarience) 추정
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2]; // 칼만이득
    // 칼만이득 계산
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    // 적절한 칼만이득을 곱하여 추정값및 바이어스 업데이트 
    float y = newAngle - angle;
    // 계산된 칼만이득을 곱하여 회전값과 그 바이어스 보정
    angle += K[0] * y;
    bias  += K[1] * y;  

    // 오차공분산 실시간 업데이트 
    float P_first_buff = P[0][0];
    float P_second_buff = P[0][1];
    P[0][0] -= K[0] * P_first_buff;
    P[0][1] -= K[0] * P_second_buff;
    P[1][0] -= K[1] * P_first_buff;
    P[1][1] -= K[1] * P_second_buff;

    return angle;
}

// 사출 함수
void Parachute_ejection() {
    if (!ejectionStarted) {
        ejectionStarted = true;
        ejectionStartTime = millis();
        digitalWrite(ignitor_relay_pin, LOW);
        Serial.println("\n>>> Deployed!\n");
    }
}

void setup() {
    Serial.begin(230400);
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU Connection Failed!");
        while (1);
    }
    Serial.println("I2C Connection Succeeded!");
    delay(200);

    // 자이로 offset 보정
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < 100; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        sumX += gx; sumY += gy; sumZ += gz;
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

    // === Stabilizing ===
    if (isStabilizing) {
        if (currentTime - stabilizeStartTime >= 2000) {
            isStabilizing = false;
            isArming = true;
            armingStartTime = millis();
            Serial.println("Stabilizing Done, Arming...");
            prev_Kal_Roll = prev_Kal_Pitch = prev_Kal_Yaw = 0.0f;
        } else {
            digitalWrite(ignitor_relay_pin, HIGH);
            Serial.println("Sensor Stabilizing 2s...");
            return;
        }
    }

    // === Arming 딜레이 : 센서의 갑작스러운 튀는 값 방지 ==
    if (isArming) {
        if (currentTime - armingStartTime >= 2000) {
            isArming = false;
            Serial.println("Arming Completed, Ready!");
        } 
        else {
            return;
        }
    }

    // === 센서 읽기 ===
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

    // == 릴레이 전압 발생 조건 == 
    bool trigger_X = fabs(lpf_Kal_Roll) > THRESHOLD_ROLL_ANGLE && fabs(lpf_angularVelocity_Roll) > THRESHOLD_ROLL_RATE;
    bool trigger_Y = fabs(lpf_Kal_Pitch) > THRESHOLD_PITCH_ANGLE && fabs(lpf_angularVelocity_Pitch) > THRESHOLD_PITCH_RATE;
    bool trigger_Z = fabs(lpf_Kal_Yaw) > THRESHOLD_YAW_ANGLE && fabs(lpf_angularVelocity_Yaw) > THRESHOLD_YAW_RATE;

    // === 사출 제어 ===
    // 사출 안정성 강화 : 연산량이 많아 시간차가 존재, 그 시간에 센서가 튀는걸 감지를 못하면 사출되므로 0.2초동안 사출조건이 유지된다면 릴레이전압 가동 
    static unsigned long trigger_time = 0;
    static unsigned long last_eject_time = 0;
    static bool trigger_pending = false;

    if (ejectionStarted && millis() - ejectionStartTime >= 2000) {
        digitalWrite(ignitor_relay_pin, HIGH);
    }

    if ((trigger_X + trigger_Y + trigger_Z) >= 2 && eject_option) {
        if (!trigger_pending) {
            trigger_pending = true;
            trigger_time = millis();
        }
        if (millis() - trigger_time >= TRIGGER_HOLD_TIME && millis() - last_eject_time > TRIGGER_RESET_TIME) {
            last_eject_time = millis();
            Parachute_ejection();
        }
    }
     else {
        trigger_pending = false;
    }

    // === 출력 ===
    Serial.print(Roll, 2); Serial.print("\t");
    Serial.print(Kal_Roll, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Roll, 2); Serial.print("\t");
    Serial.print(Pitch, 2); Serial.print('\t');
    Serial.print(Kal_Pitch, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Pitch, 2); Serial.print("\t");
    Serial.print(Yaw, 2); Serial.print("\t");
    Serial.print(Kal_Yaw, 2); Serial.print("\t");
    Serial.println(lpf_Kal_Yaw, 2);

    // 이전값 갱신
    prev_Kal_Roll = Kal_Roll;
    prev_Kal_Pitch = Kal_Pitch;
    prev_Kal_Yaw = Kal_Yaw;
}
