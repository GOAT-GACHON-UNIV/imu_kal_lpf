#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define ALPHA 0.873 //고속푸리에 변환을 통해 계산 
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
// 칼만필터 초기값 설정 
float KalAngleX = 0.0f, KalBiasX = 0.0f;
float KalAngleY = 0.0f, KalBiasY = 0.0f;
float KalAngleZ = 0.0f, KalBiasZ = 0.0f;
float P_X[2][2] = {{1,0}, {0,1}};
float P_Y[2][2] = {{1,0}, {0,1}};
float P_Z[2][2] = {{1,0}, {0,1}};
float Q_angle = 0.001f;
float Q_bias = 0.003f; //오차공분산의 예측값, 오차공분산의 예측값에 비례 
float R_measure = 0.03f; //측정잡음의 공분산, 측정잡음에 비례 

float prev_Kal_Roll = 0.0f;
float prev_Kal_Pitch = 0.0f;
float prev_Kal_Yaw = 0.0f;

bool firstRun = true;
unsigned long lastLoopTime = 0;

float lpf_angularVelocity_Roll = 0.0f;
float lpf_angularVelocity_Pitch = 0.0f;
float lpf_angularVelocity_Yaw = 0.0f;
float lpf_Kal_Roll = 0.0f;
float lpf_Kal_Pitch = 0.0f;
float lpf_Kal_Yaw = 0.0f;

// 칼만 예측(K) = PH*(PH + R)^-1 H : 출력벡터 
// 1차원 칼만 필터를 
// 예측 + 보정 단계
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
    //상태예측
    newRate -= bias; // 각도의 추정값 
    angle += newRate * dt; // 각속도 수치적분 = 각도 
    // 오차공분산 예측 : 2차원 상태 벡터 P=[angle, bias]
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle); // 각도 오차분산 
    // 각도와 바이어스간의 오차분산
    P[0][1] -= dt * P[1][1]; 
    P[1][0] -= dt * P[1][1]; 
    P[1][1] += Q_bias * dt; //바이어스 오차분산 

    float S = P[0][0] + R_measure; //각도 오차분산 + 측정 오차공분산 
    float K[2]; // 
    // 칼만이득 계산 
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 칼만 예측 보정 단계 
    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1]; 
    // 오차 공분산 엡데이트
    // Pk = (I-KH)*Pk-1 I : 아이젠 벡터 
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
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU Connection Failed!");
        while (1);
    }
    //Serial.print("MPU 6050");
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

    // loop 주기 유지
    if (currentTime - lastLoopTime < LOOP_PERIOD) {
        return;
    }

    float dt = (currentTime - lastLoopTime) / 1000.0f;
    lastLoopTime = currentTime;
    if (dt <= 0) dt = 0.001;

    // 센서 초기 안정화 구간
    if (isStabilizing) {
        if (currentTime - stabilizeStartTime >= 3000) {
            isStabilizing = false;
            prev_Kal_Roll = 0.0f;
            prev_Kal_Pitch = 0.0f;
            prev_Kal_Yaw = 0.0f;
        } else {
            digitalWrite(ignitor_relay_pin, HIGH);
            Serial.println("Sensor Stabilizing 3s...");
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

    // === 회전각(orientation) 계산(좌표계 변환) === 
    float Roll = atan2(ay_g, az_g) * 180.0f / PI;

    float denom = sqrt(ay_g * ay_g + az_g * az_g);
    if (denom < 1e-6) denom = 1e-6;
    float Pitch = atan2(-ax_g, denom) * 180.0f / PI;
    float Yaw = Yaw_prev + gz_dps * dt;
    Yaw_prev = Yaw;
    if (Yaw < 0) Yaw += 360.0f;
    if (Yaw >= 360.0f) Yaw -= 360.0f;

    // === 칼만 필터 ===
    // 처음값은 원본데이터로 처리 그 다음부터 필터링된 값을 사용
    if (firstRun) {
        KalAngleX = Roll;
        KalAngleY = Pitch;
        KalAngleZ = Yaw;
        firstRun = false;
    }

    float Kal_Roll = kalmanFilter(Roll, gx_dps, dt, KalAngleX, KalBiasX, P_X);
    float Kal_Pitch = kalmanFilter(Pitch, gy_dps, dt, KalAngleY, KalBiasY, P_Y);
    float Kal_Yaw = kalmanFilter(Yaw, gz_dps, dt, KalAngleZ, KalBiasZ, P_Z);

    // === 각속도 LPF ===
    float angularVelocity_Roll = (Kal_Roll - prev_Kal_Roll) / dt;
    float angularVelocity_Pitch = (Kal_Pitch - prev_Kal_Pitch) / dt;
    float angularVelocity_Yaw = (Kal_Yaw - prev_Kal_Yaw) / dt;

    lpf_angularVelocity_Roll = ALPHA * lpf_angularVelocity_Roll + (1 - ALPHA) * angularVelocity_Roll;
    lpf_angularVelocity_Pitch = ALPHA * lpf_angularVelocity_Pitch + (1 - ALPHA) * angularVelocity_Pitch;
    lpf_angularVelocity_Yaw = ALPHA * lpf_angularVelocity_Yaw + (1 - ALPHA) * angularVelocity_Yaw;

    // === Roll Pitch Yaw 칼만예측 로우패스필터 처리 ===
    lpf_Kal_Roll = ALPHA * lpf_Kal_Roll + (1 - ALPHA) * Kal_Roll;
    lpf_Kal_Pitch = ALPHA * lpf_Kal_Pitch + (1 - ALPHA) * Kal_Pitch;
    lpf_Kal_Yaw = ALPHA * lpf_Kal_Yaw + (1 - ALPHA) * Kal_Yaw;
    // === Trigger 판단 ===
    bool trigger_X = fabs(lpf_Kal_Roll) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Roll) > TRIGGER_THRESHOLD;
    bool trigger_Y = fabs(lpf_Kal_Pitch) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Pitch) > TRIGGER_THRESHOLD;
    bool trigger_Z = fabs(lpf_Kal_Yaw) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Yaw) > TRIGGER_THRESHOLD;

    if (ejectionStarted && millis() - ejectionStartTime >= 2000) {
        digitalWrite(ignitor_relay_pin, HIGH);
    }

    if ((trigger_X + trigger_Y + trigger_Z) >= 2 && ejcet_option) {
        Parachute_ejection();
        // Serial.println("Deploy!");
    }

    // === 출력 ===
    Serial.print(Roll, 2); Serial.print("\t");
    Serial.print(Kal_Roll, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Roll, 2); Serial.print("\t");
    Serial.print(Pitch, 2); Serial.print("\t");
    Serial.print(Kal_Pitch, 2); Serial.print("\t");
    Serial.print(lpf_Kal_Pitch, 2); Serial.print("\t");
    Serial.print(Yaw, 2); Serial.print("\t");
    Serial.print(Kal_Yaw, 2); Serial.print("\t");    
    Serial.println(lpf_Kal_Yaw, 2);    
    //Serial.print(Kal_Pitch, 2); Serial.print(","); 
    //Serial.print(Yaw, 2); Serial.print(",");       
    //Serial.println(Kal_Yaw, 2);                    

    // === 상태 업데이트 ===
    prev_Kal_Roll = Kal_Roll;
    prev_Kal_Pitch = Kal_Pitch;
    prev_Kal_Yaw = Kal_Yaw;
}
