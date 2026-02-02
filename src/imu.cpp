#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "config.h"
#include "motors.h"
#include "kalman.h"

// --- FreeRTOS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ==================== MUTEX I2C GLOBAL ====================
SemaphoreHandle_t i2c_mutex = nullptr;

// --- Offsets calibration ---
static double gyro_off_x = 0;
static double gyro_off_y = 0;
static double gyro_off_z = 0;

// --- Variables brutes ---
static int16_t acc_raw[3];
static int16_t gyro_raw[3];
static int16_t temperature;

// --- FILTRAGE TYPE MADFLIGHT (PT1) ---
static float gyro_roll_filt = 0.0f;
static float gyro_pitch_filt = 0.0f;
static float gyro_yaw_filt = 0.0f;

// Variables pour le monitoring (Plotter)
static float gyro_roll_raw_dps = 0.0f;
static float gyro_pitch_raw_dps = 0.0f;
static float gyro_yaw_raw_dps = 0.0f;

#if GYRO_PT1_CASCADE
static float gyro_roll_filt2 = 0.0f;
static float gyro_pitch_filt2 = 0.0f;
static float gyro_yaw_filt2 = 0.0f;
#endif

// Constante de temps RC (Fréquence de coupure)
static const float GYRO_PT1_RC = 1.0f / (2.0f * 3.14159265f * GYRO_PT1_CUTOFF_HZ);

// --- Filtres de Kalman ---
static Kalman kalman_roll;
static Kalman kalman_pitch;
static float yaw_angle = 0.0f; 
static bool kalman_initialized = false;

// ==================== SNAPSHOT IMU ====================
typedef struct {
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float gyro_roll_raw;    
    float gyro_pitch_raw;   
    float gyro_yaw_raw;     
    float angle_roll;
    float angle_pitch;
    float angle_yaw;
    float acc_total_vector;
    float acc_x;
    float acc_y;
    float acc_z;
    unsigned long last_dur_us;
    unsigned long last_ok_ms;
    bool ok;
    uint32_t dt_us;
} ImuSnapshot;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot imu_snap = {0};

static volatile bool imu_reset_req = false; // Flag pour demander un reset

static DroneState imu_state;
static TaskHandle_t imu_task_handle = nullptr;

// ==================== FONCTION RESET INTERNE ====================
void imu_request_reset() {
    portENTER_CRITICAL(&imu_mux);
    imu_reset_req = true;
    portEXIT_CRITICAL(&imu_mux);
}

// ==================== IMU INIT ====================
void imu_init() {
    motors_write_direct(1000, 1000, 1000, 1000);
    Serial.println(F("IMU: Init Raw I2C..."));

    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(MPU_DLPF_CFG); Wire.endTransmission();

    Serial.println(F("IMU: Calibration Gyro - NE PAS BOUGER!"));
    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    int valid_samples = 0;
    unsigned long calib_start = millis();

    while (millis() - calib_start < 5000) {
        if ((millis() % 100) < 50) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        int pwm_heartbeat = 1000 + (millis() % 20);
        motors_write_direct(pwm_heartbeat, pwm_heartbeat, pwm_heartbeat, pwm_heartbeat);

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);
        if (Wire.available() == 6) {
            gyro_sum_x += (int16_t)(Wire.read() << 8 | Wire.read());
            gyro_sum_y += (int16_t)(Wire.read() << 8 | Wire.read());
            gyro_sum_z += (int16_t)(Wire.read() << 8 | Wire.read());
            valid_samples++;
        }
        delayMicroseconds(2000);
    }

    if (valid_samples > 0) {
        gyro_off_x = (double)gyro_sum_x / valid_samples;
        gyro_off_y = (double)gyro_sum_y / valid_samples;
        gyro_off_z = (double)gyro_sum_z / valid_samples;
    }

    kalman_roll.setQangle(0.001f); kalman_roll.setQbias(0.003f); kalman_roll.setRmeasure(0.03f);
    kalman_pitch.setQangle(0.001f); kalman_pitch.setQbias(0.003f); kalman_pitch.setRmeasure(0.03f);

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK"));
}

static bool imu_read_internal(DroneState *drone) {
    static unsigned long last_us = 0;
    const unsigned long now_us = micros();

    if (i2c_mutex != nullptr) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(IMU_MUTEX_TIMEOUT_MS)) != pdTRUE) {
            drone->imu_ok = false;
            return false;
        }
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission() != 0) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        drone->imu_ok = false;
        return false;
    }

    Wire.requestFrom(MPU_ADDR, 14);
    if (Wire.available() < 14) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        drone->imu_ok = false;
        return false;
    }

    acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
    temperature = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());

    if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);

    float dt_s = (last_us == 0) ? 0.004f : (now_us - last_us) * 1e-6f;
    if (dt_s < 0.001f) dt_s = 0.001f;
    last_us = now_us;

    const float gyro_scale = 65.5f;
    gyro_roll_raw_dps = (float)((gyro_raw[0] - gyro_off_x) / gyro_scale);
    gyro_pitch_raw_dps = (float)((-gyro_raw[1] - gyro_off_y) / gyro_scale);
    #if IMU_INVERT_YAW
        gyro_yaw_raw_dps = (float)((-gyro_raw[2] - gyro_off_z) / gyro_scale);
    #else
        gyro_yaw_raw_dps = (float)((gyro_raw[2] - gyro_off_z) / gyro_scale);
    #endif

    float alpha = dt_s / (GYRO_PT1_RC + dt_s);
    if (alpha > 1.0f) alpha = 1.0f;

    gyro_roll_filt  += alpha * (gyro_roll_raw_dps  - gyro_roll_filt);
    gyro_pitch_filt += alpha * (gyro_pitch_raw_dps - gyro_pitch_filt);
    gyro_yaw_filt   += alpha * (gyro_yaw_raw_dps   - gyro_yaw_filt);

#if GYRO_PT1_CASCADE
    gyro_roll_filt2  += alpha * (gyro_roll_filt  - gyro_roll_filt2);
    gyro_pitch_filt2 += alpha * (gyro_pitch_filt - gyro_pitch_filt2);
    gyro_yaw_filt2   += alpha * (gyro_yaw_filt   - gyro_yaw_filt2);
    drone->gyro_roll_input = gyro_roll_filt2;
    drone->gyro_pitch_input = gyro_pitch_filt2;
    drone->gyro_yaw_input = gyro_yaw_filt2;
#else
    drone->gyro_roll_input = gyro_roll_filt;
    drone->gyro_pitch_input = gyro_pitch_filt;
    drone->gyro_yaw_input = gyro_yaw_filt;
#endif

    drone->acc_total_vector = sqrtf((float)acc_raw[0]*acc_raw[0] + (float)acc_raw[1]*acc_raw[1] + (float)acc_raw[2]*acc_raw[2]);
    float angle_pitch_acc = asinf((float)acc_raw[0] / drone->acc_total_vector) * RAD_TO_DEG;
    float angle_roll_acc  = asinf((float)acc_raw[1] / drone->acc_total_vector) * RAD_TO_DEG;

    drone->angle_roll  = kalman_roll.update(angle_roll_acc, gyro_roll_raw_dps, dt_s);
    drone->angle_pitch = kalman_pitch.update(angle_pitch_acc, gyro_pitch_raw_dps, dt_s);

    yaw_angle += gyro_yaw_raw_dps * dt_s;
    drone->angle_yaw = yaw_angle;
    
    drone->imu_dt_us = (uint32_t)(dt_s * 1e6f);
    drone->imu_ok = true;
    return true;
}

static void imu_task(void *parameter) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        // Gérer la requête de reset
        bool reset_needed = false;
        portENTER_CRITICAL(&imu_mux);
        if(imu_reset_req) { imu_reset_req = false; reset_needed = true; }
        portEXIT_CRITICAL(&imu_mux);

        if(reset_needed) {
            gyro_roll_filt = 0; gyro_pitch_filt = 0; gyro_yaw_filt = 0;
            yaw_angle = 0; kalman_initialized = false;
        }

        imu_read_internal(&imu_state);
        
        portENTER_CRITICAL(&imu_mux);
        imu_snap.gyro_roll_input   = imu_state.gyro_roll_input;
        imu_snap.gyro_pitch_input  = imu_state.gyro_pitch_input;
        imu_snap.gyro_yaw_input    = imu_state.gyro_yaw_input;
        imu_snap.gyro_roll_raw     = gyro_roll_raw_dps;  
        imu_snap.gyro_pitch_raw    = gyro_pitch_raw_dps; 
        imu_snap.angle_roll        = imu_state.angle_roll;
        imu_snap.angle_pitch       = imu_state.angle_pitch;
        imu_snap.ok                = imu_state.imu_ok;
        imu_snap.dt_us             = imu_state.imu_dt_us;
        portEXIT_CRITICAL(&imu_mux);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4)); 
    }
}

void imu_start_task() {
    if (i2c_mutex == nullptr) i2c_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, nullptr, 4, &imu_task_handle, 0);
}

void imu_update(DroneState *drone) {
    portENTER_CRITICAL(&imu_mux);
    drone->gyro_roll_input = imu_snap.gyro_roll_input;
    drone->gyro_pitch_input = imu_snap.gyro_pitch_input;
    drone->gyro_yaw_input = imu_snap.gyro_yaw_input;
    drone->gyro_raw_roll = imu_snap.gyro_roll_raw; 
    drone->angle_roll = imu_snap.angle_roll;
    drone->angle_pitch = imu_snap.angle_pitch;
    drone->imu_ok = imu_snap.ok;
    portEXIT_CRITICAL(&imu_mux);
}

void imu_read(DroneState *drone) {
    imu_read_internal(drone);
}