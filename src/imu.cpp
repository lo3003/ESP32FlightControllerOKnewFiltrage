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
// Ce mutex est partagé entre imu.cpp et alt_imu.cpp pour éviter les conflits I2C
SemaphoreHandle_t i2c_mutex = nullptr;

// --- Offsets calibration ---
static double gyro_off_x = 0;
static double gyro_off_y = 0;
static double gyro_off_z = 0;

// --- Variables brutes ---
static int16_t acc_raw[3];
static int16_t gyro_raw[3];
static int16_t temperature;

// --- Filtres PT1 pour gyro (entrées PID) ---
static float gyro_roll_filt = 0.0f;
static float gyro_pitch_filt = 0.0f;
static float gyro_yaw_filt = 0.0f;

// PT1 cascade (2nd stage) - utilisé si GYRO_PT1_CASCADE == 1
#if GYRO_PT1_CASCADE
static float gyro_roll_filt2 = 0.0f;
static float gyro_pitch_filt2 = 0.0f;
static float gyro_yaw_filt2 = 0.0f;
#endif

// Constante pour calcul PT1: RC = 1/(2*PI*fc)
static const float GYRO_PT1_RC = 1.0f / (2.0f * 3.14159265f * GYRO_PT1_CUTOFF_HZ);

// --- Filtres de Kalman pour Roll et Pitch ---
static Kalman kalman_roll;
static Kalman kalman_pitch;
static float yaw_angle = 0.0f;  // AJOUT: intégration simple du Yaw
static bool kalman_initialized = false;

// ==================== SNAPSHOT IMU ====================
typedef struct {
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float angle_roll;
    float angle_pitch;
    float angle_yaw;          // AJOUT
    float acc_total_vector;
    float acc_x;              // Accélération X en G
    float acc_y;              // Accélération Y en G
    float acc_z;              // Accélération Z en G
    unsigned long last_dur_us;
    unsigned long last_ok_ms;
    bool ok;
    // --- DEBUG ---
    uint32_t dt_us;           // Delta-time IMU (microsecondes)
    float gyro_raw_roll;      // Gyro brut roll (dps, avant PT1)
    float gyro_raw_pitch;     // Gyro brut pitch (dps, avant PT1)
    float gyro_raw_yaw;       // Gyro brut yaw (dps, avant PT1)
} ImuSnapshot;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot imu_snap = {0};

static volatile FlightMode imu_in_mode = MODE_SAFE;
static volatile int imu_in_ch3 = 1000;
static volatile bool imu_reset_req = false;

static DroneState imu_state;
static TaskHandle_t imu_task_handle = nullptr;

// ==================== IMU INIT ====================
void imu_init() {
    motors_write_direct(1000, 1000, 1000, 1000);

    Serial.println(F("IMU: Init Raw I2C..."));

    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // Wake up
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();  // Gyro ±500°/s
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();  // Accel ±8g
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(MPU_DLPF_CFG); Wire.endTransmission();  // DLPF configurable
    
    Serial.printf("IMU: DLPF_CFG = 0x%02X (fc ~%dHz)\n", MPU_DLPF_CFG,
        (MPU_DLPF_CFG == 0) ? 260 : (MPU_DLPF_CFG == 1) ? 184 : (MPU_DLPF_CFG == 2) ? 94 :
        (MPU_DLPF_CFG == 3) ? 44 : (MPU_DLPF_CFG == 4) ? 21 : (MPU_DLPF_CFG == 5) ? 10 : 5);

    Serial.println(F(""));
    Serial.println(F("IMU: Calibration Gyro MPU6050 - NE PAS BOUGER LE DRONE!"));
    Serial.println(F("IMU: Calibration en cours (5 secondes)..."));

    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    int valid_samples = 0;

    // Calibration pendant 5 secondes avec LED clignotante rapide (50ms)
    unsigned long calib_start = millis();
    unsigned long last_led_toggle = 0;
    bool led_state = false;

    while (millis() - calib_start < 5000) {
        // Clignotement LED rapide (50ms on, 50ms off)
        if (millis() - last_led_toggle >= 50) {
            led_state = !led_state;
            digitalWrite(PIN_LED, led_state);
            last_led_toggle = millis();
        }

        // Signal ESC "heartbeat" variable (1000-1019µs) pour éviter timeout ESC
        // Le signal oscille au lieu d'être statique, ce qui empêche la protection timeout
        int pwm_heartbeat = 1000 + (millis() % 20);
        motors_write_direct(pwm_heartbeat, pwm_heartbeat, pwm_heartbeat, pwm_heartbeat);

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);

        if (Wire.available() < 6) { delay(2); continue; }

        gyro_sum_x += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_y += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_z += (int16_t)(Wire.read() << 8 | Wire.read());
        valid_samples++;

        delayMicroseconds(2000);
    }

    if (valid_samples > 0) {
        gyro_off_x = (double)gyro_sum_x / valid_samples;
        gyro_off_y = (double)gyro_sum_y / valid_samples;
        gyro_off_z = (double)gyro_sum_z / valid_samples;
    }

    Serial.printf("IMU: %d echantillons collectes\n", valid_samples);

    // Configuration Kalman pour MPU6050
    kalman_roll.setQangle(0.001f);
    kalman_roll.setQbias(0.003f);
    kalman_roll.setRmeasure(0.03f);
    
    kalman_pitch.setQangle(0.001f);
    kalman_pitch.setQbias(0.003f);
    kalman_pitch.setRmeasure(0.03f);

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK (Kalman Filter Enabled)"));
}

// ==================== IMU READ INTERNAL ====================
// Retourne true si lecture OK, false sinon (pour ne pas polluer les filtres)
static bool imu_read_internal(DroneState *drone) {
    static unsigned long last_us = 0;
    static unsigned long fail_count = 0;  // DEBUG: compteur d'échecs
    const unsigned long now_us = micros();

    // ========== SECTION I2C PROTEGEE PAR MUTEX ==========
    // Timeout court pour ne pas rater de cycles IMU
    if (i2c_mutex != nullptr) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(IMU_MUTEX_TIMEOUT_MS)) != pdTRUE) {
            // Mutex occupé: NE PAS mettre à jour last_us, conserver anciennes valeurs
            drone->imu_ok = false;
            return false;
        }
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        fail_count++;
        drone->max_time_imu = 888888;
        drone->imu_ok = false;
        // NE PAS mettre à jour last_us -> dt reste cohérent au prochain succès
        return false;
    }

    uint8_t count = Wire.requestFrom(MPU_ADDR, 14);

    if (count < 14) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        fail_count++;
        drone->max_time_imu = 888888;
        drone->imu_ok = false;
        return false;
    }
    
    fail_count = 0;  // Reset si succès
    drone->imu_ok = true;

    acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
    temperature = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());

    // Libérer le mutex après la lecture I2C
    if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
    // ========== FIN SECTION I2C PROTEGEE ==========

    // === CALCUL DT UNIQUEMENT APRES LECTURE REUSSIE ===
    float dt_s = 0.004f;  // Valeur par défaut (250Hz)
    if (last_us != 0) {
        dt_s = (now_us - last_us) * 1e-6f;
        // Clamp pour éviter aberrations
        if (dt_s < 0.002f) dt_s = 0.002f;
        if (dt_s > 0.020f) dt_s = 0.020f;  // Max 20ms (si plusieurs échecs consécutifs)
    }
    last_us = now_us;  // Mis à jour UNIQUEMENT après succès

    double gyro_x_cal = gyro_raw[0] - gyro_off_x;
    double gyro_y_cal = gyro_raw[1] - gyro_off_y;
    double gyro_z_cal = gyro_raw[2] - gyro_off_z;

    // Mapping axes
    double gyro_roll  = gyro_x_cal;
    long acc_roll_val = acc_raw[1];

    double gyro_pitch = -gyro_y_cal;
    long acc_pitch_val = acc_raw[0];

    // CORRECTION: Inverser le signe du gyro Yaw
       // YAW : selon l'orientation de la carte, il peut être nécessaire d'inverser le signe.
    #if IMU_INVERT_YAW
        double gyro_yaw = -gyro_z_cal;
    #else
        double gyro_yaw = gyro_z_cal;
    #endif
        long acc_yaw_val  = acc_raw[2];


    const float gyro_scale = 65.5f;

    // Gyro en degrés/seconde
    float gyro_roll_dps  = (float)(gyro_roll / gyro_scale);
    float gyro_pitch_dps = (float)(gyro_pitch / gyro_scale);
    float gyro_yaw_dps   = (float)(gyro_yaw / gyro_scale);

    // --- DEBUG: Stocker dt_us et gyro brut AVANT filtrage ---
    drone->imu_dt_us = (uint32_t)(dt_s * 1e6f);  // Convertir dt_s en microsecondes
    drone->gyro_raw_roll  = gyro_roll_dps;
    drone->gyro_raw_pitch = gyro_pitch_dps;
    drone->gyro_raw_yaw   = gyro_yaw_dps;

    // === FILTRE PT1 DYNAMIQUE (alpha dépend du dt réel) ===
    // Formule: alpha = dt / (RC + dt), où RC = 1/(2*PI*fc)
    const float alpha = dt_s / (GYRO_PT1_RC + dt_s);

    // Premier étage PT1
    gyro_roll_filt  += alpha * (gyro_roll_dps  - gyro_roll_filt);
    gyro_pitch_filt += alpha * (gyro_pitch_dps - gyro_pitch_filt);
    gyro_yaw_filt   += alpha * (gyro_yaw_dps   - gyro_yaw_filt);

#if GYRO_PT1_CASCADE
    // Deuxième étage PT1 (cascade = -40dB/dec au lieu de -20dB/dec)
    gyro_roll_filt2  += alpha * (gyro_roll_filt  - gyro_roll_filt2);
    gyro_pitch_filt2 += alpha * (gyro_pitch_filt - gyro_pitch_filt2);
    gyro_yaw_filt2   += alpha * (gyro_yaw_filt   - gyro_yaw_filt2);

    drone->gyro_roll_input  = gyro_roll_filt2;
    drone->gyro_pitch_input = gyro_pitch_filt2;
    drone->gyro_yaw_input   = gyro_yaw_filt2;
#else
    drone->gyro_roll_input  = gyro_roll_filt;
    drone->gyro_pitch_input = gyro_pitch_filt;
    drone->gyro_yaw_input   = gyro_yaw_filt;
#endif

    // Calcul du vecteur accélération total
    drone->acc_total_vector = sqrtf((float)(acc_roll_val * acc_roll_val) +
                                    (float)(acc_pitch_val * acc_pitch_val) +
                                    (float)(acc_yaw_val * acc_yaw_val));

    // Accélération normalisée en G (1G = 4096 LSB pour ±8g)
    const float ACC_SCALE = 4096.0f;
    drone->acc_x = (float)acc_pitch_val / ACC_SCALE;  // X = acc_pitch
    drone->acc_y = (float)acc_roll_val / ACC_SCALE;   // Y = acc_roll
    drone->acc_z = (float)acc_yaw_val / ACC_SCALE;    // Z = acc_z

    // Calcul angles accéléromètre
    float angle_pitch_acc = 0.0f, angle_roll_acc = 0.0f;

    if (fabsf((float)acc_pitch_val) < drone->acc_total_vector) {
        angle_pitch_acc = asinf((float)acc_pitch_val / drone->acc_total_vector) * RAD_TO_DEG;
    }
    if (fabsf((float)acc_roll_val) < drone->acc_total_vector) {
        angle_roll_acc = asinf((float)acc_roll_val / drone->acc_total_vector) * RAD_TO_DEG;
    }

    // Trim mécanique (ajuster selon calibration physique du drone)
    angle_roll_acc  += 0.8f;
    angle_pitch_acc += -3.7f;

    // --- FILTRE DE KALMAN ---
    if (!kalman_initialized) {
        kalman_roll.setAngle(angle_roll_acc);
        kalman_pitch.setAngle(angle_pitch_acc);
        drone->angle_roll = angle_roll_acc;
        drone->angle_pitch = angle_pitch_acc;
        kalman_initialized = true;
    } else {
        // Mode adaptatif
        if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
            kalman_roll.setRmeasure(0.01f);
            kalman_pitch.setRmeasure(0.01f);
        } else {
            kalman_roll.setRmeasure(0.03f);
            kalman_pitch.setRmeasure(0.03f);
        }

        drone->angle_roll  = kalman_roll.update(angle_roll_acc, gyro_roll_dps, dt_s);
        drone->angle_pitch = kalman_pitch.update(angle_pitch_acc, gyro_pitch_dps, dt_s);
    }

    // AJOUT: Intégration Yaw (pas de correction accéléromètre possible)
    yaw_angle += gyro_yaw_dps * dt_s;
    
    // Normalisation -180 à +180
    while (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    while (yaw_angle < -180.0f) yaw_angle += 360.0f;
    
    drone->angle_yaw = yaw_angle;

    // Compensation yaw - DESACTIVER POUR TEST
    #if 0  // <-- Mettre 1 pour réactiver
    const float yaw_rad = gyro_yaw_dps * dt_s * (PI / 180.0f);
    const float s = sinf(yaw_rad);

    const float roll0  = drone->angle_roll;
    const float pitch0 = drone->angle_pitch;

    drone->angle_pitch = pitch0 - roll0  * s;
    drone->angle_roll  = roll0  + pitch0 * s;
    #endif

    return true;  // Lecture réussie
}

// ==================== TASK IMU ====================
static void imu_task(void *parameter) {
    (void)parameter;

    memset(&imu_state, 0, sizeof(imu_state));
    imu_state.current_mode = MODE_SAFE;
    imu_state.channel_3 = 1000;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        FlightMode m;
        int ch3;
        bool do_reset = false;

        portENTER_CRITICAL(&imu_mux);
        m = imu_in_mode;
        ch3 = imu_in_ch3;
        if (imu_reset_req) { imu_reset_req = false; do_reset = true; }
        portEXIT_CRITICAL(&imu_mux);

        imu_state.current_mode = m;
        imu_state.channel_3 = ch3;

        if (do_reset) {
            imu_state.angle_pitch = 0.0f;
            imu_state.angle_roll  = 0.0f;
            imu_state.gyro_roll_input = 0.0f;
            imu_state.gyro_pitch_input = 0.0f;
            imu_state.gyro_yaw_input = 0.0f;
            
            kalman_roll.setAngle(0.0f);
            kalman_pitch.setAngle(0.0f);
            kalman_initialized = false;
            yaw_angle = 0.0f;  // AJOUT: reset angle yaw
            
            gyro_roll_filt = 0.0f;
            gyro_pitch_filt = 0.0f;
            gyro_yaw_filt = 0.0f;
#if GYRO_PT1_CASCADE
            gyro_roll_filt2 = 0.0f;
            gyro_pitch_filt2 = 0.0f;
            gyro_yaw_filt2 = 0.0f;
#endif
        }

        unsigned long t0 = micros();
        bool read_ok = imu_read_internal(&imu_state);
        unsigned long dur = micros() - t0;

        bool ok = read_ok && (imu_state.max_time_imu != 888888);

        portENTER_CRITICAL(&imu_mux);
        imu_snap.gyro_roll_input  = imu_state.gyro_roll_input;
        imu_snap.gyro_pitch_input = imu_state.gyro_pitch_input;
        imu_snap.gyro_yaw_input   = imu_state.gyro_yaw_input;
        imu_snap.angle_roll       = imu_state.angle_roll;
        imu_snap.angle_pitch      = imu_state.angle_pitch;
        imu_snap.angle_yaw        = imu_state.angle_yaw;  // AJOUT
        imu_snap.acc_total_vector = imu_state.acc_total_vector;
        imu_snap.acc_x            = imu_state.acc_x;
        imu_snap.acc_y            = imu_state.acc_y;
        imu_snap.acc_z            = imu_state.acc_z;
        imu_snap.last_dur_us      = dur;
        imu_snap.ok               = ok;
        imu_state.imu_ok          = ok;  // Propager le flag
        if (ok) imu_snap.last_ok_ms = millis();
        // --- DEBUG: Copier dt_us et gyro brut ---
        imu_snap.dt_us          = imu_state.imu_dt_us;
        imu_snap.gyro_raw_roll  = imu_state.gyro_raw_roll;
        imu_snap.gyro_raw_pitch = imu_state.gyro_raw_pitch;
        imu_snap.gyro_raw_yaw   = imu_state.gyro_raw_yaw;
        portEXIT_CRITICAL(&imu_mux);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4)); // 250 Hz
    }
}

// ==================== API PUBLIQUE ====================
void imu_start_task() {
    if (imu_task_handle != nullptr) {
        Serial.println(F("IMU: Task already running!"));
        return;
    }

    // Créer le mutex I2C s'il n'existe pas encore
    if (i2c_mutex == nullptr) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == nullptr) {
            Serial.println(F("IMU: ERREUR - Impossible de créer le mutex I2C!"));
            return;
        }
        Serial.println(F("IMU: Mutex I2C créé"));
    }

    Serial.println(F("IMU: Starting FreeRTOS task..."));

    xTaskCreatePinnedToCore(
        imu_task,
        "imu_i2c",
        4096,
        nullptr,
        4,
        &imu_task_handle,
        0
    );
    
    Serial.println(F("IMU: Task started successfully"));
}

void imu_update(DroneState *drone) {
    static FlightMode last_mode = MODE_SAFE;

    ImuSnapshot s;

    portENTER_CRITICAL(&imu_mux);
    imu_in_mode = drone->current_mode;
    imu_in_ch3  = drone->channel_3;
    s = imu_snap;
    portEXIT_CRITICAL(&imu_mux);

    if ((drone->current_mode != last_mode) &&
        (drone->current_mode == MODE_SAFE || drone->current_mode == MODE_ARMED)) {
        imu_request_reset();
    }
    last_mode = drone->current_mode;

    drone->gyro_roll_input  = s.gyro_roll_input;
    drone->gyro_pitch_input = s.gyro_pitch_input;
    drone->gyro_yaw_input   = s.gyro_yaw_input;
    drone->angle_roll       = s.angle_roll;
    drone->angle_pitch      = s.angle_pitch;
    drone->angle_yaw        = s.angle_yaw;  // AJOUT
    drone->acc_total_vector = s.acc_total_vector;
    drone->acc_x            = s.acc_x;
    drone->acc_y            = s.acc_y;
    drone->acc_z            = s.acc_z;
    drone->current_time_imu = s.last_dur_us;
    // --- DEBUG: Copier dt_us et gyro brut ---
    drone->imu_dt_us        = s.dt_us;
    drone->gyro_raw_roll    = s.gyro_raw_roll;
    drone->gyro_raw_pitch   = s.gyro_raw_pitch;
    drone->gyro_raw_yaw     = s.gyro_raw_yaw;
    drone->imu_ok           = s.ok;
}

void imu_request_reset() {
    portENTER_CRITICAL(&imu_mux);
    imu_reset_req = true;
    portEXIT_CRITICAL(&imu_mux);
}

void imu_read(DroneState *drone) {
    imu_read_internal(drone);
}