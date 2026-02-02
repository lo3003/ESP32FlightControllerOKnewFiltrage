#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

// Modes de vol
typedef enum {
    MODE_SAFE,           // Moteurs coupés, LED Fixe
    MODE_CALIBRATION,    // Mode spécial Calibration ESC
    MODE_SETUP,          // Mode spécial Setup Wizard
    MODE_PRE_ARM,        // Attente armement
    MODE_ARMED,          // Moteurs au ralenti
    MODE_FLYING,         // Boucle de vol complète
    MODE_WEB_TEST        // Mode de test via interface web
} FlightMode;

// Etat global (partagé entre les modules)
typedef struct {
    // Mode
    FlightMode current_mode;

    // Radio (1000-2000)
    int channel_1; // Roll
    int channel_2; // Pitch
    int channel_3; // Throttle
    int channel_4; // Yaw

    // IMU
    float gyro_roll_input;   // deg/s
    float gyro_pitch_input;  // deg/s
    float gyro_yaw_input;    // deg/s
    float angle_roll;        // deg
    float angle_pitch;       // deg
    float angle_yaw;         // deg (intégré gyro, pas utilisé pour PID yaw)
    float acc_total_vector;
    
    // Accélération normalisée (en G) pour PoC drift
    float acc_x;             // G (axe X)
    float acc_y;             // G (axe Y)
    float acc_z;             // G (axe Z)

    // PID setpoints (rate)
    float pid_setpoint_roll;
    float pid_setpoint_pitch;
    float pid_setpoint_yaw;

    // PID outputs (mixage)
    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

    // Gains PID
    float p_pitch_roll;
    float i_pitch_roll;
    float d_pitch_roll;

    float p_yaw;
    float i_yaw;
    float d_yaw;

    // FeedForward (gain appliqué à la consigne pilote en RATE)
    float ff_pitch_roll;  // commun roll/pitch
    float ff_yaw;

    // Auto-level (outer loop)
    float p_level;

    // Heading Hold (maintien de cap)
    float p_heading;

    // Web test
    int web_test_vals[5];

    // Timing / debug
    unsigned long loop_time;
    unsigned long max_time_radio;
    unsigned long max_time_imu;
    unsigned long max_time_pid;

    // Durée IMU (task) exposée au main
    unsigned long current_time_imu;

    // --- DEBUG IMU (logging série) ---
    uint32_t imu_dt_us;              // Delta-time IMU en microsecondes
    float gyro_raw_roll;             // Gyro brut roll (dps, avant filtrage)
    float gyro_raw_pitch;            // Gyro brut pitch (dps, avant filtrage)
    float gyro_raw_yaw;              // Gyro brut yaw (dps, avant filtrage)
    bool imu_ok;                     // true si dernière lecture IMU valide

    // Tension batterie (V)
    float voltage_bat;

    // --- Second IMU (AltIMU-10 v2) pour comparaison ---
    float alt_acc_x;         // G (axe X)
    float alt_acc_y;         // G (axe Y)
    float alt_acc_z;         // G (axe Z)
    float alt_gyro_roll;     // deg/s
    float alt_gyro_pitch;    // deg/s
    float alt_gyro_yaw;      // deg/s
    float alt_angle_roll;    // deg (Kalman)
    float alt_angle_pitch;   // deg (Kalman)
    float alt_angle_yaw;     // deg (magnétomètre heading)
} DroneState;

#endif
