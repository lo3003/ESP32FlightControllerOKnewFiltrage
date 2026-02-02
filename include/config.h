#ifndef CONFIG_H
#define CONFIG_H

// --- SERIAL IMU DEBUG ---
// Active l'affichage série des données IMU pour diagnostic filtrage
#define SERIAL_IMU_DEBUG          1      // 1 = activer, 0 = désactiver
#define SERIAL_IMU_DEBUG_HZ       20     // Fréquence d'affichage (10-20 Hz recommandé)
#define SERIAL_IMU_DEBUG_ONLY_ARMED 0    // 1 = afficher uniquement si armé/en vol

// --- PINOUT (ESP32 30-PIN) ---
// Adapté à votre câblage physique :
#define PIN_MOTOR_1    27
#define PIN_MOTOR_2    13
#define PIN_MOTOR_3    25
#define PIN_MOTOR_4    26

// --- BATTERIE ---
#define PIN_BATTERY    34
#define BAT_SCALE      5.88f  // Calibré: 12.19V mesurés pour 2.09V lus

#define ESC_FREQ       250 

// --- RADIO (S.BUS) ---
#define PIN_SBUS_RX    4   
#define D_FILTER_COEFF 0.18f 
#define PIN_LED        5

// --- I2C ---
#define GYRO_ADDRESS   0x68
#define I2C_SPEED      400000

// Mettre à 1 si ton yaw est juste dans le mauvais sens (commande pilote inversée)
#define IMU_INVERT_YAW 1
#define RC_INVERT_YAW  0

// --- PARAMETRES DE VOL (PID) ---
#define PID_P_ROLL     2
#define PID_I_ROLL     0.01
#define PID_D_ROLL     4
#define PID_MAX_ROLL   400

#define PID_P_PITCH    2
#define PID_I_PITCH    0.01
#define PID_D_PITCH    4
#define PID_MAX_PITCH  400

#define PID_P_YAW      2
#define PID_I_YAW      0.0002
#define PID_D_YAW      0
#define PID_MAX_YAW    400

// --- REGLAGES MOTEURS ---
#define MAX_THROTTLE_FLIGHT 1700
#define MIN_THROTTLE_IDLE   1050
#define MOTOR_OFF           1000

#define LOOP_TIME_US   4000 

// --- IMU Calibration tuning ---
#define IMU_CALIB_SETTLE_MS     200     // temps après config (stabilisation capteur)
#define IMU_CALIB_DISCARD       50      // lectures jetées au début
#define IMU_CALIB_SAMPLES       1500    // échantillons utiles (plus = plus stable)
#define IMU_CALIB_DELAY_US      2000    // spacing entre échantillons
#define IMU_CALIB_STD_MAX_RAW   8.0f    // seuil "ça bouge" (en unités brutes gyro)

// --- IMU FILTERING (hover stability) ---
// DLPF MPU6050: 0x00=260Hz, 0x01=184Hz, 0x02=94Hz, 0x03=44Hz, 0x04=21Hz, 0x05=10Hz, 0x06=5Hz
#define MPU_DLPF_CFG            0x02    // 44Hz pour hover (moins de bruit, latence acceptable)

// PT1 Gyro Filter: cutoff frequency en Hz (15Hz = bon compromis hover/réactivité)
#define GYRO_PT1_CUTOFF_HZ      60.0   // Fréquence de coupure du filtre PT1

// PT1 Cascade: 1 = 2 filtres PT1 en série (atténuation -40dB/dec), 0 = simple PT1 (-20dB/dec)
#define GYRO_PT1_CASCADE        0       // Recommandé pour hover stabilisé

// Mutex I2C timeout (ms) - court pour ne pas rater de cycles IMU
#define IMU_MUTEX_TIMEOUT_MS    2       // 2ms max d'attente mutex


// --- Paramètres de filtrage ---
#define GYRO_LPF_CUTOFF 80.0f    // Fréquence de coupure pour le Gyro (Hz)
#define DTERM_LPF_CUTOFF 40.0f   // Fréquence de coupure plus basse pour le terme D (plus de lissage)

#endif