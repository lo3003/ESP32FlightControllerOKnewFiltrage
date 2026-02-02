#include "pid.h"
#include "config.h"
#include <Arduino.h>

// --- TENSION DE REFERENCE POUR LA COMPENSATION (A ajuster selon ta batterie) ---
// 12.0f pour 3S (11.1V nominal, 12.6V max)
// 16.0f pour 4S (14.8V nominal, 16.8V max)
#define PID_REF_VOLTAGE 12.6f 

// --- FLAG POUR DESACTIVER LE HEADING HOLD ---
// Mettre à 1 pour activer, 0 pour désactiver (mode simple)
#define HEADING_HOLD_ENABLED 1

// --- MEMOIRES PID ---
// Note : pid_last_..._input stocke la dernière MESURE (Gyro) pour le D
static float pid_i_mem_roll,  pid_last_roll_input;
static float pid_i_mem_pitch, pid_last_pitch_input;
static float pid_i_mem_yaw,   pid_last_yaw_input;

static float pid_roll_d_filter_old;
static float pid_pitch_d_filter_old;
static float pid_yaw_d_filter_old;

// FeedForward : consigne "pilote" (rate) mémorisée depuis pid_compute_setpoints()
static float ff_sp_roll = 0.0f;
static float ff_sp_pitch = 0.0f;
static float ff_sp_yaw = 0.0f;

// --- HEADING HOLD (Maintien de Cap) ---
static float yaw_target_angle = 0.0f;   // Cap cible mémorisé
static bool yaw_heading_lock = false;   // Flag: lock de cap actif
static float last_valid_yaw = 0.0f;     // Dernier yaw valide mémorisé

// Paramètres Heading Hold
static const float YAW_DEADBAND = 20.0f;        // Deadband stick yaw (±20 autour de 1500)
static const float YAW_RATE_LIMIT = 200.0f;     // Limite max de la consigne rate générée (deg/s)
static const float YAW_JUMP_THRESHOLD = 30.0f;  // Seuil de détection de saut aberrant (deg)

// Timer de sécurité pour ne pas resetter le I sur une micro-coupure radio
static unsigned long pid_inflight_timer = 0;

// --- INITIALISATION DES PARAMETRES PID ---
// Appelé au démarrage pour charger des valeurs par défaut
void pid_init_params(DroneState *drone) {
    // ROLL/PITCH
    drone->p_pitch_roll = PID_P_ROLL;
    drone->i_pitch_roll = PID_I_ROLL;
    drone->d_pitch_roll = PID_D_ROLL;

    // YAW (rate)
    drone->p_yaw = PID_P_YAW;
    drone->i_yaw = PID_I_YAW;
    drone->d_yaw = PID_D_YAW;

    // FEEDFORWARD
    drone->ff_pitch_roll = 0;
    drone->ff_yaw        = 0;

    // AUTO LEVEL
    drone->p_level = 4.0f;

    // HEADING HOLD
    drone->p_heading = 1.1f;

    Serial.println("PID Params Initialized (Flight-Ready Mode + VBat Comp)");
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0;  pid_last_roll_input = 0;  pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_input = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0;   pid_last_yaw_input = 0;   pid_yaw_d_filter_old = 0;
    pid_inflight_timer = 0;

    ff_sp_roll = ff_sp_pitch = ff_sp_yaw = 0.0f;

    // Reset Heading Hold
    yaw_target_angle = 0.0f;
    yaw_heading_lock = false;
    last_valid_yaw = 0.0f;
}

// --- CALCUL DES CONSIGNES (Setpoints) ---
void pid_compute_setpoints(DroneState *drone) {
    // --- ROLL (stick) ---
    float stick_roll = 0.0f;
    if (drone->channel_1 > 1520)      stick_roll = (float)(drone->channel_1 - 1520);
    else if (drone->channel_1 < 1480) stick_roll = (float)(drone->channel_1 - 1480);

    // FeedForward = uniquement la commande pilote (rate)
    ff_sp_roll = stick_roll / 3.0f;

    // Self-level (outer loop)
    float input_roll = stick_roll - (drone->angle_roll * drone->p_level);
    drone->pid_setpoint_roll = input_roll / 3.0f;

    // --- PITCH (stick) ---
    float stick_pitch = 0.0f;
    if (drone->channel_2 > 1520)      stick_pitch = (float)(drone->channel_2 - 1520);
    else if (drone->channel_2 < 1480) stick_pitch = (float)(drone->channel_2 - 1480);

    ff_sp_pitch = stick_pitch / 3.0f;

    float input_pitch = stick_pitch - (drone->angle_pitch * drone->p_level);
    drone->pid_setpoint_pitch = input_pitch / 3.0f;

    // --- YAW ---
#if HEADING_HOLD_ENABLED
    float raw_stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        raw_stick_yaw = (float)(drone->channel_4 - 1500);
    }

#if RC_INVERT_YAW
    raw_stick_yaw = -raw_stick_yaw;
#endif

    bool stick_in_deadband = (fabsf(raw_stick_yaw) < YAW_DEADBAND);

    if (stick_in_deadband) {
        // --- Heading Hold ---
        float current_yaw = drone->angle_yaw;

        float yaw_delta = current_yaw - last_valid_yaw;
        while (yaw_delta > 180.0f)  yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;

        if (fabsf(yaw_delta) > YAW_JUMP_THRESHOLD && yaw_heading_lock) {
            current_yaw = last_valid_yaw;
        } else {
            last_valid_yaw = current_yaw;
        }

        if (!yaw_heading_lock) {
            yaw_target_angle = current_yaw;
            last_valid_yaw = current_yaw;
            yaw_heading_lock = true;
        }

        float error_angle = yaw_target_angle - current_yaw;
        while (error_angle > 180.0f)  error_angle -= 360.0f;
        while (error_angle < -180.0f) error_angle += 360.0f;

        float rate_setpoint = error_angle * drone->p_heading;

        if (rate_setpoint > YAW_RATE_LIMIT) rate_setpoint = YAW_RATE_LIMIT;
        else if (rate_setpoint < -YAW_RATE_LIMIT) rate_setpoint = -YAW_RATE_LIMIT;

        drone->pid_setpoint_yaw = rate_setpoint;
        ff_sp_yaw = 0.0f;

    } else {
        // --- Pilot Control ---
        yaw_heading_lock = false;
        last_valid_yaw = drone->angle_yaw;

        float stick_yaw = 0.0f;
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);

#if RC_INVERT_YAW
        stick_yaw = -stick_yaw;
#endif

        ff_sp_yaw = stick_yaw;
        drone->pid_setpoint_yaw = stick_yaw;
    }

#else
    // --- YAW SIMPLE ---
    float stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);
    }

#if RC_INVERT_YAW
    stick_yaw = -stick_yaw;
#endif

    ff_sp_yaw = stick_yaw;
    drone->pid_setpoint_yaw = stick_yaw;
#endif
}

// --- BOUCLE PID PRINCIPALE (Avec V-Bat Compensation) ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // 1) DETECTION "IN FLIGHT"
    if (drone->channel_3 > 1020) {
        pid_inflight_timer = millis();
    }
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2) TPA (Throtte PID Attenuation)
    // Réduit les gains quand on met plein gaz pour éviter les oscillations
    float tpa_factor = 1.0f;
    if (drone->channel_3 > 1500) {
        // Décommenter la ligne suivante pour activer le TPA
        tpa_factor = map(drone->channel_3, 1500, 2000, 100, 90) / 100.0f;
    }

    // 3) COMPENSATION TENSION BATTERIE (V-Bat)
    // Augmente les gains quand la batterie faiblit (les moteurs ont moins de couple)
    float vbat_compensation = 1.0f;
    if (drone->voltage_bat > 6.0f) { // Sécurité : on ne compense que si lecture > 6V
        vbat_compensation = PID_REF_VOLTAGE / drone->voltage_bat;
        
        // Bornes de sécurité (±30%)
        if (vbat_compensation > 1.3f) vbat_compensation = 1.3f;
        if (vbat_compensation < 0.7f) vbat_compensation = 0.7f;
    }

    // SCALER GLOBAL : Combine TPA et V-Bat
    float pid_scaler = tpa_factor * vbat_compensation;


    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;

    float p_term_roll = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    
    float d_term_roll = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_roll + pid_i_mem_roll + d_term_roll;
        if (fabsf(output_before_i) < PID_MAX_ROLL) {
            pid_i_mem_roll += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
        else if (pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    } else {
        pid_i_mem_roll = 0;
    }

    float ff_term_roll = (drone->ff_pitch_roll * pid_scaler) * ff_sp_roll;

    drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll + ff_term_roll;
    if (drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if (drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;

    float p_term_pitch = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;
    
    float d_term_pitch = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_pitch + pid_i_mem_pitch + d_term_pitch;
        if (fabsf(output_before_i) < PID_MAX_PITCH) {
            pid_i_mem_pitch += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
        else if (pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    } else {
        pid_i_mem_pitch = 0;
    }

    float ff_term_pitch = (drone->ff_pitch_roll * pid_scaler) * ff_sp_pitch;

    drone->pid_output_pitch = p_term_pitch + pid_i_mem_pitch + d_term_pitch + ff_term_pitch;
    if (drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if (drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;

    float p_term_yaw = (drone->p_yaw * pid_scaler) * error;

    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;
    
    float d_term_yaw = (drone->d_yaw * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_yaw + pid_i_mem_yaw + d_term_yaw;
        if (fabsf(output_before_i) < PID_MAX_YAW) {
            pid_i_mem_yaw += drone->i_yaw * error;
        }
        if (pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
        else if (pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    } else {
        pid_i_mem_yaw = 0;
    }

    float ff_term_yaw = (drone->ff_yaw * pid_scaler) * ff_sp_yaw;

    drone->pid_output_yaw = p_term_yaw + pid_i_mem_yaw + d_term_yaw + ff_term_yaw;
    if (drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if (drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}