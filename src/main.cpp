#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "alt_imu.h"
#include "yaw_fusion.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"
#include "telemetry.h"

// --- FLAG POUR DESACTIVER LA FUSION YAW ---
// Mettre à 1 pour activer, 0 pour désactiver
#define YAW_FUSION_ENABLED 1

DroneState drone;
unsigned long loop_timer;
unsigned long arming_timer = 0;
unsigned long disarm_debounce_timer = 0; // Chrono pour la coupure Radio
unsigned long angle_security_timer = 0;  // Chrono pour l'angle excessif
int error_code = 0;                      // 0=OK, 1=CRASH ANGLE, 2=PERTE RADIO

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    Wire.setTimeOut(1);

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BATTERY, INPUT);
    analogReadResolution(12);

    // --- CORRECTION ESC : ON N'INITIALISE PAS LES MOTEURS TOUT DE SUITE ---
    // Si on lance motors_init() ici, on envoie 1000us pendant les 30s de calibration.
    // Cela provoque la mise en sécurité de certains ESC.
    
    // À la place, on force les pins à 0V (LOW). Les ESC vont biper "Signal Lost", mais ne se verrouilleront pas.
    // NOTE : Vérifie que ces pins correspondent bien à tes defines dans config.h (souvent 12, 13, 14, 15 sur ESP32)
    pinMode(12, OUTPUT); digitalWrite(12, LOW);
    pinMode(13, OUTPUT); digitalWrite(13, LOW);
    pinMode(14, OUTPUT); digitalWrite(14, LOW);
    pinMode(15, OUTPUT); digitalWrite(15, LOW);

    radio_init();
    radio_start_task(); // Radio indépendante de la loop()

    // On ne lance PAS motors_write_direct(2000...) ici. On attend de connaître le mode.

    // 3. DEMARRAGE TÂCHE TELEMETRIE (WIFI)
    start_telemetry_task(&drone); 

    // On attend un signal valide. Pendant ce temps, les ESC bipent (Signal Lost) ou attendent.
    unsigned long wait_start = millis();
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        // Clignotement rapide
        if((millis() / 50) % 2) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        delay(5);
        if(millis() - wait_start > 15000) break; // Sécurité 15s
    }

    // 4. DECISION SELON LE STICK
    if(drone.channel_3 > 1900) {
        // ================= MODE CALIBRATION ESC =================
        // L'utilisateur veut calibrer : ON INITIALISE LES MOTEURS MAINTENANT
        motors_init(); 
        // On envoie direct le MAX throttle pour entrer en mode prog ESC
        motors_write_direct(2000, 2000, 2000, 2000);

        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();

    } else {
        // ================= MODE VOL NORMAL (SAFE) =================
        // On reste silencieux vers les moteurs (LOW) pour ne pas déclencher le timeout ESC
        
        drone.current_mode = MODE_SAFE;

        // ========== PHASE 1 : CALIBRATION GYRO/ACCEL ==========
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# PHASE 1 - CALIBRATION GYRO/ACCEL        #"));
        Serial.println(F("# >>> NE PAS BOUGER LE DRONE <<<          #"));
        Serial.println(F("############################################"));

        imu_init();            // Calibration MPU6050 (5s)
        alt_imu_init();        // Calibration AltIMU (5s)

        // Pause visuelle
        digitalWrite(PIN_LED, LOW);
        delay(1000);

        // ========== PHASE 2 : CALIBRATION MAGNETOMETRE ==========
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# PHASE 2 - CALIBRATION MAGNETOMETRE      #"));
        Serial.println(F("# >>> TOURNER LE DRONE DANS TOUS LES SENS #"));
        Serial.println(F("############################################"));

        alt_imu_calibrate_mag();  // C'est ici que ça bloquait 20s

        // Fin de calibration - LED fixe 1 seconde
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# CALIBRATION TERMINEE                    #"));
        Serial.println(F("############################################"));
        digitalWrite(PIN_LED, HIGH);
        delay(1000);
        digitalWrite(PIN_LED, LOW);

        // --- C'EST MAINTENANT QU'ON REVEILLE LES ESC ---
        // La calibration longue est finie, on peut envoyer le signal PWM 1000us
        Serial.println(F("INITIALISATION MOTEURS..."));
        motors_init(); 
        // Les ESC vont maintenant faire leur musique de démarrage "123" + Bips LiPo
        
        // Démarrage des tâches FreeRTOS
        imu_start_task();      
        alt_imu_start_task();  

        // Initialisation fusion yaw
        yaw_fusion_init();

        pid_init();
        pid_init_params(&drone);
    }

    // Initialisation des compteurs de diag
    drone.max_time_radio = 0;
    drone.max_time_imu = 0;
    drone.max_time_pid = 0;
    
    // --- IMU DEBUG: Afficher l'entête CSV si debug activé ---
#if SERIAL_IMU_DEBUG
    Serial.println(F("dt_us,gx_raw,gx_filt,gy_raw,gy_filt,gz_raw,gz_filt,roll,pitch,yaw,thr,mode,ok"));
#endif

    loop_timer = micros();
}

void loop() {
    unsigned long t_start = micros();

    // Lecture tension batterie (filtrée) - seulement 1 fois sur 25 (10Hz)
    static uint8_t bat_counter = 0;
    static float vbat_filter = 11.1f;
    if (++bat_counter >= 25) {
        bat_counter = 0;
        int raw = analogRead(PIN_BATTERY);
        float v_pin = (raw / 4095.0f) * 3.3f;
        float v_bat = v_pin * BAT_SCALE;
        vbat_filter = (vbat_filter * 0.95f) + (v_bat * 0.05f);
        drone.voltage_bat = vbat_filter;
    }

    radio_update(&drone);
    unsigned long t_radio = micros();

    // 2. Gestion LED Erreur
    if (error_code > 0) {
        if ((millis() % 50) < 25) digitalWrite(PIN_LED, HIGH);
        else digitalWrite(PIN_LED, LOW);
    } 
    else if (drone.current_mode == MODE_SAFE) {
        digitalWrite(PIN_LED, HIGH); // Fixe en SAFE
    }

    // --- IMU DEBUG: Affichage série throttlé (10-20 Hz max) ---
    // Fonctionne dans TOUS les modes (y compris calibration ESC) pour faciliter les tests
#if SERIAL_IMU_DEBUG
    {
        static uint32_t lastPrintMs = 0;
        const uint32_t periodMs = 1000 / SERIAL_IMU_DEBUG_HZ;
        uint32_t nowMs = millis();
        if (nowMs - lastPrintMs >= periodMs) {
            lastPrintMs += periodMs;
            // Rattrapage si on a pris du retard (évite burst)
            if (nowMs - lastPrintMs >= periodMs) lastPrintMs = nowMs;

#if SERIAL_IMU_DEBUG_ONLY_ARMED
            if (drone.current_mode == MODE_ARMED || drone.current_mode == MODE_FLYING) {
#endif
            // Format CSV: dt_us,gx_raw,gx_filt,gy_raw,gy_filt,gz_raw,gz_filt,roll,pitch,yaw,thr,mode,ok
            Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                (unsigned long)drone.imu_dt_us,
                drone.gyro_raw_roll, drone.gyro_roll_input,
                drone.gyro_raw_pitch, drone.gyro_pitch_input,
                drone.gyro_raw_yaw, drone.gyro_yaw_input,
                drone.angle_roll, drone.angle_pitch, drone.angle_yaw,
                drone.channel_3,
                (int)drone.current_mode,
                (int)drone.imu_ok);
#if SERIAL_IMU_DEBUG_ONLY_ARMED
            }
#endif
        }
    }
#endif

    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } else {
        // Au lieu de bloquer sur I2C ici:
        // imu_read(&drone);

        unsigned long t_imu_start = micros();
        imu_update(&drone);      // <-- snapshot non-bloquant
        alt_imu_update(&drone);  // <-- snapshot alt_imu non-bloquant

        // Fusion Yaw (gyro + magnétomètre) - réduit à 50Hz pour économiser du CPU
#if YAW_FUSION_ENABLED
        static uint8_t fusion_counter = 0;
        if (drone.current_mode == MODE_ARMED || drone.current_mode == MODE_FLYING) {
            if (++fusion_counter >= 5) {
                fusion_counter = 0;
                const float dt_s = LOOP_TIME_US * 5.0f * 1e-6f;  // 5 cycles = 20ms
                yaw_fusion_update(&drone, dt_s);
            }
        } else {
            fusion_counter = 0;
        }
#endif

        unsigned long t_imu = micros();
        (void)t_imu_start;   // durée "loop" IMU n'a plus de sens; drone.current_time_imu vient de la task

        switch(drone.current_mode) {
            // ---------------- MODE SAFE ----------------
            case MODE_SAFE:
                motors_stop();
                
                // ARMEMENTS : Gaz Bas + Yaw Gauche
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0; // Reset erreurs
                        pid_reset_integral();
                        drone.angle_pitch = 0;
                        drone.angle_roll = 0;
                        imu_request_reset();      // Reset IMU angles
#if YAW_FUSION_ENABLED
                        yaw_fusion_reset(&drone); // Reset fusion yaw avec magnétomètre
#endif
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE ARMED ----------------
            case MODE_ARMED:
                motors_stop(); 
                // Clignotement lent
                if ((millis() % 500) < 100) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);

                // Décollage (Gaz > 1040)
                if(drone.channel_3 > 1040) {
                    drone.current_mode = MODE_FLYING;
                    disarm_debounce_timer = 0;
                    angle_security_timer = 0;
                }
                
                // Désarmement (Gaz Bas + Yaw Droite)
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        arming_timer = 0;
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE FLYING ----------------
            case MODE_FLYING:
                digitalWrite(PIN_LED, LOW); 

                // --- INTELLIGENCE DE VOL ---
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // --- DESARMEMENT MANUEL D'URGENCE ---
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        motors_stop(); 
                        arming_timer = 0;
                     }
                } else { 
                    arming_timer = 0; 
                }

                // --- SECURITE RADIO / AUTOLANDING ---
                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    
                    if (millis() - disarm_debounce_timer > 60000) { 
                        drone.current_mode = MODE_ARMED; 
                        error_code = 2; // PERTE RADIO
                        disarm_debounce_timer = 0;
                    }
                } else {
                    disarm_debounce_timer = 0; 
                }
                break;

            case MODE_WEB_TEST:
                motors_write_direct(
                    drone.web_test_vals[1], drone.web_test_vals[2], 
                    drone.web_test_vals[3], drone.web_test_vals[4]
                );
                // Sortie du mode test si on touche aux gaz
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }

        // --- DIAGNOSTIC LAG (BOÎTE NOIRE) ---
        unsigned long t_end = micros();
        unsigned long dur_radio = t_radio - t_start;
        unsigned long dur_imu = t_imu - t_radio;
        unsigned long dur_pid_mix = t_end - t_imu;
        unsigned long total_loop = t_end - t_start;

        drone.loop_time = total_loop; 
        
        // Si on détecte un gros lag (> 6000us), on enregistre le coupable
        // MAIS on évite d'overwrite le code d'erreur 888888 (IMU CRASH)
        if (total_loop > 6000) {
            if(dur_radio > drone.max_time_radio) drone.max_time_radio = dur_radio;
            if(drone.max_time_imu != 888888 && dur_imu > drone.max_time_imu) drone.max_time_imu = dur_imu;
            if(dur_pid_mix > drone.max_time_pid) drone.max_time_pid = dur_pid_mix;
        }
    }

    // --- OPTIMISATION MULTITÂCHE ---
    // Gestion du Loop Time (250Hz = 4000µs)
    unsigned long time_now = micros();
    
    // Si on a fini les calculs tôt (qu'il reste du temps avant le prochain cycle)
    if (loop_timer + LOOP_TIME_US > time_now) {
        unsigned long remaining = (loop_timer + LOOP_TIME_US) - time_now;
        
        // Si on a plus de 1.5ms de marge, on "dort" 1ms pour laisser le WiFi travailler
        // C'est LA ligne qui sauve la télémétrie
        if (remaining > 1500) {
             vTaskDelay(1); // Libère le CPU pour ~1ms (WiFi, Télémétrie)
        }
        
        // On finit le reste du temps (quelques µs) en attente active pour la précision PID
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    
    loop_timer = micros();
}