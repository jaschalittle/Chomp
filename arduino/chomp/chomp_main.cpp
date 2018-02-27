#include "Arduino.h"
#include "chomp_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h"
#include "pins.h"
#include "drive.h"
#include "weapons.h"
#include "utils.h"
#include "chump_targeting.h"
#include <avr/wdt.h>
#include "command.h"
#include "I2C.h"
#include "MPU6050.h"
#include "telem_message_stream.h"


// SAFETY CODE ----------------------------------------------------
void safeState(){
    valveSafe();
    flameSafe();
    magnetSafe();
}

void enableState(){
   valveEnable();
}

static volatile int WEAPONS_ENABLE_pwm_val = 500; // disabled
static volatile int WEAPONS_ENABLE_prev_time = 0;
#define WEAPONS_ENABLE_THRESHOLD 1500
void weaponsEnableFalling();
void weaponsEnableRising(){
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableFalling, FALLING );
    WEAPONS_ENABLE_prev_time = micros();
}

void weaponsEnableFalling(){
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
    WEAPONS_ENABLE_pwm_val = micros() - WEAPONS_ENABLE_prev_time;
    if ( WEAPONS_ENABLE_pwm_val < WEAPONS_ENABLE_THRESHOLD){
        safeState();
        g_enabled = false;
    } else {
        if (!g_enabled) {
            g_enabled = true;
            enableState();
        }  
    }
}
// -----------------------------------------------------------------
//TelemetryMessageStream telemetry_stream;
//
MPU6050 IMU;

void chompSetup() {
    // Come up safely
    safeState();
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
    wdt_enable(WDTO_4S);
    xbeeInit();
    Sbus.begin(100000);
    driveSetup();
    leddarWrapperInit();
    sensorSetup();
    attachRCInterrupts();
    debug_print("STARTUP");
    //I2c.scan(telemetry_stream);
    IMU.initialize();
    bool IMUcheck = IMU.testConnection();
    debug_print(String("IMU.initialize() = " + IMUcheck));
}

static int16_t previous_leddar_state = FAR_ZONE;
static uint16_t previous_rc_bitfield = 0;
static uint16_t hammer_intensity = 0;
static uint32_t last_request_time = micros();
static uint32_t last_telem_time = micros();
static uint32_t last_leddar_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
static int16_t steer_bias = 0; // positive turns right, negative turns left
static bool targeting_enabled = false;
static bool new_autodrive = false;
static bool reset_targeting = false;

extern uint16_t leddar_overrun;
extern uint16_t leddar_crc_error;
extern uint16_t sbus_overrun;

uint32_t telemetry_interval=50000L;
uint32_t leddar_telemetry_interval=100000L;

void chompLoop() {
    uint32_t start_time = micros();

    // If there has been no data from the LEDDAR for a while, ask again
    if (micros() - last_request_time > 100000UL){
        last_request_time = micros();
        requestDetections();
    }

    // Check if data is available from the LEDDAR
    if (bufferDetections()){

        // extract detections from LEDDAR packet
        uint8_t raw_detection_count = parseDetections();

        // request new detections
        last_request_time = micros();
        requestDetections();

        calculateMinimumDetections(raw_detection_count);
        const Detection (*minDetections)[LEDDAR_SEGMENTS] = NULL;
        getMinimumDetections(&minDetections);

        // check for detections in zones
        LeddarState current_leddar_state = getState(*minDetections, getRange());
        switch (current_leddar_state){
            case FAR_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case ARM_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case HIT_ZONE:
            case PREDICTIVE_HIT_ZONE:
                if (previous_leddar_state == ARM_ZONE) {
                    if (autofireEnabled(previous_rc_bitfield)){
                        fire( hammer_intensity, previous_rc_bitfield & FLAME_PULSE_BIT, previous_rc_bitfield & MAG_PULSE_BIT, true /*autofire*/ );
                    }
                } else {
                    previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
                }
                break;
        }

        // get targeting RC command. reset targeting if RC state changes.
        reset_targeting = targeting_enabled ^ getTargetingEnable();
        targeting_enabled = getTargetingEnable();

        // auto centering code
        pidSteer(*minDetections, 600, &steer_bias, reset_targeting);   // 600 cm ~ 20 ft
        new_autodrive = true;

        // Send subsampled leddar telem
        if (micros() - last_leddar_telem_time > leddar_telemetry_interval){
          bool success = sendLeddarTelem(*minDetections, raw_detection_count, current_leddar_state);
          if (success){
            last_leddar_telem_time = micros();
          }
        }
    }

    if (bufferSbusData()){
        bool in_failsafe_state = parseSbus();
        if (!in_failsafe_state) {
            wdt_reset();
            // React to RC state changes
            uint16_t current_rc_bitfield = getRcBitfield();
            uint16_t diff = previous_rc_bitfield ^ current_rc_bitfield;
            hammer_intensity = getHammerIntensity();
            if( !(current_rc_bitfield & FLAME_PULSE_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                flameSafe();
            }
            if( (diff & FLAME_PULSE_BIT) && (current_rc_bitfield & FLAME_PULSE_BIT) ){
                flameEnable();
            }
            // Flame on -> off
            if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                flameEnd();
            }
            // Flame off -> on
            if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
                flameEnable();
                flameStart();
            }
            // Manual hammer fire
            if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                if (current_rc_bitfield & DANGER_CTRL_BIT){
                  noAngleFire(hammer_intensity, current_rc_bitfield & FLAME_PULSE_BIT, current_rc_bitfield & MAG_PULSE_BIT);
                } else {
                  fire(hammer_intensity, current_rc_bitfield & FLAME_PULSE_BIT, current_rc_bitfield & MAG_PULSE_BIT, false /*autofire*/);
                }
            }
            if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
              if (current_rc_bitfield & DANGER_CTRL_BIT){
                gentleRetract(HAMMER_RETRACT_BIT);
              } else {
                retract();
              }
            }
            if( (current_rc_bitfield & GENTLE_HAM_F_BIT)) {
                gentleFire(GENTLE_HAM_F_BIT);
            }
            if( (current_rc_bitfield & GENTLE_HAM_R_BIT)) {
                gentleRetract(GENTLE_HAM_R_BIT);
            }
            if( (diff & MAG_CTRL_BIT) && (current_rc_bitfield & MAG_CTRL_BIT)){
                magOn();
            }
            if( (diff & MAG_CTRL_BIT) && !(current_rc_bitfield & MAG_CTRL_BIT)){
                magOff();
            }
            previous_rc_bitfield = current_rc_bitfield;
        }
    }

    // always sent in telemetry
    left_drive_value = getLeftRc();
    right_drive_value = getRightRc();
    bool new_rc = newRc();
    if(new_autodrive || new_rc) {
        if(getTargetingEnable()) {
            left_drive_value -= steer_bias;
            right_drive_value -= steer_bias;
            // values passed by reference to capture clamping
            drive(left_drive_value, right_drive_value);
            updateDriveHistory(left_drive_value, right_drive_value);
        } else if(new_rc) {
            updateDriveHistory(left_drive_value, right_drive_value);
        }
        new_autodrive = false;
   }

  int16_t acceleration[3], angular_rate[3], magnetic_field[3];
  IMU.getMotion9(&acceleration[0], &acceleration[1], &acceleration[2],
                 &angular_rate[0], &angular_rate[1], &angular_rate[2],
                 &magnetic_field[0], &magnetic_field[1], &magnetic_field[2]);
  int16_t temperature = IMU.getTemperature();
  if (micros() - last_telem_time > telemetry_interval){
      uint32_t loop_speed = micros() - start_time;
      int16_t pressure = 0;
      readMlhPressure(&pressure);
      uint16_t angle = 0;
      readAngle(&angle);
      sendSensorTelem(pressure, angle);
      sendSystemTelem(loop_speed,
                      leddar_overrun,
                      leddar_crc_error,
                      sbus_overrun,
                      last_command,
                      command_overrun,
                      invalid_command);
      sendSbusTelem(previous_rc_bitfield);
      sendPWMTelem(left_drive_value, right_drive_value);
      sendIMUTelem(acceleration, angular_rate, magnetic_field, temperature);
      last_telem_time = micros();
  }

  handle_commands();
}
