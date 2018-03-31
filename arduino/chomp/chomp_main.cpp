#include "Arduino.h"
#include "chomp_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h"
#include "pins.h"
#include "drive.h"
#include "weapons.h"
#include "utils.h"
#include "targeting.h"
#include <avr/wdt.h>
#include "command.h"
#include "imu.h"
#include "selfright.h"
#include "autodrive.h"

uint32_t start_time, loop_speed_min, loop_speed_avg, loop_speed_max, loop_count;
void reset_loop_stats(void) {
    loop_count = loop_speed_max = loop_speed_avg = 0;
    loop_speed_min = (uint32_t)(-1L);
}


void update_loop_stats() {
    uint32_t loop_speed = micros() - start_time;
    start_time = micros();
    loop_speed_min = min(loop_speed, loop_speed_min);
    loop_speed_avg += loop_speed;
    loop_count += 1;
    loop_speed_max = max(loop_speed, loop_speed_max);
}


void chompSetup() {
    // Come up safely
    safeState();
    wdt_enable(WDTO_4S);
    xbeeInit();
    rcInit();
    driveSetup();
    leddarWrapperInit();
    sensorSetup();
    initializeIMU();
    reset_loop_stats();
    debug_print("STARTUP");
    start_time = micros();
}

static uint16_t hammer_intensity = 0;
static uint32_t last_request_time = micros();
static uint32_t last_telem_time = micros();
static uint32_t last_drive_telem_time = micros();
static uint32_t last_leddar_telem_time = micros();
static uint32_t last_sensor_time = micros();
static int16_t steer_bias = 0; // positive turns right, negative turns left
static int16_t drive_bias = 0;
static bool targeting_enabled = false;
static bool new_autodrive = false;

extern uint16_t leddar_overrun;
extern uint16_t leddar_crc_error;
extern uint16_t sbus_overrun;

uint32_t telemetry_interval=50000L;
uint32_t leddar_telemetry_interval=100000L;
uint32_t sensor_period=5000L;
uint32_t drive_telem_interval=20000L;
uint32_t leddar_max_request_period=100000L;

// parameters written in command
Track tracked_object;

void chompLoop() {
    if(micros() - last_sensor_time>sensor_period) {
        readSensors();
        last_sensor_time = micros();
    }

    // If there has been no data from the LEDDAR for a while, ask again
    if (micros() - last_request_time > leddar_max_request_period){
        last_request_time = micros();
        requestDetections();
    }

    int16_t drive_range = getDriveDistance();
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

        trackObject(*minDetections, tracked_object);

        // get targeting RC command. reset targeting if RC state changes.
        targeting_enabled = getTargetingEnable();

        // auto centering code
        new_autodrive = pidSteer(tracked_object, 
                                 drive_range, &drive_bias, &steer_bias);

        // Send subsampled leddar telem
        if (micros() - last_leddar_telem_time > leddar_telemetry_interval){
          bool success = sendLeddarTelem(*minDetections, raw_detection_count);
          if (success){
            last_leddar_telem_time = micros();
          }
        }
    }

    // check for data from weapons radio
    bool radio_working = processSbusData();
    uint16_t current_rc_bitfield = getRcBitfield();
    if (radio_working) {
        wdt_reset();
        // React to RC state changes (change since last time this call was made)
        int16_t diff = getRcBitfieldChanges();
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
              noAngleFire(hammer_intensity, current_rc_bitfield & FLAME_PULSE_BIT);
            } else {
              fire(hammer_intensity, current_rc_bitfield & FLAME_PULSE_BIT, false /*autofire*/);
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
        if( (diff & MANUAL_SELF_RIGHT_LEFT_BIT) && (current_rc_bitfield & MANUAL_SELF_RIGHT_LEFT_BIT)){
            selfRightLeft();
        }
        if( (diff & MANUAL_SELF_RIGHT_RIGHT_BIT) && (current_rc_bitfield & MANUAL_SELF_RIGHT_RIGHT_BIT)){
            selfRightRight();
        }
        if( (diff & (MANUAL_SELF_RIGHT_LEFT_BIT|MANUAL_SELF_RIGHT_RIGHT_BIT)) &&
           !(current_rc_bitfield & (MANUAL_SELF_RIGHT_LEFT_BIT|MANUAL_SELF_RIGHT_RIGHT_BIT))){
            selfRightOff();
        }
    }

    // always sent in telemetry, cache values here
    int16_t left_drive_value = getLeftRc();
    int16_t right_drive_value = getRightRc();
    // newRc is destructive, make sure to only call it once
    bool new_rc = newRc();
    // check for autodrive
    if(new_autodrive || new_rc) {
        if(getTargetingEnable()) {
            left_drive_value += steer_bias - drive_bias;
            right_drive_value += steer_bias + drive_bias;
            // values passed by reference to capture clamping
            drive(left_drive_value, right_drive_value);
        }
        new_autodrive = false;
    }


    // read IMU and compute orientation
    processIMU();


    // if enabled, make sure robot is right-side-up
    autoSelfRight(current_rc_bitfield & AUTO_SELF_RIGHT_BIT);


    // send telemetry
    uint32_t now = micros();
    if (now - last_telem_time > telemetry_interval){
        sendSensorTelem(getPressure(), getAngle());
        sendSystemTelem(loop_speed_min, loop_speed_avg/loop_count,
                        loop_speed_max, loop_count,
                        leddar_overrun,
                        leddar_crc_error,
                        sbus_overrun,
                        last_command,
                        command_overrun,
                        invalid_command,
                        valid_command);
        reset_loop_stats();
        sendSbusTelem(current_rc_bitfield);
        sendPWMTelem(targeting_enabled, left_drive_value, right_drive_value,
                     drive_range);
        telemetryIMU();
        telemetrySelfRight();
        last_telem_time = micros();
    }
    if(now-last_drive_telem_time > drive_telem_interval) {
        driveTelem();
        last_drive_telem_time = now;
    }




    handle_commands();
    update_loop_stats();
}
