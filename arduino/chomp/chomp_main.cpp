#include "Arduino.h"
#include "chomp_main.h"
#include "rc_pwm.h"
#include "sbus.h"
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
#include "autofire.h"

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


static uint32_t last_request_time = micros();
static uint32_t last_sensor_time = micros();
static int16_t steer_bias = 0; // positive turns left, negative turns right
static int16_t drive_bias = 0;
static bool new_autodrive = false;
static enum AutofireState autofire = AF_NO_TARGET;

extern uint16_t leddar_overrun;
extern uint16_t leddar_crc_error;
extern uint16_t sbus_overrun;
extern uint8_t HAMMER_INTENSITIES_ANGLE[9];

uint32_t sensor_period=5000L;
uint32_t leddar_max_request_period=100000L;

uint32_t manual_self_right_retract_start = 0;
uint32_t manual_self_right_retract_duration = 1000000;
uint32_t manual_self_right_dead_duration = 250000;

// parameters written in command
Track tracked_object;

void chompSetup() {
    // Come up safely
    safeState();
    wdt_enable(WDTO_4S);
    xbeeInit();
    rcInit();
    SBusInit();
    driveSetup();
    leddarWrapperInit();
    sensorSetup();
    initializeIMU();
    reset_loop_stats();
    restoreDriveControlParameters();
    restoreObjectSegmentationParameters();
    tracked_object.restoreTrackingFilterParams();
    restoreAutofireParameters();
    restoreSelfRightParameters();
    restoreTelemetryParameters();
    debug_print("STARTUP");
    start_time = micros();
}

void chompLoop() {
    if(micros() - last_sensor_time>sensor_period) {
        readSensors();
        last_sensor_time = micros();
    }

    // check for data from weapons radio
    bool working = sbusGood();
    if (working) {
        wdt_reset();
    }
    uint16_t current_rc_bitfield = getRcBitfield();

    // If there has been no data from the LEDDAR for a while, ask again
    if (micros() - last_request_time > leddar_max_request_period){
        last_request_time = micros();
        requestDetections();
    }

    int16_t drive_range = getDriveDistance();
    int16_t hammer_intensity = getHammerIntensity();
    int16_t hammer_distance = getRange();
    bool targeting_enabled = getTargetingEnable();
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


        // auto centering code
        new_autodrive = pidSteer(tracked_object, 
                                 drive_range, &drive_bias, &steer_bias);


        autofire = willHit(tracked_object, hammer_distance, hammer_intensity);
        if((autofire==AF_HIT) && (current_rc_bitfield & AUTO_HAMMER_ENABLE_BIT)) {
            fire(hammer_intensity, current_rc_bitfield & FLAME_PULSE_BIT, true);
        }

        // Send subsampled leddar telem
        if (isTimeToSendLeddarTelem(micros())){
            sendLeddarTelem(*minDetections, raw_detection_count);
        }
    }

    // React to RC state changes (change since last time this call was made)
    int16_t diff = getRcBitfieldChanges();
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
        selfRightOff();
        selfRightExtendLeft();
        manual_self_right_retract_start = 0;
    }
    if( (diff & MANUAL_SELF_RIGHT_RIGHT_BIT) && (current_rc_bitfield & MANUAL_SELF_RIGHT_RIGHT_BIT)){
        selfRightOff();
        selfRightExtendRight();
        manual_self_right_retract_start = 0;
    }
    if( (diff & (MANUAL_SELF_RIGHT_LEFT_BIT|MANUAL_SELF_RIGHT_RIGHT_BIT)) &&
       !(current_rc_bitfield & (MANUAL_SELF_RIGHT_LEFT_BIT|MANUAL_SELF_RIGHT_RIGHT_BIT))){
        selfRightOff();
        manual_self_right_retract_start = micros() | 1;
    }
    if(manual_self_right_retract_start > 0)
    {
        if(micros() - manual_self_right_retract_start > manual_self_right_retract_duration)
        {
            selfRightOff();
            manual_self_right_retract_start = 0;
        }
        else if(micros() - manual_self_right_retract_start > manual_self_right_dead_duration)
        {
            selfRightRetractBoth();
        }
    }

    // always sent in telemetry, cache values here
    int16_t left_drive_value = getLeftRc();
    int16_t right_drive_value = getRightRc();
    // newRc is destructive, make sure to only call it once
    bool new_rc = newRc();
    // check for autodrive
    if(new_autodrive || new_rc) {
        if(targeting_enabled) {
            left_drive_value -= steer_bias - drive_bias;
            right_drive_value -= steer_bias + drive_bias;
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
    if (isTimeToSendTelemetry(now)) {
        // get targeting RC command.
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
        int16_t hammer_angle = HAMMER_INTENSITIES_ANGLE[hammer_intensity];
        sendSbusTelem(current_rc_bitfield, hammer_angle, hammer_distance);
        sendPWMTelem(targeting_enabled, left_drive_value, right_drive_value,
                     drive_range);
        telemetryIMU();
        telemetrySelfRight();
    }
    if(isTimeToSendDriveTelemetry(now)) {
        driveTelem();
    }


    handle_commands();
    update_loop_stats();
}
