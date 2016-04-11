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


// SAFETY CODE ----------------------------------------------------
void safeState(){
    valveSafe();
    flameSafe();
    magnetSafe();
}

void enableState(){
   valveEnable();
//    flameEnable(); 
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

void chompSetup() {
    // Come up safely
    safeState();
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
    // wdt_enable(WDTO_4S);
    // xbeeInit();
    Debug.begin(115200);
    Sbus.begin(100000);
    driveSetup();
    leddarWrapperInit();
    sensorSetup();
    pinMode(A3, OUTPUT);  // laser pointer
    attachRCInterrupts();
    Debug.println("STARTUP");
}

static int16_t previous_leddar_state = FAR_ZONE;
static int8_t previous_rc_bitfield = 0;
static uint16_t hammer_intensity = 0;
static uint32_t last_request_time = micros();
static uint32_t last_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
static bool targeting_enabled = false;
static int16_t steer_bias = 0; // positive turns right, negative turns left
int16_t target_x_after_leadtime;
int16_t target_y_after_leadtime;
static uint8_t loop_type;

// int16_t drive_command = 0;  // for correlating RC to actual velocity

void chompLoop() {
    if (millis() % 1000 == 0) {
        uint16_t angle;
        readAngle(&angle);
        Debug.print("millis\t");
        Debug.print(millis());
        Debug.print("\t");
        Debug.println(angle);
    }
    loop_type = 1;
    uint32_t start_time = micros();
    if (micros() - last_request_time > 1000000UL){
        last_request_time = micros();
        requestDetections();
        // Debug.write("Request\r\n");
    }
    if (bufferDetections()){
        loop_type |= 2;
        uint8_t detection_count = parseDetections();
        last_request_time = micros();
        LeddarState current_leddar_state = getState(detection_count, getDetections());
        switch (current_leddar_state){
            case FAR_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case ARM_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case HIT_ZONE:
                if (previous_leddar_state == ARM_ZONE) {
                    if (autofireEnabled(previous_rc_bitfield)){
                        // delay(200);
                        // digitalWrite(A3, HIGH);
                        fire( hammer_intensity );
                    }
                } else {
                    previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
                }
                break;
        }
        // sendLeddarTelem(getDetections(), detection_count, current_leddar_state);
        // steer_bias = pidSteer(detection_count, getDetections(), 600);   // 600 cm ~ 20 ft
        // targetPredict(left_drive_value, right_drive_value, detection_count, getDetections(), 200, &target_x_after_leadtime, &target_y_after_leadtime, &steer_bias);
        
        requestDetections();
        
        // below is for correlating RC to actual velocity
        // if (getTargetingEnable()) {
        //     printMiddleDistance(detection_count, getDetections());
        //     Debug.print(drive_command); Debug.print("\t");
        //     mpu.read_all();
        //     Debug.print(mpu.accel_data[1] * 9.8, 6); Debug.print("\t");
        //     Debug.println(micros() - last_imu_read);
        //     last_imu_read = micros();
        //     drive_command += 1;
        // } else {
        //     last_imu_read = micros();
        //     drive_command = 0;
        // }
    }

    if (bufferSbusData()){
        bool in_failsafe_state = parseSbus();
        if (!in_failsafe_state) {
            loop_type |= 4;
            wdt_reset();
            // React to RC state changes
            char current_rc_bitfield = getRcBitfield();
            char diff = previous_rc_bitfield ^ current_rc_bitfield;
            hammer_intensity = getHammerIntensity();
            if (diff) {
                // Flame on -> off
                if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                    flameEnd();
                }
                // Flame off -> on
                if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
                    flameStart();
                }
                // Manual hammer fire
                if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                    if (current_rc_bitfield & DANGER_CTRL_BIT){
                      no_angle_fire(hammer_intensity);
                    } else {
                      fire(hammer_intensity);
                      // gentleFire();  // use retract system to put hammer forward
                      // delay(200);
                      // digitalWrite(A3, HIGH);
                    }
                }
                if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
                  if (current_rc_bitfield & DANGER_CTRL_BIT){
                    no_angle_retract();
                  } else {
                    retract();
                    // digitalWrite(A3, LOW);
                  }
                }
                if( (diff & MAG_CTRL_BIT) && (current_rc_bitfield & MAG_CTRL_BIT)){
                    magOn();
                }
                if( (diff & MAG_CTRL_BIT) && !(current_rc_bitfield & MAG_CTRL_BIT)){
                    magOff();
                }
            }
            previous_rc_bitfield = current_rc_bitfield;
        }
    }
    
    left_drive_value = getLeftRc();
    right_drive_value = getRightRc();
    
    // ideally, this would only be called if there is fresh RC input. otherwise, it is ugly for drive prediction stuff
    // don't spam motor controllers -- only send drive command every 10 ms
    // drive always called now to log drive command history. only commands roboteqs if getTargetingEnable()
    if (newRc()) {
        drive(left_drive_value - steer_bias, right_drive_value - steer_bias, getTargetingEnable());
        // drive(-drive_command, drive_command, getTargetingEnable());
    }

   if (micros() - last_telem_time > 200000L){
      uint32_t loop_speed = micros() - start_time;
      int16_t pressure;
      bool pressure_read_ok = readMlhPressure(&pressure);
      uint16_t angle;
      bool angle_read_ok = readAngle(&angle);
      send_sensor_telem(loop_speed, pressure, angle);
      last_telem_time = micros();
    }
   xbeePushData();
}

void electricalCheckouts(){
  Debug.print("Clicking valves\r\n");
  int pulse_time = 3000;
  Debug.print("Enable\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(ENABLE_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(ENABLE_VALVE_DO, LOW);
  
  Debug.print("Retract\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(RETRACT_VALVE_DO, LOW);

  Debug.print("Vent\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(VENT_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(VENT_VALVE_DO, LOW);

  Debug.print("Throw\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(THROW_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(THROW_VALVE_DO, LOW);

  Debug.print("Clicking magnets\r\n");
  Debug.print("Magnet 1\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(MAG1_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(MAG1_DO, LOW);

  Debug.print("Magnet 2\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(MAG2_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(MAG2_DO, LOW);

  Debug.print("Clicking flamethrower\r\n");
  Debug.print("Igniter\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(IGNITER_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(IGNITER_DO, LOW);

  Debug.print("Propane valve\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(PROPANE_DO, HIGH);
  delay(1000);
  safeDigitalWrite(PROPANE_DO, LOW);
  
  Debug.print("All done\r\n");
  safeState();
}
