#include "Arduino.h"
#include "chump_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "chump_targeting.h"
#include "sensors.h"
#include "xbee.h"
#include "drive.h"
#include "telem.h"
#include "pins.h"


void printMiddleDistance(unsigned int num_detections, Detection* detections) {
    int segment7_dist = 10000;
    int segment8_dist = 10000;
    for (uint8_t i = 0; i < num_detections; i++) {
        if (detections[i].Segment == 7 && detections[i].Distance < segment7_dist) {
            segment7_dist = detections[i].Distance;
        } else if (detections[i].Segment == 8 && detections[i].Distance < segment8_dist) {
            segment8_dist = detections[i].Distance;
        }
    }
    if (segment7_dist == 10000 && segment8_dist == 10000) {
        Xbee.print("n/a");
        Xbee.print("\t");
    } else if (segment7_dist == 10000) {
        Xbee.print(segment8_dist);
        Xbee.print("\t");
    } else if (segment8_dist == 10000) {
        Xbee.print(segment7_dist);
        Xbee.print("\t");
    } else {
        Xbee.print((segment7_dist + segment8_dist) / 2);
        Xbee.print("\t");
    }
}


void chumpSetup() {
    // canSetup();
    leddarWrapperInit();
    attachRCInterrupts();
    requestDetections();
    // Debug.begin(115200);
    Sbus.begin(100000);
    Sbus.setTimeout(10);
    DriveSerial.begin(115200);
    xbeeInit();
}

static char previous_rc_bitfield = 0;
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
bool targeting_enabled = getTargetingEnable() > 0.09;
static unsigned long last_request_time = micros();
static int16_t steer_bias = 0; // positive turns right, negative turns left
unsigned long last_loop_begin = micros();

void chumpLoop() {
//  Debug.println(micros() - last_loop_begin);
//  last_loop_begin = micros();

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000) {
    last_request_time = micros();
    requestDetections();
  }
  
  // check if there is new Leddar data
  bool complete = bufferDetections();
  if (complete) {
    unsigned int detection_count = parseDetections();
    last_request_time = micros();
    int current_leddar_state = getState(detection_count, getDetections());
    requestDetections();
    steer_bias = pidSteer(detection_count, getDetections(), 600);   // 600 cm ~ 20 ft
    
    // code for correlating distance to RC input
    // if (targeting_enabled && left_drive_value <= 600) {
        // left_drive_value += 50;
        // right_drive_value += 50;
        // printMiddleDistance(detection_count, getDetections());
        // Debug.print(left_drive_value);
        // Debug.print("\t");
        // Debug.print(right_drive_value);
        // Debug.println();
    // }
  }

  // should this be renamed to weapons_rc for clarity?
//   bool rc_complete = bufferSbusData();
//   if (rc_complete) {
//     parseSbus();
//   }

  left_drive_value = getLeftRc();
  right_drive_value = getRightRc();
  
  targeting_enabled = getTargetingEnable() > 0.09;
  if (targeting_enabled) {
    drive(left_drive_value - steer_bias, right_drive_value - steer_bias);
    // Debug.print(steer_bias);
    // Debug.println();
  }
}
