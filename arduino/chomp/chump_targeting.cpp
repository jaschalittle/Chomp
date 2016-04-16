#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <math.h>


// #define P_COEFF 2000  // coeff for radians, should correspond to 100 for segments. seems okay for Chump, too fast for Chomp
#define P_COEFF 1800

// 8 Leddar segments is 0.436332 rad
#define SEGMENTS_TO_RAD 0.049087385f
#define RAD_TO_SEGMENTS 20.371832f
#define SEGMENTS_TO_DEGREES 2.8125f
#define DEGREES_TO_SEGMENTS 0.3555556f

#define MAX_FOLLOW_DISTANCE 90
#define MIN_OBJECT_SIZE 20
#define MAX_OBJECT_SIZE 150
#define EDGE_CALL_THRESHOLD 60
#define MATCH_THRESHOLD 60
#define NO_OBS_THRESHOLD 25  // 0.5 seconds with 50 Hz Leddar data
// Consider adding requirement that near objects must cover multiple segments
static Track tracked_object;
static uint32_t last_leddar_time = micros();
void trackObject(uint8_t num_detections, Detection* detections, uint16_t distance_threshold) {
    
    // only keep and analyze nearest detection in each segment
    Detection min_detections[16];
    getMinDetections(num_detections, detections, min_detections);
    
    // call all objects in frame by detecting edges
    int16_t last_seg_distance = min_detections[0].Distance;
    int16_t min_obj_distance = min_detections[0].Distance;
    int8_t right_edge = -1;
    int8_t left_edge = 0;
    Object objects[8];
    uint16_t size;
    uint8_t num_objects = 0;
    // this currently will not call a more distant object obscured by a nearer object, even if both edges of more distant object are visible
    for (uint8_t i = 1; i < 16; i++) {
        int16_t delta = (int16_t) min_detections[i].Distance - last_seg_distance;
        if (delta < -EDGE_CALL_THRESHOLD) {
            // Xbee.print("LEFT\t");
            // Debug.print(min_detections[i].Distance); Debug.print("\t");
            left_edge = i;
            min_obj_distance = min_detections[i].Distance;
            last_seg_distance = min_detections[i].Distance;
        } else if (delta > EDGE_CALL_THRESHOLD) {
            // call object if there is an unmatched left edge
            if (left_edge > right_edge) {
                // Xbee.print("RIGHT\t");
                // Debug.print(min_detections[i].Distance); Debug.print("\t");
                right_edge = i;
                size = sin(SEGMENTS_TO_RAD * (float) (right_edge - left_edge) / 2) * min_obj_distance * 2;
                // no size check if left edge is FOV edge-- that means we can't see both edges
                if ((size > MIN_OBJECT_SIZE || left_edge == 0) && size < MAX_OBJECT_SIZE && min_obj_distance < distance_threshold) {
                    objects[num_objects].Distance = min_obj_distance;
                    objects[num_objects].Left_edge = left_edge;
                    objects[num_objects].Right_edge = right_edge;
                    objects[num_objects].Angle = ((float) (left_edge + right_edge) / 2 - 8) * SEGMENTS_TO_RAD;
                    objects[num_objects].Size = size;
                    num_objects++;
                }
                last_seg_distance = min_detections[i].Distance;
            } else {
                last_seg_distance = min_detections[i].Distance;
            }
        } else {
            // Debug.print(min_detections[i].Distance); Debug.print("\t");
            // if there is an unmatched left edge, update min_obj_distance
            if (left_edge > right_edge && min_detections[i].Distance < min_obj_distance) { min_obj_distance = min_detections[i].Distance; }
            last_seg_distance = min_detections[i].Distance;
        }
    }
    
    // call object after loop if no matching right edge seen for a left edge-- end of loop can be a right edge. do not call entire FOV an object
    if ((left_edge != 0 && left_edge > right_edge) && min_obj_distance < distance_threshold) {
        right_edge = 16.0;
        size = sin(SEGMENTS_TO_RAD * (float) (right_edge - left_edge) / 2) * min_obj_distance * 2;
        // no min size check here, since we can't see both edges
        if (size < MAX_OBJECT_SIZE) {
            objects[num_objects].Distance = min_obj_distance;
            objects[num_objects].Left_edge = left_edge;
            objects[num_objects].Right_edge = 16;
            objects[num_objects].Angle = ((float) (left_edge + right_edge) / 2 - 8) * SEGMENTS_TO_RAD;
            objects[num_objects].Size = size;
            num_objects++;
        }
    }
    
#ifdef HARD_WIRED
    // DEBUG PRINTS
    for (uint8_t i = 0; i < num_objects; i++) {
        Debug.print(objects[i].Distance); Debug.print(" ");
        Debug.print(objects[i].Left_edge); Debug.print(" ");
        Debug.print(objects[i].Right_edge); Debug.print(" ");
        Debug.print(objects[i].Angle); Debug.print(" ");
        Debug.print(objects[i].Size); Debug.print(" ");
    }
    Debug.println();
    // DEBUG PRINTS
#endif

    // if an object has been called, assign it to existing tracked object or to new one
    if (num_objects > 0) {
        // if we have been tracking something, pick new object closest to previous one
        if (tracked_object.Num_obs > 0) {
            Object best_match = objects[0];
            int16_t min_diff_from_last = abs((int16_t) tracked_object.Distance - (int16_t) objects[0].Distance);
            for (uint8_t i = 1; i < num_objects; i++) {
                uint16_t diff_from_last = abs((int16_t) tracked_object.Distance - (int16_t) objects[i].Distance);  // difference between radial distances
                uint16_t avg_distance = (tracked_object.Distance + objects[i].Distance) / 2;  // average radial distance of tracked_object and this object
                diff_from_last += sin(abs(tracked_object.Angle - objects[i].Angle) / 2) * avg_distance * 2;  // add estimate of lateral distance diff to radial distance diff
                if (diff_from_last < min_diff_from_last) {
                    min_diff_from_last = diff_from_last;
                    best_match = objects[i];
                }
            }
            if (min_diff_from_last < MATCH_THRESHOLD) {
                tracked_object.update(best_match, micros() - last_leddar_time);
            // if MATCH_THRESHOLD exceeded and object was close when lost, continue following it until Num_no_obs exceeded
            } else if (tracked_object.Num_no_obs < NO_OBS_THRESHOLD && tracked_object.Distance < MAX_FOLLOW_DISTANCE) { 
                tracked_object.countNoObs(micros() - last_leddar_time);
            // if object was farther than MAX_FOLLOW_DISTANCE or Num_no_obs exceeded, forget it and lock onto new object
            } else {
                tracked_object.reset();
                Object nearest_obj = objects[0];
                for (uint8_t i = 1; i < num_objects; i++) {
                    if (objects[i].Distance < nearest_obj.Distance) {
                        nearest_obj = objects[i];
                    }
                }
                tracked_object.update(nearest_obj, micros() - last_leddar_time);
            }
        // if we aren't tracking something, lock onto nearest object
        } else {
            Object nearest_obj = objects[0];
            // if there is an obj with two edges, select the nearest one to track
            // bool two_edge_obj_present = objects[0].Left_edge != 0 && objects[0].Right_edge != 16;
            // for (uint8_t i = 1; i < num_objects; i++) {
            //     if (objects[i].Distance < nearest_obj.Distance) {
            //         if (objects[i].Left_edge != 0 && objects[i].Right_edge != 16) { 
            //             nearest_obj = objects[i];
            //             two_edge_obj_present = true; 
            //         }
            //     }
            // }
            // if there is no obj with two edges, select nearest single edge obj to track
            // nearest_obj = objects[0];
            // if (!two_edge_obj_present) {
                for (uint8_t i = 1; i < num_objects; i++) {
                    if (objects[i].Distance < nearest_obj.Distance) {
                        nearest_obj = objects[i];
                    }
                }
            // }
            tracked_object.update(nearest_obj, micros() - last_leddar_time);
        }
    // below is called if no objects called in current Leddar return
    } else {
        // if no objects called and we have been tracking something, increment Num_no_obs
        if (tracked_object.Num_obs > 0 && tracked_object.Distance < MAX_FOLLOW_DISTANCE) {
            tracked_object.countNoObs(micros() - last_leddar_time);
        } else {
            tracked_object.reset();
        }
        // if no objects called and Num_no_obs exceeded, forget tracked object
        if (tracked_object.Num_no_obs >= NO_OBS_THRESHOLD) {
            tracked_object.reset();
        }
    }
    last_leddar_time = micros();
    
    // DEBUG PRINTS
    // for (uint8_t i = 0; i < 16; i++) {
    //     Debug.print(min_detections[i].Distance); Debug.print("\t");
    // }
    // for (uint8_t i = 0; i < num_objects; i++) {
    //     Debug.print(objects[i].Angle); Debug.print(" ");
    //     Debug.print(objects[i].Distance); Debug.print(" ");
    // }
    
    // Debug.print(tracked_object.Angle); Debug.print(" ");
    // Debug.print(tracked_object.Distance); Debug.print(" ");
    // Debug.print(tracked_object.Size);
    
    // Debug.println();
    // DEBUG PRINTS
}

#define ERROR_DELTA_BUFFER_LENGTH 5
float error_delta_buffer[ERROR_DELTA_BUFFER_LENGTH];
static float leddar_delta_ts[ERROR_DELTA_BUFFER_LENGTH];
static uint8_t error_buffer_index = 0;
float target_angular_vel = 0.0;
float last_angle = 0.0;

int16_t pidSteer (unsigned int num_detections, Detection* detections, uint16_t distance_threshold, int16_t *steer_bias, bool reset) {
    if (reset) {
        Debug.println("reset");
        tracked_object.reset();
    }
    trackObject(num_detections, detections, distance_threshold);
    *steer_bias = P_COEFF * tracked_object.Angle;
    
    // Debug.print(tracked_object.Angle); Debug.print(" "); 
    // Debug.print(tracked_object.Size); Debug.print(" "); 
    // Debug.print(our_angular_vel); Debug.print(" "); 
    // Debug.println(*steer_bias);
}
