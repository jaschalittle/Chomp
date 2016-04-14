#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <math.h>

#define SEGMENTS_TO_RAD 0.049087385f
#define RAD_TO_SEGMENTS 20.371832f
#define SEGMENTS_TO_DEGREES 2.8125f
#define DEGREES_TO_SEGMENTS 0.3555556f


#define MIN_OBJECT_DISTANCE 25
#define MIN_OBJECT_SIZE 30
#define EDGE_CALL_THRESHOLD 90
#define MATCH_THRESHOLD 40  // will be exceeded by a 20 m/s object
#define NO_OBS_THRESHOLD 100  // 2 seconds with 50 Hz Leddar data
// Consider adding requirement that near objects must cover multiple segments
static Track tracked_object;
static uint32_t last_leddar_time = micros();
void trackObject(uint8_t num_detections, Detection* detections) {
    
    // only keep and analyze nearest detection in each segment
    Detection min_detections[16];
    for (uint8_t i = 0; i < num_detections; i++) {
        if (detections[i].Distance < min_detections[detections[i].Segment].Distance && detections[i].Distance > MIN_OBJECT_DISTANCE) {
            min_detections[detections[i].Segment] = detections[i];
        }
    }
    
    // call all objects in frame by detecting edges
    int16_t last_seg_distance = min_detections[0].Distance;
    int16_t min_obj_distance = min_detections[0].Distance;
    uint8_t right_edge = -1.0;
    uint8_t left_edge = 0.0;
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
                if (size > MIN_OBJECT_SIZE) {
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
    
    // call object after loop if no matching right edge seen for a left edge-- end of loop can be a right edge
    if (left_edge > right_edge) {
        right_edge = 16.0;
        size = sin(SEGMENTS_TO_RAD * (float) (right_edge - left_edge) / 2) * min_obj_distance * 2;
        if (size > MIN_OBJECT_SIZE) {
            objects[num_objects].Distance = min_obj_distance;
            objects[num_objects].Left_edge = left_edge;
            objects[num_objects].Right_edge = 16;
            objects[num_objects].Angle = ((float) (left_edge + right_edge) / 2 - 8) * SEGMENTS_TO_RAD;
            objects[num_objects].Size = size;
            num_objects++;
        }
    }
    
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
            // if new detection doesn't agree well with previous, ignore and wait for next reading
            // allow this to happen NO_OBS_THRESHOLD times before going for best match anyway
            // CONSIDER GOING FOR NEAREST OBJECT IF NO_OBS_THRESHOLD EXCEEDED
            if (min_diff_from_last < MATCH_THRESHOLD && tracked_object.Num_no_obs < NO_OBS_THRESHOLD) { 
                tracked_object.countNoObs(micros() - last_leddar_time);
            }
            else {
                tracked_object.update(best_match, micros() - last_leddar_time);
            }
        } else {
            // if there is an obj with two edges, select the nearest one to track
            Object nearest_obj = objects[0];
            bool two_edge_obj_present = objects[0].Left_edge != 0 && objects[0].Right_edge != 16;
            for (uint8_t i = 1; i < num_objects; i++) {
                if (objects[i].Size > MIN_OBJECT_SIZE && objects[i].Distance < nearest_obj.Distance) {
                    if (objects[i].Left_edge != 0 && objects[i].Right_edge != 16) { 
                        nearest_obj = objects[i];
                        two_edge_obj_present = true; 
                    }
                }
            }
            // if there is no obj with two edges, select nearest single edge obj to track
            nearest_obj = objects[0];
            if (!two_edge_obj_present) {
                for (uint8_t i = 1; i < num_objects; i++) {
                    if (objects[i].Size > MIN_OBJECT_SIZE && objects[i].Distance < nearest_obj.Distance) {
                        nearest_obj = objects[i];
                        two_edge_obj_present = true; 
                    }
                }
            }
        }
    } else {
        // if no objects called and we have been tracking something, increment Num_no_obs
        if (tracked_object.Num_obs > 0) {
            tracked_object.countNoObs(micros() - last_leddar_time);
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
    // Debug.print(tracked_object.Distance);
    // Serial.print("\t");
    // Serial.print(tracked_object.Right_edge);
    // Serial.print("\t");
    // Serial.print("Nearest/");
    // Serial.print(tracked_object.Distance);
    // Serial.print("\t");
    // Serial.print(SEGMENTS_TO_RAD, 5);
    // Serial.print("\t");
    // Serial.println();
    // return tracked_object;
    
    // Debug.println();
    // DEBUG PRINTS
}

// 8 Leddar segments is 0.436332 rad
// #define P_COEFF 2000  // coeff for radians, should correspond to 100 for segments. seems okay for Chump, too fast for Chomp
#define P_COEFF 1000
#define ERROR_DELTA_BUFFER_LENGTH 5
float error_delta_buffer[ERROR_DELTA_BUFFER_LENGTH];
static float leddar_delta_ts[ERROR_DELTA_BUFFER_LENGTH];
static uint8_t error_buffer_index = 0;
float target_angular_vel = 0.0;
float last_angle = 0.0;

int16_t pidSteer (unsigned int num_detections, Detection* detections, uint16_t threshold, int16_t *steer_bias, bool reset) {
    if (reset) {
        Debug.println("reset");
        tracked_object.reset();
    }
    trackObject(num_detections, detections);
    *steer_bias = P_COEFF * tracked_object.Angle;
    
    // code below is for trying to add feed forward term
    // int16_t feed_forward = 0;
    // float error_from_lead = tracked_object.Angle;
    // calculate feed_forward term if vel more than 5.7 deg/s
    // float our_angular_vel = getZgyroBuffered();
    // tracked_object.estimateVelocity(our_angular_vel);
    // if (abs(tracked_object.Angular_velocity) > 0.1 && tracked_object.Num_obs >= TRACK_BUFFER_LENGTH) {
    //     feed_forward = (tracked_object.Angular_velocity > 0) ? -322.1380f * tracked_object.Angular_velocity + 126.8426f : -322.1380f * tracked_object.Angular_velocity - 126.8426f;
    //     error_from_lead = tracked_object.Angle + tracked_object.Angular_velocity * 0.2;  // angle at 200 ms
    // }
    // *steer_bias = feed_forward + P_COEFF * error_from_lead;
    // Debug.print(tracked_object.Angle); Debug.print(" "); 
    // Debug.print(tracked_object.Size); Debug.print(" "); 
    // Debug.print(our_angular_vel); Debug.print(" "); 
    // Debug.println(*steer_bias);
}
