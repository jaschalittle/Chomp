#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <math.h>


// #define P_COEFF 2000  // coeff for radians, should correspond to 100 for segments. seems okay for Chump, too fast for Chomp
#define P_COEFF 1800
#define STEER_BIAS_CAP 500

// 8 Leddar segments is 0.436332 rad
#define SEGMENTS_TO_RAD 0.049087385f
#define RAD_TO_SEGMENTS 20.371832f
#define SEGMENTS_TO_DEGREES 2.8125f
#define DEGREES_TO_SEGMENTS 0.3555556f

#define MAX_FOLLOW_DISTANCE 90
#define MIN_OBJECT_SIZE 20
#define MAX_OBJECT_SIZE 180
#define EDGE_CALL_THRESHOLD 60
#define MATCH_THRESHOLD 60
#define NO_OBS_THRESHOLD 25  // 0.5 seconds with 50 Hz Leddar data
// Consider adding requirement that near objects must cover multiple segments
static Track tracked_object;
static uint32_t last_leddar_time = micros();
void trackObject(uint8_t num_detections, Detection* detections, int16_t distance_threshold) {
    
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
            left_edge = i;
            min_obj_distance = min_detections[i].Distance;
            last_seg_distance = min_detections[i].Distance;
        } else if (delta > EDGE_CALL_THRESHOLD) {
            // call object if there is an unmatched left edge
            if (left_edge > right_edge) {
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

    // if an object has been called, assign it to existing tracked object or to new one
    if (num_objects > 0) {
        // if we have been tracking something, pick new object closest to previous one
        if (tracked_object.Num_obs > 0) {
            Object best_match = objects[0];
            int16_t min_diff_from_last = abs((int16_t) tracked_object.Distance - (int16_t) objects[0].Distance);
            for (uint8_t i = 1; i < num_objects; i++) {
                int16_t diff_from_last = abs((int16_t) tracked_object.Distance - (int16_t) objects[i].Distance);  // difference between radial distances
                int16_t avg_distance = (tracked_object.Distance + objects[i].Distance) / 2;  // average radial distance of tracked_object and this object
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
            for (uint8_t i = 1; i < num_objects; i++) {
                if (objects[i].Distance < nearest_obj.Distance) {
                    nearest_obj = objects[i];
                }
            }
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
}

#define ERROR_DELTA_BUFFER_LENGTH 5
float error_delta_buffer[ERROR_DELTA_BUFFER_LENGTH];
float target_angular_vel = 0.0;
float last_angle = 0.0;

void pidSteer (unsigned int num_detections, Detection* detections, uint16_t distance_threshold, int16_t *steer_bias, bool reset) {
    if (reset) {
        tracked_object.reset();
    }
    trackObject(num_detections, detections, distance_threshold);
    int16_t calculated_steer_bias = P_COEFF * tracked_object.Angle;
    if (calculated_steer_bias > STEER_BIAS_CAP) { calculated_steer_bias = STEER_BIAS_CAP; }
    if (calculated_steer_bias < -STEER_BIAS_CAP) { calculated_steer_bias = -STEER_BIAS_CAP; }
    *steer_bias = calculated_steer_bias;
}

void Track::reset() {
    Distance = 0;
    Size = 0;
    Angle = 0.0;
    for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { 
        Distance_history[i] = 0;
        Angle_delta_history[i] = 0.0;
        Inter_leddar_times[i] = 0;
        Gyro_history[i] = 0.0;
    }
    Leddar_history_index = 0;
    Num_obs = 0;
    Num_no_obs = 0;
    Gyro_history_index = 0;
    Closing_velocity = 0.0;
    Angular_velocity = 0.0;
}

void Track::update(Object best_match, uint32_t inter_leddar_time) {
    Distance = best_match.Distance;
    Size = best_match.Size;
    Distance_history[Leddar_history_index] = best_match.Distance;
    Angle_delta_history[Leddar_history_index] = best_match.Angle - Angle;
    Angle = best_match.Angle;
    Inter_leddar_times[Leddar_history_index] = inter_leddar_time;
    Leddar_history_index = (Leddar_history_index + 1) % TRACK_BUFFER_LENGTH;
    Num_obs += 1;
    Num_no_obs = 0;
}

void Track::countNoObs(uint32_t inter_leddar_time) {
    Distance_history[Leddar_history_index] = Distance;
    Angle_delta_history[Leddar_history_index] = 0.0;
    Inter_leddar_times[Leddar_history_index] = inter_leddar_time;
    Leddar_history_index = (Leddar_history_index + 1) % TRACK_BUFFER_LENGTH;
    Num_no_obs += 1;
}

void Track::estimateVelocity(float our_angular_velocity) {
        // calculate total time of buffered data in ms
        uint32_t total_time = 100;  // five Leddar reads is 100 ms
        // for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { total_time += Inter_leddar_times[i]; Serial.print(Inter_leddar_times[i]); Serial.print(" ");}
        // total_time /= 1000; // in ms
        
        // calculate average enemy angular velocity in robot frame
        Angular_velocity = 0.0;
        for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) {
            Angular_velocity += Angle_delta_history[i];
        }
        Angular_velocity = Angular_velocity / (float) total_time * 1000;  // average radians per s
        
        // estimate how fast we've been spinning in buffered period
        Gyro_history[Gyro_history_index] = our_angular_velocity;
        Gyro_history_index = (Gyro_history_index + 1) % TRACK_BUFFER_LENGTH;
        our_angular_velocity = 0.0;
        for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { our_angular_velocity += Gyro_history[i]; }
        our_angular_velocity /= TRACK_BUFFER_LENGTH;  // average radians per sec
        
        // calculate average enemy angular velocity in world frame
        Angular_velocity = Angular_velocity - our_angular_velocity;
    }
