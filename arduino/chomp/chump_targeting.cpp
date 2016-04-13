#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <math.h>


void sendLeddar (uint8_t segment, uint16_t distance) {
    // if (distance > 0x0FFF) distance = 0x0FFF;
    // Xbee.write((segment << 4) | (distance >> 4));
    // Xbee.write(distance & 0xFF);
    
    // Xbee.print(segment);
    // Xbee.print("/");
    // Xbee.print(distance);
    // Xbee.print("\t");
}


void sendAngle (uint8_t angle) {
    // Xbee.write(angle);
    // Xbee.write(0x00);
    // Xbee.write(0x00);
    // Xbee.println(angle);
}


#define MIN_OBJECT_DISTANCE 10
#define EDGE_MIN_DELTA 30
// Consider adding requirement that near objects must cover multiple segments
Object callNearestObj (uint8_t num_detections, Detection* detections) {
    Detection min_detections[16];
    for (uint8_t i = 0; i < num_detections; i++) {
        // flip segment numbers around here for later code legibility-- remember that Leddar is upside down
        uint8_t segment = 15 - detections[i].Segment;
        if (detections[i].Distance < min_detections[segment].Distance && detections[i].Distance > MIN_OBJECT_DISTANCE) {
            min_detections[segment] = detections[i];
        }
    }
    Detection min_detection = min_detections[0];
    uint16_t min_distance = min_detection.Distance;
    float right_edge = -1.0;
    float left_edge = 0.0;
    sendLeddar(0, min_detection.Distance);

    Object objects[8];
    uint8_t num_objects = 0;
    for (uint8_t i = 1; i < 16; i++) {
        int16_t delta = min_detections[i].Distance - min_distance;
        if (delta < -30) {
            // Xbee.print("LEFT\t");
            left_edge = i;
            min_distance = min_detections[i].Distance;
        } else if (delta > EDGE_MIN_DELTA) {
            if (left_edge > right_edge) {
                // Xbee.print("RIGHT\t");
                right_edge = i;
                objects[num_objects].Distance = min_distance;
                objects[num_objects].Left_edge = left_edge;
                objects[num_objects].Right_edge = right_edge;
                num_objects++;
                min_distance = min_detections[i].Distance;  // set min_distance to new background
            } else {
                min_distance = min_detections[i].Distance;  // set min_distance to new background
            }
        } else {
            if (min_detections[i].Distance < min_distance) min_distance = min_detections[i].Distance;
        }
        sendLeddar(i, min_detections[i].Distance);
    }
    
    // call object after loop if no matching right edge seen for a left edge-- end of loop can be a right edge
    if (left_edge > right_edge) {
        objects[num_objects].Distance = min_distance;
        objects[num_objects].Left_edge = left_edge;
        objects[num_objects].Right_edge = 16;
        num_objects++;
    }

    Object nearest_obj = objects[0];
    for (uint8_t i = 1; i < num_objects; i++) {
        if (objects[i].Distance < nearest_obj.Distance) nearest_obj = objects[i];
    }
    // nearest_obj.Angle = (left_edge + right_edge) / 2 - 8;
    // Serial.print(nearest_obj.Left_edge);
    // Serial.print("\t");
    // Serial.print(nearest_obj.Right_edge);
    // Serial.print("\t");
    // Serial.print("Nearest/");
    // Serial.print(nearest_obj.Distance);
    // Serial.print("/");
    // Serial.print(((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2 - 8);
    // Serial.print("\t");
    // Serial.println();
    return nearest_obj;
}

// 8 Leddar segments is 0.436332 rad
#define P_COEFF 100.0f
// #define P_COEFF 100
int16_t pidSteer (unsigned int num_detections, Detection* detections, uint16_t threshold) {
    Object nearest_obj = callNearestObj(num_detections, detections);
    if (nearest_obj.Distance < threshold) {
        float angle = ((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2.0 - 8.0;
        // sendAngle((uint8_t) (angle + 8.0) / 16.0 * 255.0);
        int16_t steer_bias = P_COEFF * angle;
        return steer_bias;
    } else {
        return 0;
    }
}


// static const float dtC = 1.0 / LEDDAR_FREQ;
// static const float tau = 0.2;  // time constant-- when model prediction in balance with sensor data
// static const float a = tau / (tau + dtC);
// static const float a_comp = 1 - a;
// static const float drive_coeff = 1.0;  // drive_coeff * roboteq drive command = cm/s. won't need this if IMU works
// static const uint16_t chomp_width = 36;  // in cm. won't need this if IMU works
// static const float max_vel = 1.5;  // cm/ms
// static const float max_accel = 1.0;

// static bool filter_initialized = false;
// static uint32_t last_leddar_time;
// static int16_t last_target_x;
// static int16_t last_target_y;
// static int16_t pred_target_x;
// static int16_t pred_target_y;
// static float est_target_x_vel;
// static float est_target_y_vel;
// static float est_target_vector_dir;
// static float est_target_vector_mag;

// #define LEDDAR_DELTA_HISTORY_LENGTH 5
// static int16_t x_deltas[LEDDAR_DELTA_HISTORY_LENGTH];
// static int16_t y_deltas[LEDDAR_DELTA_HISTORY_LENGTH];
// static float leddar_delta_ts[LEDDAR_DELTA_HISTORY_LENGTH];
// static uint8_t leddar_delta_index = 0;
// #define SEGMENTS_TO_RAD 0.049087385f
// #define RAD_TO_SEGMENTS 20.371832f

// void targetPredict(uint8_t num_detections, Detection* detections, uint16_t leadtime, 
//                    int16_t* target_x_after_leadtime, int16_t* target_y_after_leadtime, int16_t* steer_bias) {
//     Object obs_object = callNearestObj(num_detections, detections);
//     float obs_target_angle = (((float) obs_object.Left_edge + (float) obs_object.Right_edge) / 2.0 - 8.0) * SEGMENTS_TO_RAD;  // convert to radians
//     uint16_t obs_target_distance = obs_object.Distance;
//     int16_t obs_target_x = obs_target_distance * sin(obs_target_angle);
//     int16_t obs_target_y = obs_target_distance * cos(obs_target_angle);
//     Debug.print(obs_target_x); Debug.print("\t"); Debug.print(obs_target_y); Debug.print("\t");
    
//     // when filter starts up, initialize with nearest object and velocity estimates of 0
//     if (!filter_initialized) {
//         filter_initialized = true;
//         // est_target_x_vel = 0.0;  
//         // est_target_y_vel = 0.0;
//         est_target_vector_dir = 0.0;
//         est_target_vector_mag = 0.0;
//         // this initializes to object call. could maybe try initializing to something "neutral" if this seems problematic
//         last_target_x = obs_target_x;
//         last_target_y = obs_target_y;
//         last_leddar_time = micros();
        
//         // zero out IMU history and start tracking timing
//         resetImu();
        
//         *target_x_after_leadtime = obs_target_x;
//         *target_y_after_leadtime = obs_target_y;
//         *steer_bias = P_COEFF * obs_target_angle;
        
//         return;
//     } else {
//         // what we really want here is the leddar time stamp
//         float leddar_delta_t = (micros() - last_leddar_time) / 1000.0;
        
//         if (leddar_delta_t > 500.0) {
//             est_target_vector_mag = 0.0;
//             est_target_vector_dir = 0.0;
//             last_target_x = obs_target_x;
//             last_target_y = obs_target_y;
//             // for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
//             //     x_deltas[i] = 0;
//             //     y_deltas[i] = 0;
//             // }
//             *target_x_after_leadtime = obs_target_x;
//             *target_y_after_leadtime = obs_target_y;
//             *steer_bias = P_COEFF * obs_target_angle * RAD_TO_SEGMENTS;
//             return;
//         }
        
//         // infer our motion to transform robot frame
//         // IF THIS IS SKITTISH AT LONG DISTANCE, NEED TO FILTER GYRO DATA MORE?
//         // float our_forward_vel, our_angular_vel;
//         float our_angular_vel = getZgyroBuffered() / 1000;  // this gets average of 10 buffered vals. vals buffered during main loop
//         // readImu(&our_forward_vel, &our_angular_vel);  // looks like IMU returns pos angle if left rotation, neg if right
//         float turn_angle = our_angular_vel * leddar_delta_t;
//         int16_t our_drive_command = -1 * getAvgDriveCommand();  // neg command is forward
//         // little to no drive output below 150
//         float our_forward_vel = 0;
//         if (abs(our_drive_command) > 150) {
//             our_forward_vel = (our_drive_command > 0) ? 0.00008578077 * (our_drive_command - 150) : 0.00008578077 * (our_drive_command + 150);  // cm/s. pretty linear beyond 150
//         }
//         float turn_radius = (our_angular_vel != 0.0) ? (our_forward_vel * leddar_delta_t) / our_angular_vel : 0;
//         float our_new_x = (our_forward_vel > 0 && turn_angle > 0.02) ? turn_radius - cos(turn_angle) * turn_radius : 0;
//         float our_new_y = (turn_angle > 0.05) ? sin(turn_angle) * turn_radius : our_forward_vel * leddar_delta_t;  // need to figure out angle sign. I think this is right too
        
//         // int16_t our_new_x = our_x_vel * leddar_delta_t;
//         // int16_t our_new_y = our_y_vel * leddar_delta_t;
        
        
//         // translate estimated target velocity to our new robot frame
//         est_target_vector_dir -= turn_angle;
//         est_target_x_vel = sin(est_target_vector_dir) * est_target_vector_mag;
//         est_target_y_vel = cos(est_target_vector_dir) * est_target_vector_mag;
//         // predict target position now with translated velocity estimate 
//         int16_t pred_target_x = last_target_x - our_new_x + est_target_x_vel * leddar_delta_t;
//         int16_t pred_target_y = last_target_y - our_new_y + est_target_y_vel * leddar_delta_t;
//         // calculate error between Leddar reading and prediction
//         float x_error = obs_target_x - pred_target_x;
//         float x_delta = obs_target_x - (last_target_x - our_new_x);
//         float y_error = obs_target_y - pred_target_y;
//         float y_delta = obs_target_y - (last_target_y - our_new_y);
//         float xy_error = sqrt(pow(x_error, 2) + pow(y_error, 2));
//         float xy_delta = sqrt(pow(x_delta, 2) + pow(y_delta, 2));
//         // Debug.print(x_delta); Debug.print("\t"); Debug.print(y_delta); Debug.print("\t");
//         // Debug.print(pred_target_x); Debug.print("\t"); Debug.print(pred_target_y); Debug.print("\t");
//         Debug.print(est_target_x_vel); Debug.print("\t"); Debug.print(est_target_y_vel); Debug.print("\t");
//         // if implied acceleration is ridiculous, reset estimates and go to new object
//         // this threshold needs to be set properly. error accumulates over ~20 ms, so 20 cm * 50 Hz = 10 m/s/s near instantaneous acceleration
//         // TEST IF THIS ALONE IS LIKE P CONTROL LOOP
//         // SCALE THRESHOLD BY DISTANCE
//         if (xy_error > obs_target_distance * 0.05) {
//             est_target_vector_mag = 0.0;
//             est_target_vector_dir = 0.0;
//             last_target_x = obs_target_x;
//             last_target_y = obs_target_y;
//             // for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
//             //     x_deltas[i] = 0;
//             //     y_deltas[i] = 0;
//             // }
//             *target_x_after_leadtime = obs_target_x;
//             *target_y_after_leadtime = obs_target_y;
//             *steer_bias = P_COEFF * obs_target_angle * RAD_TO_SEGMENTS;
//             return;
//         }
        
//         // could do with acceleration on estimate as below, or average buffer
//         est_target_vector_mag = 0.9 * est_target_vector_mag + 0.1 * (xy_delta / leddar_delta_t);
//         est_target_vector_dir = 0.9 * est_target_vector_dir + 0.1 * atan2(x_delta, y_delta);
//         est_target_x_vel = sin(est_target_vector_dir) * est_target_vector_mag;
//         est_target_y_vel = cos(est_target_vector_dir) * est_target_vector_mag;
//         // Debug.print(est_target_vector_dir); Debug.print("\t"); Debug.println(est_target_vector_mag);
//         Debug.print(est_target_x_vel); Debug.print("\t"); Debug.print(est_target_y_vel); Debug.print("\t");
//         // buffered strategy doesn't work with changing robot frames
//         // average velocity over last LEDDAR_DELTA_HISTORY_LENGTH Leddar returns
//         // x_deltas[leddar_delta_index] = obs_target_x - last_target_x - our_new_x;
//         // y_deltas[leddar_delta_index] = obs_target_y - last_target_y - our_new_y;
//         // int16_t x_delta_sum = 0;
//         // int16_t y_delta_sum = 0;
//         // for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
//         //     x_delta_sum += x_deltas[i];
//         //     y_delta_sum += y_deltas[i];
//         // }
//         // leddar_delta_ts[leddar_delta_index] = leddar_delta_t;
//         // leddar_delta_index = (leddar_delta_index + 1) % LEDDAR_DELTA_HISTORY_LENGTH;
//         // float leddar_delta_t_sum = 0;
//         // for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
//         //     leddar_delta_t_sum += leddar_delta_ts[i];
//         // }
//         // float new_x_vel = (float) x_delta_sum / leddar_delta_t_sum;
//         // float new_y_vel = (float) y_delta_sum / leddar_delta_t_sum;
        
//         // est_target_vector_dir = atan2(new_x_vel, new_y_vel);
//         // est_target_vector_mag = sqrt(pow(new_x_vel, 2) + pow(new_y_vel, 2));
        
//         // radians per second
//         float target_angular_velocity = (est_target_x_vel * 1000) / obs_target_distance;
//         Debug.println(target_angular_velocity);
        
//         // base drive output is slew rate to match enemy estimated velocity
//         // THESE VALS ARE FOR CHUMP
//         int16_t base_slew_command = 0;
//         if (abs(target_angular_velocity) > 0.1) {
//             base_slew_command = (target_angular_velocity > 0) ? 322.1380f * target_angular_velocity + 126.8426f : 322.1380f * target_angular_velocity - 126.8426f;
//         }
        
//         // add prediction of our next pos when we get it running
        
//         // predict x and y after leadtime in current robot frame
//         turn_radius = (our_forward_vel * leadtime) / our_angular_vel;
//         turn_angle = our_angular_vel * leadtime;
//         our_new_x = turn_radius - cos(turn_angle) * turn_radius;  // need to confirm out angle sign. I think this is right if left turns have neg angle
//         our_new_y = sin(turn_angle) * turn_radius;  // need to figure out angle sign. I think this is right too
//         float lead_target_vector_dir = est_target_vector_dir - turn_angle;
//         float lead_target_x_vel = sin(est_target_vector_dir) * est_target_vector_mag;
//         float lead_target_y_vel = cos(est_target_vector_dir) * est_target_vector_mag;
//         *target_x_after_leadtime = obs_target_x - our_new_x + lead_target_x_vel * leadtime;
//         *target_y_after_leadtime = obs_target_y - our_new_y + lead_target_y_vel * leadtime;

//         // int16_t pred_target_distance = sqrt(pow(*target_x_after_leadtime, 2) + pow(*target_y_after_leadtime, 2));
        
//         // set for next cycle
//         last_target_x = obs_target_x;
//         last_target_y = obs_target_y;
//         // pred_target_x = obs_target_x + est_target_x_vel * 20 - our_new_x;
//         // pred_target_y = obs_target_y + est_target_y_vel * 20 - our_new_y;
        
//         int16_t lead_error = target_angular_velocity * leadtime;  // this is 0 when we are tracking properly
        
//         *steer_bias = base_slew_command + P_COEFF * lead_error;

//         last_leddar_time = micros();

//         // Debug.print(obs_target_x); Debug.print("\t");
//         // Debug.print(obs_target_y); Debug.print("\t");
//         // Debug.print(*target_x_after_leadtime); Debug.print("\t");
//         // Debug.print(*target_y_after_leadtime); Debug.print("\t");
//         // Debug.print(est_target_x_vel); Debug.print("\t");
//         // Debug.print(est_target_y_vel); Debug.print("\t");

        

//         // Debug.println(*steer_bias);
//         // Debug.println(atan((float) *target_x_after_leadtime / *target_y_after_leadtime));
//         // Debug.print(our_angular_vel); Debug.print("\t");
//         // Debug.print(our_forward_vel); Debug.print("\t");
        
//         // Debug.println();

//         return;
// 	}	
// }
