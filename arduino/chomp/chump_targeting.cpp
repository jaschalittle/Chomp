#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include "sensors.h"
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


#define P_COEFF 100
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


static const float dtC = 1.0 / LEDDAR_FREQ;
static const float tau = 0.2;  // time constant-- when model prediction in balance with sensor data
static const float a = tau / (tau + dtC);
static const float a_comp = 1 - a;
static const float drive_coeff = 1.0;  // drive_coeff * roboteq drive command = cm/s. won't need this if IMU works
static const uint16_t chomp_width = 36;  // in cm. won't need this if IMU works
static const float max_vel = 1.5;  // cm/ms
static const float max_accel = 1.0;

static bool filter_initialized = false;
static uint32_t last_leddar_time;
static int16_t last_target_x;
static int16_t last_target_y;
static float est_target_x_vel;
static float est_target_y_vel;

#define LEDDAR_DELTA_HISTORY_LENGTH 5
static int16_t x_deltas[LEDDAR_DELTA_HISTORY_LENGTH];
static int16_t y_deltas[LEDDAR_DELTA_HISTORY_LENGTH];
static float leddar_delta_ts[LEDDAR_DELTA_HISTORY_LENGTH];
static uint8_t leddar_delta_index = 0;

void targetPredict(int16_t drive_left, int16_t drive_right, uint8_t num_detections, Detection* detections, uint16_t leadtime, 
                   int16_t* target_x_after_leadtime, int16_t* target_y_after_leadtime, int16_t* steer_bias) {
    Object obs_object = callNearestObj(num_detections, detections);
    float obs_target_angle = (((float) obs_object.Left_edge + (float) obs_object.Right_edge) / 2.0 - 8.0) * PI / 64 ;
    uint16_t obs_target_distance = obs_object.Distance;
    int16_t obs_target_x = obs_target_distance * sin(obs_target_angle);
    int16_t obs_target_y = obs_target_distance * cos(obs_target_angle);
    
    // when filter starts up, initialize with nearest object and velocity estimates of 0
    if (!filter_initialized) {
        filter_initialized = true;
        est_target_x_vel = 0.0;  
        est_target_y_vel = 0.0;
        // this initializes to object call. could maybe try initializing to something "neutral" if this seems problematic
        last_target_x = obs_target_x;
        last_target_y = obs_target_y;
        last_leddar_time = micros();
        
        // zero out IMU history and start tracking timing
        resetImu();
        
        *target_x_after_leadtime = obs_target_x;
        *target_y_after_leadtime = obs_target_y;
        *steer_bias = P_COEFF * obs_target_angle * 64 / PI;
        
        return;
    } else {
        // what we really want here is the leddar time stamp
        float leddar_delta_t = (micros() - last_leddar_time) / 1000;
        
        int16_t pred_target_x = last_target_x + est_target_x_vel * leddar_delta_t;
        int16_t pred_target_y = last_target_y + est_target_y_vel * leddar_delta_t;

        // calculate error between Leddar reading and prediction
        int16_t x_error = obs_target_x - pred_target_x;
        int16_t y_error = obs_target_y - pred_target_y;
        int16_t xy_error = sqrt(pow(x_error, 2) + pow(y_error, 2));
        
        // if implied acceleration is ridiculous, reset estimates and go to new object
        // this threshold needs to be set properly. error is over ~20 ms, so 20 * 50 = 10 m/s/s
        if (xy_error > 20) {
            est_target_x_vel = 0;
            est_target_y_vel = 0;
            last_target_x = obs_target_x;
            last_target_y = obs_target_y;
            for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
                x_deltas[i] = 0;
                y_deltas[i] = 0;
            }
            *steer_bias = P_COEFF * obs_target_angle * 64 / PI;
            return;
        }
        
        // average velocity over last LEDDAR_DELTA_HISTORY_LENGTH Leddar returns
        x_deltas[leddar_delta_index] = obs_target_x - last_target_x;
        y_deltas[leddar_delta_index] = obs_target_y - last_target_y;
        int16_t x_delta_sum = 0;
        int16_t y_delta_sum = 0;
        for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
            x_delta_sum += x_deltas[i];
            y_delta_sum += y_deltas[i];
        }
        leddar_delta_ts[leddar_delta_index] = leddar_delta_t;
        leddar_delta_index = (leddar_delta_index + 1) % LEDDAR_DELTA_HISTORY_LENGTH;
        float leddar_delta_t_sum = 0;
        for (uint8_t i = 0; i < LEDDAR_DELTA_HISTORY_LENGTH; i++) {
            leddar_delta_t_sum += leddar_delta_ts[i];
        }
        float new_x_vel = (float) x_delta_sum / leddar_delta_t_sum;
        float new_y_vel = (float) y_delta_sum / leddar_delta_t_sum;
        
        est_target_x_vel = new_x_vel;
        est_target_y_vel = new_y_vel;

        // calc expected next position. need to get angle from us turning in here. or put in IMU read/processing
        float our_forward_vel, our_angular_vel;
        readImu(&our_forward_vel, &our_angular_vel);
        float turn_radius = our_forward_vel / our_angular_vel;
        float turn_angle = our_forward_vel / turn_radius;
        int16_t our_x_vel = turn_radius - cos(turn_angle) * turn_radius;  // need to figure out angle sign
        int16_t our_y_vel = sin(turn_angle) * turn_radius; // need to figure out angle sign
        int16_t our_new_x = our_x_vel * leadtime;
        int16_t our_new_y = our_y_vel * leadtime;
        
        // if IMU reading doesn't come back, output angle * P_TERM
        
        // until IMU set up, this will only work if we are stationary
            // int16_t our_new_x = 0;
            // int16_t our_new_y = 0;
        
        // add prediction of our next pos when we get it running
        last_target_x = obs_target_x;
        last_target_y = obs_target_y;
        
        // predict x and y after leadtime
        *target_x_after_leadtime = last_target_x + est_target_x_vel * leadtime - our_new_x;
        *target_y_after_leadtime = last_target_y + est_target_y_vel * leadtime - our_new_y;
        *steer_bias = P_COEFF * atan((float) *target_x_after_leadtime / *target_y_after_leadtime) * 64 / PI;

        last_leddar_time = micros();

        Debug.print(obs_target_x); Debug.print("\t");
        Debug.print(obs_target_y); Debug.print("\t");
        Debug.print(*target_x_after_leadtime); Debug.print("\t");
        Debug.print(*target_y_after_leadtime); Debug.print("\t");
        Debug.print(est_target_x_vel); Debug.print("\t");
        Debug.print(est_target_y_vel); Debug.print("\t");

        

        // Debug.println(*steer_bias);
        // Debug.println(atan((float) *target_x_after_leadtime / *target_y_after_leadtime));
        Debug.print(our_angular_vel); Debug.print("\t");
        Debug.print(our_forward_vel); Debug.print("\t");
        
        Debug.println();

        return;
	}	
}
