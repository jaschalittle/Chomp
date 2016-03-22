#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"


static uint8_t const error_history_length = 4;
static uint8_t error_history_index = 0;
static float errors[error_history_length] = {};
static float const finite_diff_coeffs[4] = {-1.0/3, 3.0/2, -3.0, 11.0/16};
// static float const finite_diff_coeffs[5] = {1.0/4, -4.0/3, 3.0, -4.0, 25.0/12};
// static float const finite_diff_coeffs[6] = {-1.0/5, 5.0/4, -10.0/3, 5.0, -5.0, 137.0/60};
// static float const finite_diff_coeffs[7] = {1.0/6, -6.0/5, 15.0/4, -20.0/3, 15.0/2, -6.0, 49.0/20};


float errorDerivative () {
  // central finite difference, sixth order accuracy. omit central point in input
  // requires 
  // float first_deriv = -(1.0/60) * errors[0] + (3.0/20) * errors[1] - (3.0/4) * errors[2] + (3.0/4) * errors[3] - (3.0/20) * errors[4] + (1.0/60) * errors[5];

  // backward finite difference approximation, third order accuracy
  float first_deriv = 0.0;
  uint8_t coeff_index = 0;
  for (uint8_t i = error_history_index; i < error_history_length; i++) {
      first_deriv += finite_diff_coeffs[coeff_index] * errors[i];
      coeff_index++;
  }
  for (uint8_t i = 0; i < error_history_index; i++) {
      first_deriv += finite_diff_coeffs[coeff_index] * errors[i];
      coeff_index++;
  }
//   float first_deriv = -(1.0/3) * errors[0] + (3.0/2) * errors[1] - 3 * errors[2] + (11.0/16) * errors[3];
  // backward finite difference approximation, fourth order accuracy
  // float first_deriv = (1.0/4) * errors[0] - (4.0/3) * errors[1] + 3 * errors[2] - 4 * errors[1] + (25.0/12) * errors[4];
  return first_deriv;
}


float errorIntegral () {
  float error_sum = 0.0;
  for (uint8_t i = 0; i < error_history_length; i++) {
    error_sum += errors[i];
  }
  return error_sum;
}


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
    Xbee.println(angle);
}


Object callNearestObj (uint8_t num_detections, Detection* detections) {
    Detection min_detections[16];
    for (uint8_t i = 0; i < num_detections; i++) {
        // flip segment numbers around here for later code legibility-- remember that Leddar is upside down
        uint8_t segment = 15 - detections[i].Segment;
        if (detections[i].Distance < min_detections[segment].Distance) {
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
        } else if (delta > 30) {
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

static int16_t p_coeff = 100;
static int16_t i_coeff = 0;
static int16_t d_coeff = 0;
int16_t pidSteer (unsigned int num_detections, Detection* detections, uint16_t threshold) {
    Object nearest_obj = callNearestObj(num_detections, detections);
    if (nearest_obj.Distance < threshold) {
        float angle = ((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2.0 - 8.0;
        sendAngle((uint8_t) (angle + 8.0) / 16.0 * 255.0);
        // update error history with new value
        errors[error_history_index] = angle;
        error_history_index = (error_history_index + 1) % error_history_length;
        int16_t steer_bias = p_coeff * angle;
        // int16_t steer_bias = p_coeff * angle + d_coeff * errorDerivative();
        return steer_bias;
    } else {
        return 0;
    }
}
