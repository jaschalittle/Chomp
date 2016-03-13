#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"


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

Detection getMinDetection (unsigned int num_detections, Detection* detections) {
  Detection min_detection = detections[0];
  for (unsigned int i = 1; i < num_detections; i++){
    if (detections[i].Distance < min_detection.Distance) {
      min_detection = detections[i];
    }
  }
//  Serial.print(min_detection.Distance);
  return min_detection;
}

Object callNearestObj (unsigned int num_detections, Detection* detections) {
  unsigned int min_distance = detections[(num_detections - 1)].Distance;
  Detection min_detection = detections[(num_detections - 1)];
  float right_edge = 0.0;
  float left_edge = 0.0;
//   Serial.print(detections[(num_detections - 1)].Segment);
//   Serial.print("/");
//   Serial.print(detections[(num_detections - 1)].Distance);
//   Serial.print("\t");

  // loop is backwards because Leddar is mounted upside down!
  Object objects[8];
  unsigned int num_objects = 0;
  for (int i = num_detections - 2; i >= 0; i--) {
    int delta = detections[i].Distance - min_distance;
    if (delta < -30) {
      left_edge = 15 - (char) detections[i].Segment;
    //   Serial.print("LEFT\t");
    //   Serial.print(detections[i].Segment);
    //   Serial.print("/");
    //   Serial.print(detections[i].Distance);
    //   Serial.print("\t");
      min_distance = detections[i].Distance;
//      right_edge = (float) (16 - (int) detections[i].Segment);
//      Serial.print(detections[i].Segment);
//      Serial.print("/");
//      Serial.print(detections[i].Distance);
//      Serial.print("\t");
    } else if (delta > 30) {
      if (left_edge > right_edge) {
        // Serial.print("RIGHT\t");
        right_edge = 15 - (char) detections[i].Segment;
        objects[num_objects].Distance = min_distance;
        objects[num_objects].Left_edge = left_edge;
        objects[num_objects].Right_edge = right_edge;
        num_objects++;
        min_distance = detections[i].Distance;
      } else {
        min_distance = detections[i].Distance;
      }
    //   Serial.print(detections[i].Segment);
    //   Serial.print("/");
    //   Serial.print(detections[i].Distance);
    //   Serial.print("\t");
    } else {
      if (detections[i].Distance < min_distance) { min_distance = detections[i].Distance; }
    //   Serial.print(detections[i].Segment);
    //   Serial.print("/");
    //   Serial.print(detections[i].Distance);
    //   Serial.print("\t");
//      if (detections[i].Segment == 0) {
//        if (left_edge > right_edge) {
//          right_edge = 16.0;
//          objects[num_objects].Distance = min_distance;
//          objects[num_objects].Left_edge = left_edge;
//          objects[num_objects].Right_edge = right_edge;
//          num_objects++;
//        }
//      }
    }
  }
  if (left_edge > right_edge) {
    right_edge = 16.0;
    objects[num_objects].Distance = min_distance;
    objects[num_objects].Left_edge = left_edge;
    objects[num_objects].Right_edge = right_edge;
    num_objects++;
  }

  Object nearest_obj = objects[0];
  for (int i = 1; i < num_objects; i++) {
    if (objects[i].Distance < nearest_obj.Distance) {
      nearest_obj = objects[i];
    }
  }
//  nearest_obj.Angle = (left_edge + right_edge) / 2 - 8;
//  Serial.print(nearest_obj.Left_edge);
//  Serial.print("\t");
//  Serial.print(nearest_obj.Right_edge);
//  Serial.print("\t");
//   Serial.print("Nearest/");
//   Serial.print(nearest_obj.Distance);
//   Serial.print("/");
//   Serial.print(((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2 - 8);
//   Serial.print("\t");
//  Serial.println();
  return nearest_obj;
}

static float p_term = 0.0005;
static float i_term = 0.0005;
static float d_term = 0.0005;
float pidSteer (unsigned int num_detections, Detection* detections) {
  Object nearest_obj = callNearestObj(num_detections, detections);

  float angle = ((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2 - 8;

  // update error history with new value
  errors[error_history_index] = angle;
  error_history_index = (error_history_index + 1) % error_history_length;

  float steer_bias = p_term * angle;
//   float steer_bias = p_term * angle + d_term * errorDerivative();
//   Debug.print(steer_bias);
//   Debug.print(" ");
//   Debug.print(angle);
//   Debug.println();
  return steer_bias;
}
