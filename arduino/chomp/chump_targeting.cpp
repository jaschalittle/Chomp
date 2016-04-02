#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"
#include "pins.h"
#include <math.h>


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
    // Xbee.println(angle);
}

// Consider adding requirement that near objects must cover multiple segments
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
static float setpoint = 0.0;
int16_t pidSteer (unsigned int num_detections, Detection* detections, uint16_t threshold) {
    Object nearest_obj = callNearestObj(num_detections, detections);
    if (nearest_obj.Distance < threshold) {
        float angle = ((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2.0 - 8.0;
        // sendAngle((uint8_t) (angle + 8.0) / 16.0 * 255.0);
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

static const int leddar_looptime = 20;
static const float dtC = float(leddar_looptime) / 1000.0;
static const float tau = 0.5;  // time constant-- when model prediction in balance with sensor data
static const float a = tau / (tau + dtC);
static const float a_comp = 1 - a;
static const float drive_coeff = 1.0;  // drive_coeff * roboteq drive command = cm/s
static const uint16_t chomp_width = 36;  // in cm
static const int16_t max_vel = 1500;  // cm/s

static bool filter_initialized = false;
static uint32_t last_pred_time;
static int16_t pred_target_x;
static int16_t pred_target_y;
static int16_t est_target_x_vel;
static int16_t est_target_y_vel;

float complementaryFilter(int16_t drive_left, int16_t drive_right, uint8_t num_detections, Detection* detections, uint16_t leadtime, 
                          int16_t* target_x_after_leadtime, int16_t* target_y_after_leadtime) {
	Object obs_object = callNearestObj(num_detections, detections);
    float obs_target_angle = ((float) obs_object.Left_edge + (float) obs_object.Right_edge) / 2.0 - 8.0 * PI / 64 ;
    uint16_t obs_target_distance = obs_object.Distance;
    int16_t obs_target_x = obs_target_distance * sin(obs_target_angle);
	int16_t obs_target_y = obs_target_distance * cos(obs_target_angle);
    if (!filter_initialized) {
		filter_initialized = true;
		est_target_x_vel = 0;  
		est_target_y_vel = 0;
        // this initializes to object call. could maybe try initializing to something "neutral" if this seems problematic
        pred_target_x = obs_target_x;
        pred_target_y = obs_target_y;
		last_pred_time = micros();
		return obs_target_angle;
	} else {
		uint32_t delta_t = (micros() - last_pred_time) / 1000;  // delta_t in ms
		// first, compare new measurement to old prediction
		
		// obs_target_y = sqrt(pow(obs_target_distance, 2) - pow(x ** 2));  // alternative to trig
		pred_target_x = est_target_x_vel * delta_t + pred_target_x;  // predicted x pos at this time step
		pred_target_y = est_target_y_vel * delta_t + pred_target_y;  // predicted y pos at this time step

		// weighted average update of current position
		pred_target_x = a * pred_target_x + a_comp * obs_target_x;
		pred_target_y = a * pred_target_y + a_comp * obs_target_y;

		// estimate velocity given latest data
		// at max, assume all error is attributable to enemy acceleration, at min, assume it is due to sensor error
		int16_t x_error = pred_target_x - obs_target_x;
		int16_t y_error = pred_target_y - obs_target_y;
		int16_t new_x_vel = est_target_x_vel + x_error / delta_t;
		int16_t new_y_vel = est_target_y_vel + y_error / delta_t;
		if (abs(new_x_vel) > max_vel) {
			new_x_vel = max_vel * ((new_x_vel > 0) - (new_x_vel < 0));
		}
		if (abs(new_y_vel) > max_vel) {
			new_y_vel = max_vel * ((new_y_vel > 0) - (new_y_vel < 0));
		}
        // 
		// first term is using model to calc new velocity, second is distrusting model and trusting object call from leddar data
		est_target_x_vel = a * (est_target_y_vel) + a_comp * (new_x_vel);  
		est_target_y_vel = a * (est_target_y_vel) + a_comp * (new_y_vel);
		
		// prediction = x_angleC + newRate * leadtime;

		// calc expected next position
		int16_t our_x_vel, our_y_vel, our_vel;
		int16_t drive_bias = drive_right - drive_left;
		our_vel = (drive_right - drive_left) / 2;  // neg RC on left is forward, pos RC on right is forward
		if (abs(drive_bias) < 10) {
			our_x_vel = 0;
			our_y_vel = our_vel;
		} else {
			int16_t turn_radius = -drive_left * chomp_width / (drive_bias);
			int16_t angle = turn_radius / our_vel;
			our_x_vel = turn_radius - cos(angle) * turn_radius;
			our_y_vel = sin(angle) * turn_radius;
		}
		int16_t our_new_x = our_x_vel * leadtime;
		int16_t our_new_y = our_y_vel * leadtime;
		
		// then, update x and y predictions given new velocity estimates
		*target_x_after_leadtime += pred_target_x + new_x_vel * leadtime - our_new_x;
		*target_y_after_leadtime += pred_target_y + new_y_vel * leadtime - our_new_y;
        // *target_angle_after_leadtime = atan2(target_x_after_leadtime, target_y_after_leadtime);
        
        last_pred_time = micros();

		Debug.print(obs_target_x);
		Debug.print("\t");
		Debug.print(obs_target_y);
		Debug.print("\t");
		Debug.print(pred_target_x);
		Debug.print("\t");
		Debug.print(pred_target_y);
		Debug.print("\t");
		Debug.print(*target_x_after_leadtime);
		Debug.print("\t");
		Debug.println(*target_y_after_leadtime);

        float predicted_target_angle = atan2(pred_target_x, pred_target_y);
		return predicted_target_angle;
	}	
}