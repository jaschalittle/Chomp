static const int leddar_looptime = 20;
static const float dtC = float(leddar_looptime) / 1000.0;
static const float tau = 0.2;  // time constant-- when model prediction in balance with sensor data
static const float a = tau / (tau + dtC);
static const float a_comp = 1 - a;
static const float drive_coeff = 1.0;  // drive_coeff * roboteq drive command = cm/s
static const uint16_t chomp_width = 36;  // in cm
static const float max_vel = 1.5;  // cm/ms
static const float max_accel = 1.0;

static bool filter_initialized = false;
static uint32_t last_pred_time;
static int16_t pred_target_x;
static int16_t pred_target_y;
static float est_target_x_vel;
static float est_target_y_vel;

#define ERROR_HISTORY_LENGTH 5
static int16_t x_errors[ERROR_HISTORY_LENGTH];
static uint8_t x_error_index = 0;
static int16_t y_errors[ERROR_HISTORY_LENGTH];
static uint8_t y_error_index = 0;
static float avg_delta_t = 20.0;
float complementaryFilter(int16_t drive_left, int16_t drive_right, uint8_t num_detections, Detection* detections, uint16_t leadtime, 
                          int16_t* target_x_after_leadtime, int16_t* target_y_after_leadtime) {
	Object obs_object = callNearestObj(num_detections, detections);
    float obs_target_angle = (((float) obs_object.Left_edge + (float) obs_object.Right_edge) / 2.0 - 8.0) * PI / 64 ;
    uint16_t obs_target_distance = obs_object.Distance;
    int16_t obs_target_x = obs_target_distance * sin(obs_target_angle);
	int16_t obs_target_y = obs_target_distance * cos(obs_target_angle);
    // obs_target_y = sqrt(pow(obs_target_distance, 2) - pow(x ** 2));  // alternative to trig
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
		float delta_t = (micros() - last_pred_time) / 1000;  // delta_t in ms
        avg_delta_t = avg_delta_t * 0.9 + delta_t * 0.1;
		// first, compare new measurement to old prediction
		
		// pred_target_x = est_target_x_vel * delta_t + pred_target_x;  // predicted x pos at this time step
		// pred_target_y = est_target_y_vel * delta_t + pred_target_y;  // predicted y pos at this time step

		// MAYBE REMOVE THIS weighted average update of current position
		// pred_target_x = a * pred_target_x + a_comp * obs_target_x;
		// pred_target_y = a * pred_target_y + a_comp * obs_target_y;

		// estimate velocity given latest data
		// at max, assume all error is attributable to enemy acceleration, at min, assume it is due to sensor error
		int16_t x_error = obs_target_x - pred_target_x;
        int16_t y_error = obs_target_y - pred_target_y;
        int16_t xy_error = sqrt(pow(x_error, 2) + pow(y_error, 2));
        
        // if change is ridiculous, reset estimates and go to new object
        if (xy_error > 200) {
            est_target_x_vel = 0;
            est_target_y_vel = 0;
            pred_target_x = obs_target_x;
            pred_target_y = obs_target_y;
            for (uint8_t i = 0; i < ERROR_HISTORY_LENGTH; i++) {
                x_errors[i] = 0;
                y_errors[i] = 0;
            }
            return P_COEFF * obs_target_angle;
        }
        
        x_errors[x_error_index] = x_error;
        x_error_index = (x_error_index + 1) % ERROR_HISTORY_LENGTH;
        y_errors[y_error_index] = y_error;
        y_error_index = (y_error_index + 1) % ERROR_HISTORY_LENGTH;
        int16_t x_error_sum = 0;
		int16_t y_error_sum = 0;
        for (uint8_t i = 0; i < ERROR_HISTORY_LENGTH; i++) {
            x_error_sum += x_errors[i];
            y_error_sum += y_errors[i];
        }
        float new_x_vel = (float) x_error_sum / ERROR_HISTORY_LENGTH / delta_t;
		float new_y_vel = (float) y_error_sum / ERROR_HISTORY_LENGTH / delta_t;
        
		// float new_x_vel = est_target_x_vel + x_error / delta_t;
		// float new_y_vel = est_target_y_vel + y_error / delta_t;
        // float new_x_vel = x_error / delta_t;
		// float new_y_vel = y_error / delta_t;
		// if (abs(new_x_vel) > max_vel) {
		// 	new_x_vel = new_x_vel > 0 ? max_vel : max_vel * -1;
		// }
		// if (abs(new_y_vel) > max_vel) {
		// 	new_y_vel = new_y_vel > 0 ? max_vel : max_vel * -1;
		// }
        
		// first term is using model to calc new velocity, second is distrusting model and trusting object call from leddar data
		// est_target_x_vel = a * est_target_x_vel + a_comp * new_x_vel;  
		// est_target_y_vel = a * est_target_y_vel + a_comp * new_y_vel;
        float accel_x = new_x_vel - est_target_x_vel;
        float accel_y = new_y_vel - est_target_y_vel;
        
        // if accel is ridiculous, set to new leddar reading and zero out velocity? or require multiple readings to shake?
        
        est_target_x_vel += accel_x;  
		est_target_y_vel += accel_y;
		
		// calc expected next position. need to get angle from us turning in here
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
        // int16_t our_new_x = 0;
		// int16_t our_new_y = 0;
        
        // update this with our position
        pred_target_x = obs_target_x;
        pred_target_y = obs_target_y;
		
		// predict x and y after leadtime
		*target_x_after_leadtime = pred_target_x + est_target_x_vel * 200 - our_new_x;
		*target_y_after_leadtime = pred_target_y + est_target_y_vel * 200 - our_new_y;
        // *target_angle_after_leadtime = atan2(target_x_after_leadtime, target_y_after_leadtime);
    
        last_pred_time = micros();
        
        Debug.print(obs_target_x);
        Debug.print("\t");
        Debug.print(obs_target_y);
        Debug.print("\t");
        // Debug.print(pred_target_x);
        // Debug.print("\t");
        // Debug.print(pred_target_y);
        // Debug.print("\t");
        Debug.print(*target_x_after_leadtime);
        Debug.print("\t");
        Debug.print(*target_y_after_leadtime);
        Debug.print("\t");
        // Debug.print(est_target_x_vel);
        // Debug.print("\t");
        // Debug.print(est_target_y_vel);
        
        Debug.print(est_target_x_vel);
        Debug.print("\t");
        Debug.print(est_target_y_vel);
        Debug.print("\t");
        // Debug.print(x_error);
        // Debug.print("\t");
        // Debug.print(y_error);
        // Debug.print("\t");
        
        // Debug.println();
        
        float predicted_target_angle = atan2(*target_x_after_leadtime, *target_y_after_leadtime) * 64 / PI;
        Debug.print(avg_delta_t);
        Debug.print("\t");
        Debug.println(predicted_target_angle);
        return predicted_target_angle;
	}	
}
