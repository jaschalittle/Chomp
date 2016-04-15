#include <math.h>
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

static const int leddar_looptime = 20;
static const float dtC = float(leddar_looptime) / 1000.0;
static const float tau = 0.5;  // time constant-- when model prediction in balance with sensor data
static const float a = tau / (tau + dtC);
static const float 1_minus_a = 1 - a;
static const float drive_coeff = 1.0;  // drive_coeff * roboteq drive command = cm/s
static const uint16_t chomp_width = 36;  // in cm
static const int16_t max_vel = 1500;  // cm/s

static int x_next = 0.0;
static int y_next = 0.0;
static uint16_t target_d;
static float target_ang;
static bool filter_initialized = false;
static uint32_t last_pred_time;

float complementaryFilter(int16_t drive_left, int16_t drive_right, float obs_target_angle, uint16_t obs_target_distance, uint16_t leadtime) {
	if (!filter_initialized) {
		filter_initialized = true;
		est_target_x_vel = 0;  
		est_target_y_vel = 0;
		last_pred_time = micros();
		obs_target_angle = 
		return obs_target_angle;
	} else {
		float delta_t = (micros() - last_pred_time) / 1000.0;  // delta_t in ms
		// first, compare new measurement to old prediction
		obs_target_x = obs_target_distance * sin(obs_target_angle);
		obs_target_y = obs_target_distance * cos(obs_target_angle);
		// obs_target_y = sqrt(pow(obs_target_distance, 2) - pow(x ** 2));  // alternative to trig
		pred_target_x = est_target_x_vel * delta_t + pred_target_x;  // predicted x pos at this time step
		pred_target_y = est_target_y_vel * delta_t + pred_target_y;  // predicted y pos at this time step

		// weighted average update of current position
		pred_target_x = a * pred_target_x + 1_minus_a * obs_target_x;
		pred_target_y = a * pred_target_y + 1_minus_a * obs_target_y;

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
		// first term is using model to calc new velocity, second is distrusting model and trusting object call from leddar data
		est_target_x_vel = a * (est_target_y_vel) + 1_minus_a * (new_x_vel);  
		est_target_y_vel = a * (est_target_y_vel) + 1_minus_a * (new_y_vel);
		
		// prediction = x_angleC + newRate * leadtime;

		// calc expected next position
		int16_t our_x_vel, our_y_vel, our_vel;
		drive_bias = drive_right - drive_left;
		our_vel = (drive_right - drive_left) / 2;  // neg RC on left is forward, pos RC on right is forward
		if (abs(drive_bias) < 10) {
			our_x_vel = 0;
			our_y_vel = our_vel;
		} else {
			int16_t turn_radius = -drive_left * chomp_width / (drive_bias);
			angle = 2 * pi * turn_radius / our_vel;
			our_x_vel = turn_radius - cos(angle) * turn_radius;
			our_y_vel = sin(angle) * turn_radius;
			// need trig here our_x_vel = turn_radius * (-drive_left + drive_right) / 2;
		}
		our_new_x = our_x_vel * leadtime;
		our_new_y = our_y_vel * leadtime;
		
		// then, update x and y predictions given new velocity estimates
		target_x_after_leadtime += pred_target_x + new_x_vel * leadtime - our_new_x;
		target_y_after_leadtime += pred_target_y + new_y_vel * leadtime - our_new_y;

		Debug.print(leddar_x);
		Debug.print("\t");
		Debug.print(leddar_y);
		Debug.print("\t");
		Debug.print(pred_x);
		Debug.print("\t");
		Debug.print(pred_y);
		Debug.print("\t");
		Debug.print(lead_x);
		Debug.print("\t");
		Debug.println(lead_y);

		predict(&target_d, &target_ang);
		return predicted_target_angle;
	}
	
}

// add something to predict if in certain x, y hitbox at timestep + hammer lag