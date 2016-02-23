// Setup and output functions for PWM output pins. 
#include "Arduino.h"
#include "leddar_wrapper.h"
#include "targeting.h"

bool cmp_dist( Detection Detection1, Detection Detection2 ) {
	// turn off interrupts for atomic register operation
	return Detection1.Distance < Detection2.Distance;
}

Detection get_min_detection (detections) {
	num_detections = sizeof detections / sizeof detections[0];
	min_detection = detections[0];
	for ( unsigned int i = 1; i < num_detections; i++){
		if (detections[i].Distance < min_detection.Distance)
			min_detection = detections[i];
	}
	return min_detection;
}
