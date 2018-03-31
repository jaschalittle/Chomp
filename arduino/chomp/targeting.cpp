#include "Arduino.h"
#include "leddar_io.h"
#include "targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <stdlib.h>
#include <math.h>
#include "imu.h"
#include "telem.h"
#include "utils.h"


static uint8_t segmentObjects(const Detection (&min_detections)[LEDDAR_SEGMENTS],
                    Object (&objects)[8]);

static int8_t selectObject(const Object (&objects)[8], uint8_t num_objects,
                           const struct Track &tracked_object,
                           int32_t *selected_distance);

// object sizes are in mm
static int32_t min_object_size = 200;
static int32_t max_object_size = 1800;   // mm of circumferential size
static int32_t edge_call_threshold = 60; // cm for edge in leddar returns

void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS],
                 struct Track& tracked_object) {

    int16_t omegaZ = 0;
    getOmegaZ(&omegaZ);

    Object objects[8];
    uint8_t num_objects = segmentObjects(min_detections, objects);

    if(num_objects>0) {
        int32_t best_distance;
        uint8_t best_object = selectObject(objects, num_objects, tracked_object, &best_distance);
        uint32_t now = objects[best_object].Time;
        
        if(tracked_object.wants_update(now, best_distance)) {
           tracked_object.update(objects[best_object], omegaZ);
        } else {
           tracked_object.updateNoObs(now, omegaZ);
        }
        sendTrackingTelemetry(objects[best_object].xcoord(),
                              objects[best_object].ycoord(),
                              tracked_object.x/16,
                              tracked_object.vx/16,
                              tracked_object.y/16,
                              tracked_object.vy/16,
                              tracked_object.rx,
                              tracked_object.ry,
                              best_distance);
    // below is called if no objects called in current Leddar return
    } else {
        tracked_object.updateNoObs(micros(), omegaZ);
        sendTrackingTelemetry(0,
                              0,
                              tracked_object.x/16,
                              tracked_object.vx/16,
                              tracked_object.y/16,
                              tracked_object.vy/16,
                              0,
                              0,
                              0);
    }
}


static uint8_t segmentObjects(const Detection (&min_detections)[LEDDAR_SEGMENTS],
                    Object (&objects)[8]) {
    // call all objects in frame by detecting edges
    int16_t last_seg_distance = min_detections[0].Distance;
    int16_t min_obj_distance = min_detections[0].Distance;
    int16_t max_obj_distance = min_detections[0].Distance;
    int16_t right_edge = 0;
    int16_t left_edge = 0;
    uint8_t num_objects = 0;
    uint32_t now = micros();
    // this currently will not call a more distant object obscured by a nearer object, even if both edges of more distant object are visible
    for (uint8_t i = 1; i < 16; i++) {
        int16_t delta = (int16_t) min_detections[i].Distance - last_seg_distance;
        if (delta < -edge_call_threshold) {
            left_edge = i;
            min_obj_distance = min_detections[i].Distance;
            max_obj_distance = min_detections[i].Distance;
            objects[num_objects].SumDistance = 0;
        } else if (delta > edge_call_threshold) {
            // call object if there is an unmatched left edge
            if (left_edge >= right_edge) {
                right_edge = i;
                objects[num_objects].MinDistance = min_obj_distance;
                objects[num_objects].MaxDistance = max_obj_distance;
                objects[num_objects].LeftEdge = left_edge;
                objects[num_objects].RightEdge = right_edge;
                objects[num_objects].Time = now;
                int16_t size = objects[num_objects].size();
                if(size>min_object_size && size<max_object_size) {
                    num_objects++;
                }
            }
        }
        min_obj_distance = min(min_obj_distance, min_detections[i].Distance);
        max_obj_distance = max(max_obj_distance, min_detections[i].Distance);
        objects[num_objects].SumDistance += min_detections[i].Distance;
        last_seg_distance = min_detections[i].Distance;
    }

    // call object after loop if no matching right edge seen for a left edge-- end of loop can be a right edge. do not call entire FOV an object
    if ((left_edge > 0 && left_edge > right_edge) ||
        (left_edge == 0 && right_edge == 0)){
        right_edge = LEDDAR_SEGMENTS;
        objects[num_objects].MinDistance = min_obj_distance;
        objects[num_objects].MaxDistance = max_obj_distance;
        objects[num_objects].LeftEdge = left_edge;
        objects[num_objects].RightEdge = right_edge;
        objects[num_objects].Time = now;
        int16_t size = objects[num_objects].size();
        if(size>min_object_size && size<max_object_size) {
            num_objects++;
        }
    }

    return num_objects;
}

static int8_t selectObject(const Object (&objects)[8], uint8_t num_objects,
                           const struct Track &tracked_object,
                           int32_t *selected_distance) {
    int8_t best_match = 0;
    int32_t best_distance;
    uint32_t now = objects[best_match].Time;
    if(tracked_object.valid(now)) {
        best_distance = tracked_object.distanceSq(objects[best_match]);
    } else {
        best_distance = objects[best_match].radius();
        best_distance *= best_distance;
    }
    for (uint8_t i = 1; i < num_objects; i++) {
        int32_t distance;
        if(tracked_object.valid(now)) {
            distance = tracked_object.distanceSq(objects[i]);
        } else {
            distance = objects[i].radius();
            distance *= distance;
        }
        if (distance < best_distance) {
            best_distance = distance;
            best_match = i;
        }
    }
    *selected_distance = best_distance;
    return best_match;
}

void setObjectSegmentationParams(int16_t p_min_object_size,
                                 int16_t p_max_object_size,
                                 int16_t p_edge_call_threshold){
    min_object_size    = p_min_object_size;
    max_object_size    = p_max_object_size;
    edge_call_threshold= p_edge_call_threshold;
}


