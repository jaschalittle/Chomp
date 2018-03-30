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


// #define P_COEFF 2000  // coeff for radians, should correspond to 100 for segments. seems okay for Chump, too fast for Chomp
#define P_COEFF 1800
#define STEER_BIAS_CAP 500

// object sizes are in mm
static int32_t min_object_size = 200;
static int32_t max_object_size = 1800;   // mm of circumferential size
static int32_t edge_call_threshold = 60; // cm for edge in leddar returns
static uint32_t min_num_updates = 3;
static uint32_t track_lost_dt = 500000;  // us
static int32_t max_off_track = 600L*600L; // squared distance in mm
static int32_t max_start_distance = 6000L*6000L; // squared distance in mm
static int32_t xtol = 200;
static int32_t ytol = 200;

struct Object
{
    uint16_t MinDistance, MaxDistance, SumDistance;
    int8_t LeftEdge, RightEdge;
    uint32_t Time;

    // Default constructor
    Object() : MinDistance(10000), MaxDistance(0), SumDistance(0),
               LeftEdge(0), RightEdge(0),
               Time(0) { }
    inline int16_t size(void) const;
    inline int16_t angle(void) const;
    inline int16_t xcoord(void) const;
    inline int16_t ycoord(void) const;
    inline int16_t radius(void) const;
    inline int32_t distanceSq(const Object &other) const;
    float originalDistance(const Object &other) const;
};

#define TRACK_BUFFER_LENGTH 16
struct Track
{
    int32_t x, vx, y, vy;
    int32_t rx, ry;
    uint32_t num_updates;
    uint32_t last_update, last_predict;
    int32_t last_omgaz;

    int16_t alpha, beta;

    // Default constructor
    Track() :
        x(0), vx(0),
        y(0), vy(0),
        num_updates(0),
        last_update(micros()),
        last_predict(micros()),
        alpha(10000), beta(16384)
        { }
    int32_t predict(uint32_t now, int16_t omegaZ);
    void update(const Object& best_match, int16_t omegaZ);
    int32_t distanceSq(const Object& obj) const;
    bool valid(uint32_t now) const;
    int16_t updateOmegaZ(int32_t dt, int16_t omegaZ);
    void updateNoObs(uint32_t inter_leddar_time, int16_t omegaZ);
    bool timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ) const;
    int16_t angle(void) const;
};

// Consider adding requirement that near objects must cover multiple segments
static Track tracked_object;
void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS]) {

    // call all objects in frame by detecting edges
    int16_t last_seg_distance = min_detections[0].Distance;
    int16_t min_obj_distance = min_detections[0].Distance;
    int16_t max_obj_distance = min_detections[0].Distance;
    int16_t right_edge = 0;
    int16_t left_edge = 0;
    Object objects[8];
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

    int16_t omegaZ = 0;
    getOmegaZ(&omegaZ);
    // if an object has been called, assign it to existing tracked object or to new one
    if (num_objects > 0) {
        int8_t best_match = 0;
        int32_t best_distance;
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
        if(( tracked_object.valid(now) && best_distance < max_off_track) ||
           (!tracked_object.valid(now) && best_distance < max_start_distance)) {
           tracked_object.update(objects[best_match], omegaZ);
        } else {
           tracked_object.updateNoObs(now, omegaZ);
        }
        sendTrackingTelemetry(objects[best_match].xcoord(),
                              objects[best_match].ycoord(),
                              tracked_object.x/16,
                              tracked_object.vx/16,
                              tracked_object.y/16,
                              tracked_object.vy/16,
                              tracked_object.rx,
                              tracked_object.ry,
                              best_distance);
    // below is called if no objects called in current Leddar return
    } else {
        tracked_object.updateNoObs(now, omegaZ);
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

void pidSteer (const Detection (&detections)[LEDDAR_SEGMENTS], int16_t *steer_bias) {
    trackObject(detections);
    int16_t calculated_steer_bias = P_COEFF * tracked_object.angle();
    if (calculated_steer_bias > STEER_BIAS_CAP) { calculated_steer_bias = STEER_BIAS_CAP; }
    if (calculated_steer_bias < -STEER_BIAS_CAP) { calculated_steer_bias = -STEER_BIAS_CAP; }
    *steer_bias = calculated_steer_bias;
}

bool timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ) {
    return tracked_object.timeToHit(dt, depth, omegaZ);
}

void setTrackingFilterParams(int16_t alpha, int16_t beta,
                             int16_t p_min_object_size,
                             int16_t p_max_object_size,
                             int16_t p_edge_call_threshold,
                             int8_t p_min_num_updates,
                             uint32_t p_track_lost_dt,
                             int16_t p_max_off_track,
                             int16_t p_max_start_distance,
                             int16_t p_xtol,
                             int16_t p_ytol
        ) {
    min_object_size    = p_min_object_size;
    max_object_size    = p_max_object_size;
    edge_call_threshold= p_edge_call_threshold;
    min_num_updates    = p_min_num_updates;
    track_lost_dt      = p_track_lost_dt;
    max_off_track      = p_max_off_track;
    max_start_distance = p_max_start_distance;
    xtol = p_xtol;
    ytol = p_ytol;
    tracked_object.alpha = alpha;
    tracked_object.beta = beta;
}

// size in mm
// r = average radius = (min+max)/2
// (pi/180)*LEDDAR_FOV/LEDDAR_SEGMENTS = (pi/180)*45/16 ~ 0.049
// theta = (left-right)*0.049
// circumferential size = theta*r = (left-right)*(min+max)*0.049/2
// leddar reports ranges in cm, so multiply this expression by 10
// and 0.49 ~ 0.5 gives
inline int16_t Object::size(void) const {
    return (RightEdge-LeftEdge)*(MaxDistance+MinDistance)/4;
}

// radius in mm
inline int16_t Object::radius(void) const {
    return SumDistance*10/(RightEdge - LeftEdge);
}

// angle in radians scaled by 2048
inline int16_t Object::angle(void) const {
    return (((LeftEdge + RightEdge) - 17)*50); // really )/2-8.5)*0.049
}


// x coordinate in mm
inline int16_t Object::xcoord(void) const {
    int32_t ma = angle();
    return (radius()*(2048L-((ma*ma)/4096L)))/2048L;
}

// y coordinate in mm
inline int16_t Object::ycoord(void) const {
    return ((int32_t)radius()*angle())/2048L;
}

// distance squared in mm
int32_t Object::distanceSq(const Object &other) const {
    int32_t r = radius(), ro=other.radius();
    int32_t a = angle(), ao=other.angle();
    // x = radius()*cos(angle());
    // y = radius()*sin(angle());
    // xo = other.radius()*cos(other.angle());
    // yo = other.radius()*sin(other.angle());
    // d = (x-xo)**2 + (y-yo)**2;
    // d = r*r*ca*ca - 2*r*ca*ro*cao + ro*ro*cao*cao + r*r*sa*sa - 2*r*sa*ro*sao + ro*ro*sao*sao;
    // d = r*r + ro*ro - 2*r*ro*(ca*cao + sa*sao);
    // angle in 1/2 miliradians, cos(angle) = (2048-angle*angle/2048/2)/2048
    //                           sin(angle) = angle/2048
    // d = r*r + ro*ro - 2*r*ro*((2048-a*a/2/2048)*(2048-ao*ao/2/2048)/2048/2048 + a*ao/2048/2048);
    // d = r*r + ro*ro - 2*r*ro*((2048-a*a/2/2048)*(2048-ao*ao/2/2048) + a*ao)/2048/2048;
    // d = r*r + ro*ro - 2*r*ro*(2048*2048 - a*a/2 - ao*ao/2 + a*a*ao*a0/2/2/2048/2048)/2048/2048;
    // d = r*r + ro*ro - r*ro*(4096*2048 - a*a - ao*ao + a*a*ao*a0/2/2048/2048)/2048/2048;
    return r*r + ro*ro - r*ro*(8388608L - a*a - ao*ao + a*a*ao*ao/8388608L)/4194304L;
}

// original distance formula from old code
float Object::originalDistance(const Object &other) const {
    float radial_distance = abs(MinDistance - other.MinDistance);
    float average_radius = (MinDistance + other.MinDistance) / 2.0f;
    float angular_distance = sin(abs(angle() - other.angle()) * 0.049);
    return radial_distance + angular_distance * average_radius;
}

bool Track::valid(uint32_t now) const {
    uint32_t dt = (now - last_update);
    return dt<track_lost_dt;
}


// distance squared in mm
int32_t Track::distanceSq(const Object &detection) const {
    int32_t dx = (x/16-detection.xcoord());
    int32_t dy = (y/16-detection.ycoord());
    return dx*dx + dy*dy;
}

// return change in body angle as measured by gyro
// radians scaled by 2048
int16_t Track::updateOmegaZ(int32_t dt, int16_t omegaZ) {
    // work with radians/sec scaled by 32768
    // (2000deg/sec)/(32768 full scale)*pi/180*32768 = 34.9
    int32_t converted = omegaZ*35;
    int32_t average_omegaZ = (last_omgaz + converted)/2;
    last_omgaz = converted;
    return ((average_omegaZ/16)*(dt/1000))/1000;
}

int32_t Track::predict(uint32_t now, int16_t omegaZ) {
    int32_t dt = (now - last_predict);
    last_predict = now;
    int32_t dtheta = updateOmegaZ(dt, omegaZ);
    // predict:
    // r = sqrt(x**2+y**2)
    // theta = atan2(y, x)
    // x = x + dt*vx/1e6 + r*(cos(theta+dtheta) - cos(theta))
    // x = x + dt*vx/1e6 + r*(cos(theta)*cos(dtheta) - sin(theta)*sin(dtheta) - x/r)
    // x = x + dt*vx/1e6 + r*((x/r)*cos(dtheta) - (y/r)*sin(dtheta) - x/r)
    // x = x + dt*vx/1e6 + (x*cos(dtheta) - y*sin(dtheta) - x)
    // x = x + dt*vx/1e6 + (x*(cos(dtheta)-1) - y*sin(dtheta))
    // x = x + dt*vx/1e6 + (x*(-dtheta*dtheta/2048/2/2048) - y*dtheta/2048)
    // x = x + dt*vx/1e6 - (x*dtheta*dtheta/2048/2 + y*dtheta)/2048
    int32_t xp = x;
    x = x + ((dt/1000)*vx)/1000 - (x*dtheta*dtheta/4096L + y*dtheta)/2048L;
    // y = y + dt*vy/1e6 + r*(sin(theta+dtheta) - sin(theta));
    // y = y + dt*vy/1e6 + r*(sin(theta)*cos(dtheta) + cos(theta)*sin(dtheta) - sin(theta));
    // y = y + dt*vy/1e6 + r*((y/r)*cos(dtheta) + (x/r)*sin(dtheta) - (y/r));
    // y = y + dt*vy/1e6 + (y*cos(dtheta) + x*sin(dtheta) - y);
    // y = y + dt*vy/1e6 + (y*(cos(dtheta)-1) + x*sin(dtheta));
    // y = y + dt*vy/1e6 + (y*(-dtheta*dtheta/2048/2) + x*dtheta)/2048;
    y = y + ((dt/1000)*vy)/1000 - (y*dtheta*dtheta/4096L - xp*dtheta)/2048L;
    return dt;
}

void Track::update(const Object& best_match, int16_t omegaZ) {
    //int32_t ma = best_match.angle();
    //int32_t mr = best_match.radius();
    int32_t mx = best_match.xcoord();
    int32_t my = best_match.ycoord();
    if(!valid(best_match.Time)) {
        x = mx*16;
        y = my*16;
        vx = 0;
        vy = 0;
        num_updates = 0;
    } else {
        predict(best_match.Time, omegaZ);
        //
        // residual:
        // rx = mr*cos(ma) - x
        // rx = mr*(2048 - ma*ma/2048/2)/2048 - x
        rx = mx*16 - x;
        // ry = mr*sin(ma) - y
        ry = my*16 - y;
        //
        // correct:
        x += alpha*rx/32767;
        y += alpha*ry/32767;
        vx += beta*rx/16384;
        vy += beta*ry/16384;
    }
    num_updates++;
    last_update = best_match.Time;
}

void Track::updateNoObs(uint32_t time, int16_t omegaZ) {
    if(valid(time)) {
        predict(time, omegaZ);
    }
}

int16_t Track::angle(void) const {
    if(valid(micros()) && num_updates>min_num_updates) {
        return (y/x - (((((y*y)/x)*y)/x)/x)/3L)*2048L;
    } else {
        return 0;
    }
}

bool Track::timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ) const
{
    if(valid(micros()) && num_updates>min_num_updates) {
        // Compute extra velocity due to body frame rotation, hold that fixed
        // through the prediction step
        // Vr = d  ( r(t)*[ cos(theta(t) + t*omegaZ)-cos(theta(t)) ] )
        //      dt        [ sin(theta(t) + t*omegaZ)-sin(theta(t)) ]
        //
        // Vr = d  ( [ (x(t)*(cos(t*omegaZ)-1) - y(t)*sin(t*omegaZ)) ] )
        //      dt   [ (y(t)*(cos(t*omegaZ)-1) + x(t)*sin(t*omegaZ)) ]
        //
        // Vr = [ vx*(cos(t*omegaZ)-1) - x*omegaZ*sin(t*omegaZ) - vy*sin(t*omegaZ) - y*omegaZ*cos(t*omegaZ) ]
        //      [ vy*(cos(t*omegaZ)-1) - y*omegaZ*sin(t*omegaZ) + vx*sin(t*omegaZ) + x*omegaZ*cos(t*omegaZ) ]
        // t=0 to evaluate velocity now
        // Vr = [ -y*omegaZ ]
        //      [  x*omegaZ ]
        //
        // p(t+dt) = [ x ] + dt*[ vx - y*omegaZ ]
        //           [ y ]      [ vy + x*omegaZ ]
        // When will the target cross y==0?
        // 0 = y + t*(vy +x*omegaZ)
        int32_t ty = -y*1000/(vy+x*omegaZ/2048);
        if(ty>0) {
            int32_t xerr = (x + ty*(vx - y*omegaZ/2048)/1000)/16 - depth;
            if(xerr<xtol) {
                *dt = ty;
                return true;
            }
        }
        // When will the target cross x==depth?
        // depth = x + dt*(vx - y*omegaZ)
        int32_t tx = (((depth-x)*1000)/(vx - y*omegaZ/2048))/16;
        if(tx>0) {
            int32_t yerr = (y + tx*(vy +x*omegaZ/2048)/1000)/16;
            if(yerr<ytol) {
                *dt = tx;
                return true;
            }
        }
    }
    return false;
}
