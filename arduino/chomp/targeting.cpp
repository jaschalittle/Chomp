#include "Arduino.h"
#include "leddar_io.h"
#include "targeting.h"
#include "pins.h"
#include "sensors.h"
#include "drive.h"
#include <stdlib.h>
#include <math.h>
#include "imu.h"


// #define P_COEFF 2000  // coeff for radians, should correspond to 100 for segments. seems okay for Chump, too fast for Chomp
#define P_COEFF 1800
#define STEER_BIAS_CAP 500

// object sizes are in mm
#define MIN_OBJECT_SIZE 200
#define MAX_OBJECT_SIZE 1800
#define EDGE_CALL_THRESHOLD 60
#define MIN_NUM_UPDATES 5

static const int32_t track_lost_dt = 2000000;

struct Object
{
    uint16_t MinDistance, MaxDistance;
    int8_t LeftEdge, RightEdge;
    uint32_t Time;

    // Default constructor
    Object() : MinDistance(10000), MaxDistance(0), LeftEdge(0), RightEdge(0),
               Time(0) { }
    inline int16_t size(void) const;
    inline int16_t angle(void) const;
    inline int16_t radius(void) const;
    inline int32_t distanceSq(const Object &other) const;
    float originalDistance(const Object &other) const;
};

#define TRACK_BUFFER_LENGTH 16
struct Track
{
    int32_t x, vx, y, vy;
    uint32_t num_updates;
    uint32_t last_update;
    int32_t last_omgaz;

    int16_t alpha, beta;

    // Default constructor
    Track() :
        x(0), vx(0),
        y(0), vy(0),
        num_updates(0),
        last_update(micros())
        { }
    inline int32_t computedt(uint32_t now);
    void update(const Object& best_match, int16_t omegaZ);
    int32_t distanceSq(const Object& obj) const;
    int16_t updateOmegaZ(int32_t dt, int16_t omegaZ);
    void updateNoObs(uint32_t inter_leddar_time, int16_t omegaZ);
    bool timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ) const;
    int16_t angle(void) const;
};

// Consider adding requirement that near objects must cover multiple segments
static Track tracked_object;
void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS], int16_t distance_threshold) {

    // call all objects in frame by detecting edges
    int16_t last_seg_distance = min_detections[0].Distance;
    int16_t min_obj_distance = min_detections[0].Distance;
    int16_t max_obj_distance = min_detections[0].Distance;
    int16_t right_edge = -1;
    int16_t left_edge = 0;
    Object objects[8];
    uint8_t num_objects = 0;
    uint32_t now = micros();
    // this currently will not call a more distant object obscured by a nearer object, even if both edges of more distant object are visible
    for (uint8_t i = 1; i < 16; i++) {
        int16_t delta = (int16_t) min_detections[i].Distance - last_seg_distance;
        if (delta < -EDGE_CALL_THRESHOLD) {
            left_edge = i;
            min_obj_distance = min_detections[i].Distance;
            max_obj_distance = min_detections[i].Distance;
        } else if (delta > EDGE_CALL_THRESHOLD) {
            // call object if there is an unmatched left edge
            if (left_edge > right_edge) {
                right_edge = i;
                objects[num_objects].MinDistance = min_obj_distance;
                objects[num_objects].MaxDistance = max_obj_distance;
                objects[num_objects].LeftEdge = left_edge;
                objects[num_objects].RightEdge = right_edge;
                objects[num_objects].Time = now;
                int16_t size = objects[num_objects].size();
                if(size>MIN_OBJECT_SIZE && size<MAX_OBJECT_SIZE) {
                    num_objects++;
                }
            }
        }
        min_obj_distance = min(min_obj_distance, min_detections[i].Distance);
        max_obj_distance = max(max_obj_distance, min_detections[i].Distance);
        last_seg_distance = min_detections[i].Distance;
    }

    // call object after loop if no matching right edge seen for a left edge-- end of loop can be a right edge. do not call entire FOV an object
    if (left_edge != 0 && left_edge > right_edge) {
        right_edge = LEDDAR_SEGMENTS;
        objects[num_objects].MinDistance = min_obj_distance;
        objects[num_objects].MaxDistance = max_obj_distance;
        objects[num_objects].LeftEdge = left_edge;
        objects[num_objects].RightEdge = right_edge;
        objects[num_objects].Time = now;
        int16_t size = objects[num_objects].size();
        if(size>MIN_OBJECT_SIZE && size<MAX_OBJECT_SIZE) {
            num_objects++;
        }
    }

    int16_t omegaZ = 0;
    getOmegaZ(&omegaZ);
    // if an object has been called, assign it to existing tracked object or to new one
    if (num_objects > 0) {
        int8_t best_match = 0;
        int32_t best_distance = tracked_object.distanceSq(objects[best_match]);
        for (uint8_t i = 1; i < num_objects; i++) {
            int32_t distance = tracked_object.distanceSq(objects[i]);
            if (distance < best_distance) {
                best_distance = distance;
                best_match = i;
            }
        }
        tracked_object.update(objects[best_match], omegaZ);
    // below is called if no objects called in current Leddar return
    } else {
        tracked_object.updateNoObs(now, omegaZ);
    }
}

void pidSteer (const Detection (&detections)[LEDDAR_SEGMENTS], uint16_t distance_threshold, int16_t *steer_bias) {
    trackObject(detections, distance_threshold);
    int16_t calculated_steer_bias = P_COEFF * tracked_object.angle();
    if (calculated_steer_bias > STEER_BIAS_CAP) { calculated_steer_bias = STEER_BIAS_CAP; }
    if (calculated_steer_bias < -STEER_BIAS_CAP) { calculated_steer_bias = -STEER_BIAS_CAP; }
    *steer_bias = calculated_steer_bias;
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
    return (MaxDistance + MinDistance)*5; // really *10/2
}

// angle in radians scaled by 2048
inline int16_t Object::angle(void) const {
    return (((LeftEdge + RightEdge) - 17)*50); // really )/2-8.5)*0.049
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
    return r*r + ro*ro - r*ro*(8388608 - a*a - ao*ao + a*a*ao*ao/8388608)/4194304;
}

// original distance formula from old code
float Object::originalDistance(const Object &other) const {
    float radial_distance = abs(MinDistance - other.MinDistance);
    float average_radius = (MinDistance + other.MinDistance) / 2.0f;
    float angular_distance = sin(abs(angle() - other.angle()) * 0.049);
    return radial_distance + angular_distance * average_radius;
}


// distance squared in mm
int32_t Track::distanceSq(const Object &detection) const {
    uint32_t dt = (detection.Time - last_update);
    if(dt>track_lost_dt) {
        return detection.radius()*detection.radius();
    } else {
        int32_t rd=detection.radius(), ad=detection.angle();
        // d = (x-rd*cad)**2 + (y-rd*sad)**2;
        // d = x*x - 2*x*rd*cad + rd*rd*cad*cad + y*y - 2*y*rd*sad + rd*rd*sad*sad;
        // d = x*x + y*y + rd*rd - 2*rd*(x*cad + y*sad);
        // d = x*x + y*y + rd*rd - 2*rd*(x*(2048-ad*ad/2048/2)/2048 + y*ad/2048);
        // d = x*x + y*y + rd*rd - 2*rd*(x*(2048-ad*ad/2048/2) + y*ad)/2048;
        // d = x*x + y*y + rd*rd - rd*(x*2048*2 - x*ad*ad/2048 + y*ad*2)/2048;
        return x*x + y*y + rd*rd - rd*(x*4096 - x*ad*ad/2048 + y*ad*2)/2048;
    }
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

inline int32_t Track::computedt(uint32_t now)
{
    int32_t dt = (now - last_update);
    last_update = now;
    return dt;
}

void Track::update(const Object& best_match, int16_t omegaZ) {
    int32_t dt = computedt(best_match.Time);
    int32_t dtheta = updateOmegaZ(dt, omegaZ);
    int32_t ma = best_match.angle();
    int32_t mr = best_match.radius();
    int32_t mx = (mr*(2048-((ma*ma)/4096)))/2048; 
    int32_t my = mr*(ma/2048);
    if(num_updates == 0) {
        x = mx;
        y = my;
        vx = 0;
        vy = 0;
    } else {
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
        x = x + ((dt/1000)*vx)/1000 - (x*dtheta*dtheta/4096 + y*dtheta)/2048;
        // y = y + dt*vy/1e6 + r*(sin(theta+dtheta) - sin(theta));
        // y = y + dt*vy/1e6 + r*(sin(theta)*cos(dtheta) + cos(theta)*sin(dtheta) - sin(theta));
        // y = y + dt*vy/1e6 + r*((y/r)*cos(dtheta) + (x/r)*sin(dtheta) - (y/r));
        // y = y + dt*vy/1e6 + (y*cos(dtheta) + x*sin(dtheta) - y);
        // y = y + dt*vy/1e6 + (y*(cos(dtheta)-1) + x*sin(dtheta));
        // y = y + dt*vy/1e6 + (y*(-dtheta*dtheta/2048/2) + x*dtheta)/2048;
        y = y + ((dt/1000)*vy)/1000 - (y*dtheta*dtheta/4096 - xp*dtheta)/2048;
        //
        // residual:
        // rx = mr*cos(ma) - x
        // rx = mr*(2048 - ma*ma/2048/2)/2048 - x
        int32_t rx = mx - x;
        // ry = mr*sin(ma) - y
        int32_t ry = my - y;
        //
        // correct:
        x += alpha*rx/32768;
        y += alpha*ry/32768;
        vx += beta*rx/32768;
        vy += beta*ry/32768;
    }
    num_updates++;
}

void Track::updateNoObs(uint32_t time, int16_t omegaZ) {
    int32_t dt = computedt(time);
    updateOmegaZ(dt, omegaZ);
    num_updates = 0;
}

int16_t Track::angle(void) const {
    return (y/x - (((((y*y)/x)*y)/x)/x)/3)*2048;
}

uint32_t sqrti(uint32_t x) {
    // https://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
    uint32_t op  = x;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

    // "one" starts at the highest power of four <= than the argument.
    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

bool Track::timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ) const
{
    int32_t predict_dt = micros() - last_update;
    if(predict_dt<track_lost_dt && num_updates>MIN_NUM_UPDATES) {
        // p(t+dt) = [ x ] + dt*[ vx ] + r(t)*[ cos(theta(t) + dt*omegaZ)-cos(theta(t)) ]
        //           [ y ]      [ vy ]        [ sin(theta(t) + dt*omegaZ)-sin(theta(t)) ]
        //
        // hit when ||p(t+dt) - [depth, 0].T|| < tol
        //
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
        //
        // ((x-depth) + dt*(vx-y*omegaZ))**2 + (y + dt*(vy+x*omegaZ))**2
        // (x-depth)**2 + 2*(x-depth)*dt*(vx-y*omegaZ) + dt*dt*(vx-y*omegaZ)**2 +
        // y**2 +         2*y*dt*(vy+x*omegaZ) +         dt*dt*(vy+x*omegaZ)**2
        //
        //       (x-depth)**2 + y**2
        // dt*   2*((x-depth)*(vx-y*omegaZ) + y*(vy+x*omegaZ))
        // dt*dt*((vx-y*omegaZ)*(vy+x*omegaZ))**2
        //
        //effective velocities including rotation
        int32_t evx = (vx-y*omegaZ/2048);
        int32_t evy = (vy+x*omegaZ/2048);
        //target x distance
        int32_t tx = x-depth;
        //polynomial coefficients
        int32_t a = (evx*evy)*(evx*evy);
        int32_t b = 2*(tx*evx + y*evy);
        int32_t c = tx*tx + y*y;
        //quadratic formula
        int32_t det = (b*b-4*a*c);
        if(det<0) {
            // negative determinant, no real solutions
            return false;
        }
        det = sqrti(det);
        int32_t dtp = (-b + det)/a/2;
        int32_t dtm = (-b - det)/a/2;
        if(dtp>0 && dtp<dtm) {
            *dt = dtp;
            return true;
        }
        if(dtm>0 && dtm<dtp) {
            *dt = dtm;
            return true;
        }
    }
    return false;
}
