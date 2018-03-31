#include <Arduino.h>
#include "track.h"

Track::Track() :
        x(0), vx(0),
        y(0), vy(0),
        num_updates(0),
        last_update(micros()),
        last_predict(micros()),
        alpha(10000), beta(16384),
        track_lost_dt(500000),
        min_num_updates(3),
        xtol(200),
        ytol(200),
        max_off_track(600L*600L),
        max_start_distance(6000L*6000L)
        { }

bool Track::recent_update(uint32_t now) const {
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
    if(!recent_update(best_match.Time)) {
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
        if(rx>65535L) rx=65535L;
        if(rx<-65535L) rx=-65535L;
        // ry = mr*sin(ma) - y
        ry = my*16 - y;
        if(ry>65535L) ry=65535L;
        if(ry<-65535L) ry=-65535L;
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
    if(recent_update(time)) {
        predict(time, omegaZ);
    }
}

bool Track::wants_update(uint32_t now, int32_t best_distance) {
    return (( recent_update(now) && best_distance < max_off_track) ||
            (!recent_update(now) && best_distance < max_start_distance));
}

bool Track::valid(uint32_t now) const {
    return recent_update(micros()) && (num_updates>min_num_updates);
}

int16_t Track::angle(void) const {
    if(valid(micros())) {
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

void Track::setAutoFireParams(int16_t p_xtol,
                       int16_t p_ytol){
    xtol = p_xtol;
    ytol = p_ytol;
}

void Track::setTrackingFilterParams(int16_t alpha, int16_t beta,
                             int8_t p_min_num_updates,
                             uint32_t p_track_lost_dt,
                             int16_t p_max_off_track,
                             int16_t p_max_start_distance
        ) {
    min_num_updates    = p_min_num_updates;
    track_lost_dt      = p_track_lost_dt;
    max_off_track      = (int32_t)p_max_off_track*p_max_off_track;
    max_start_distance = (int32_t)p_max_start_distance*p_max_start_distance;
    alpha = alpha;
    beta = beta;
}

