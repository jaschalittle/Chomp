#include <Arduino.h>
#include "autofire.h"

static int32_t xtol = 200, ytol=200;

bool timeToHit(const Track &tracked_object,
               int32_t *dt, int16_t depth, int16_t omegaZ)
{
    if(tracked_object.valid(micros())) {
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
        int32_t ty = -tracked_object.y*1000/(tracked_object.vy+tracked_object.x*omegaZ/2048);
        if(ty>0) {
            int32_t xerr = (tracked_object.x + ty*(tracked_object.vx - tracked_object.y*omegaZ/2048)/1000)/16 - depth;
            if(xerr<xtol) {
                *dt = ty;
                return true;
            }
        }
        // When will the target cross x==depth?
        // depth = x + dt*(vx - y*omegaZ)
        int32_t tx = (((depth-tracked_object.x)*1000)/(tracked_object.vx - tracked_object.y*omegaZ/2048))/16;
        if(tx>0) {
            int32_t yerr = (tracked_object.y + tx*(tracked_object.vy + tracked_object.x*omegaZ/2048)/1000)/16;
            if(yerr<ytol) {
                *dt = tx;
                return true;
            }
        }
    }
    return false;
}

void setAutoFireParams(int16_t p_xtol,
                       int16_t p_ytol){
    xtol = p_xtol;
    ytol = p_ytol;
}


