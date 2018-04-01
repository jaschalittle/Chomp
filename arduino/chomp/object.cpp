#include "object.h"
// size in mm
// r = average radius = (min+max)/2
// (pi/180)*LEDDAR_FOV/LEDDAR_SEGMENTS = (pi/180)*45/16 ~ 0.049
// theta = (left-right)*0.049
// circumferential size = theta*r = (left-right)*(min+max)*0.049/2
// leddar reports ranges in cm, so multiply this expression by 10
// and 0.49 ~ 0.5 gives
int16_t Object::size(void) const {
    return (RightEdge-LeftEdge)*(MaxDistance+MinDistance)/4;
}

// radius in mm
int16_t Object::radius(void) const {
    return SumDistance*10/(RightEdge - LeftEdge);
}

// angle in radians scaled by 2048
int16_t Object::angle(void) const {
    return -(((LeftEdge + RightEdge) - 17)*50); // really )/2-8.5)*0.049
}


// x coordinate in mm
int16_t Object::xcoord(void) const {
    int32_t ma = angle();
    return (radius()*(2048L-((ma*ma)/4096L)))/2048L;
}

// y coordinate in mm
int16_t Object::ycoord(void) const {
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

