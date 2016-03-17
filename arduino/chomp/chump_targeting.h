#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

struct Object
{
  unsigned int Distance;
  char Left_edge;
  char Right_edge;

  // Default constructor
  Object() : Distance(0xFFFF), Left_edge(0.0), Right_edge(16.0) { }
};

int16_t pidSteer(unsigned int num_detections, Detection* detections);
// float error_derivative(float errors*);
// Detection GetMinDetection(unsigned int num_detections, Detection* detections);
// Object CallNearestObj(unsigned int num_detections, Detection* detections);

#endif  // CHUMP_TARGETING_H
