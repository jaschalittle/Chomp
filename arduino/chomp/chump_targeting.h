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

Detection get_min_detection(unsigned int num_detections);
Object call_nearest_obj(unsigned int num_detections);

#endif  // CHUMP_TARGETING_H
