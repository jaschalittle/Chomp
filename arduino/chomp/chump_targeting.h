#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

struct Object
{
  unsigned int Distance;
  char Left_edge;
  char Right_edge;

  // Default constructor
  Object() : Distance(0xFFFF), Left_edge(0), Right_edge(16) { }
};

int16_t pidSteer(unsigned int num_detections, Detection* detections, uint16_t threshold);
void targetPredict(uint8_t num_detections, Detection* detections, uint16_t leadtime, 
                    int16_t* target_x_after_leadtime, int16_t* target_y_after_leadtime, int16_t* steer_bias);

#endif  // CHUMP_TARGETING_H
