#ifndef LEDDAR_IO_H
#define LEDDAR_IO_H

#define LEDDAR_FREQ 50

// Represents a measurement
struct Detection
{
  uint8_t Segment;
  int16_t Distance;
  int16_t Amplitude;

  // Default constructor
  Detection() : Segment(0), Distance(10000), Amplitude(0) { }
};

void leddarWrapperInit();

void requestDetections();
bool bufferDetections();
uint8_t parseDetections();

Detection* getDetections();
void getMinDetections(uint8_t detection_count, Detection* inputDetections, Detection* outputMinDetections);

#endif  // LEDDAR_IO_H
