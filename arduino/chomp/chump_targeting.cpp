#include "Arduino.h"
#include "leddar_io.h"
#include "chump_targeting.h"

Detection get_min_detection (unsigned int num_detections, Detection* detections) {
  Detection min_detection = detections[0];
  for (unsigned int i = 1; i < num_detections; i++){
    if (detections[i].Distance < min_detection.Distance) {
      min_detection = detections[i];
    }
  }
//  Serial.print(min_detection.Distance);
  return min_detection;
}

Object call_nearest_obj (unsigned int num_detections, Detection* detections) {
  unsigned int min_distance = detections[(num_detections - 1)].Distance;
  Detection min_detection = detections[(num_detections - 1)];
  float right_edge = 0.0;
  float left_edge = 0.0;
  Serial.print(detections[(num_detections - 1)].Segment);
  Serial.print("/");
  Serial.print(detections[(num_detections - 1)].Distance);
  Serial.print("\t");

  // loop is backwards because Leddar is mounted upside down!
  Object objects[8];
  unsigned int num_objects = 0;
  for (int i = num_detections - 2; i >= 0; i--) {
    int delta = detections[i].Distance - min_distance;
    if (delta < -30) {
      left_edge = 15 - (char) detections[i].Segment;
      Serial.print("LEFT\t");
      Serial.print(detections[i].Segment);
      Serial.print("/");
      Serial.print(detections[i].Distance);
      Serial.print("\t");
      min_distance = detections[i].Distance;
//      right_edge = (float) (16 - (int) detections[i].Segment);
//      Serial.print(detections[i].Segment);
//      Serial.print("/");
//      Serial.print(detections[i].Distance);
//      Serial.print("\t");
    } else if (delta > 30) {
      if (left_edge > right_edge) {
        Serial.print("RIGHT\t");
        right_edge = 15 - (char) detections[i].Segment;
        objects[num_objects].Distance = min_distance;
        objects[num_objects].Left_edge = left_edge;
        objects[num_objects].Right_edge = right_edge;
        num_objects++;
        min_distance = detections[i].Distance;
      } else {
        min_distance = detections[i].Distance;
      }
      Serial.print(detections[i].Segment);
      Serial.print("/");
      Serial.print(detections[i].Distance);
      Serial.print("\t");
    } else {
      if (detections[i].Distance < min_distance) { min_distance = detections[i].Distance; }
      Serial.print(detections[i].Segment);
      Serial.print("/");
      Serial.print(detections[i].Distance);
      Serial.print("\t");
//      if (detections[i].Segment == 0) {
//        if (left_edge > right_edge) {
//          right_edge = 16.0;
//          objects[num_objects].Distance = min_distance;
//          objects[num_objects].Left_edge = left_edge;
//          objects[num_objects].Right_edge = right_edge;
//          num_objects++;
//        }
//      }
    }
  }
  if (left_edge > right_edge) {
    right_edge = 16.0;
    objects[num_objects].Distance = min_distance;
    objects[num_objects].Left_edge = left_edge;
    objects[num_objects].Right_edge = right_edge;
    num_objects++;
  }

  Object nearest_obj = objects[0];
  for (int i = 1; i < num_objects; i++) {
    if (objects[i].Distance < nearest_obj.Distance) {
      nearest_obj = objects[i];
    }
  }
//  nearest_obj.Angle = (left_edge + right_edge) / 2 - 8;
//  Serial.print(nearest_obj.Left_edge);
//  Serial.print("\t");
//  Serial.print(nearest_obj.Right_edge);
//  Serial.print("\t");
  Serial.print("Nearest/");
  Serial.print(nearest_obj.Distance);
  Serial.print("/");
  Serial.print(((float) nearest_obj.Left_edge + (float) nearest_obj.Right_edge) / 2 - 8);
  Serial.print("\t");
//  Serial.println();
  return nearest_obj;
}
