#include "Arduino.h"
#include "leddar_wrapper.h"
#include "chump_targeting.h"

Detection get_min_detection (unsigned int num_detections) {
  Detection min_detection = Detections[0];
  for (unsigned int i = 1; i < num_detections; i++){
    if (Detections[i].Distance < min_detection.Distance) {
      min_detection = Detections[i];
    }
  }
//  Serial.print(min_detection.Distance);
  return min_detection;
}

Object_call call_nearest_obj (unsigned int num_detections) {
  unsigned int min_distance = Detections[(num_detections - 1)].Distance;
  Detection min_detection = Detections[(num_detections - 1)];
  float right_edge = 0.0;
  float left_edge = 0.0;
  Serial.print(Detections[(num_detections - 1)].Segment);
  Serial.print("/");
  Serial.print(Detections[(num_detections - 1)].Distance);
  Serial.print("\t");

  // loop is backwards because Leddar is mounted upside down!
  Object_call Objects[8];
  unsigned int num_objects = 0;
  for (int i = num_detections - 2; i >= 0; i--) {
    int delta = Detections[i].Distance - min_distance;
    if (delta < -30) {
      left_edge = 15 - (char) Detections[i].Segment;
      Serial.print("LEFT\t");
      Serial.print(Detections[i].Segment);
      Serial.print("/");
      Serial.print(Detections[i].Distance);
      Serial.print("\t");
      min_distance = Detections[i].Distance;
//      right_edge = (float) (16 - (int) Detections[i].Segment);
//      Serial.print(Detections[i].Segment);
//      Serial.print("/");
//      Serial.print(Detections[i].Distance);
//      Serial.print("\t");
    } else if (delta > 30) {
      if (left_edge > right_edge) {
        Serial.print("RIGHT\t");
        right_edge = 15 - (char) Detections[i].Segment;
        Objects[num_objects].Distance = min_distance;
        Objects[num_objects].Left_edge = left_edge;
        Objects[num_objects].Right_edge = right_edge;
        num_objects++;
        min_distance = Detections[i].Distance;
      } else {
        min_distance = Detections[i].Distance;
      }
      Serial.print(Detections[i].Segment);
      Serial.print("/");
      Serial.print(Detections[i].Distance);
      Serial.print("\t");
    } else {
      if (Detections[i].Distance < min_distance) { min_distance = Detections[i].Distance; }
      Serial.print(Detections[i].Segment);
      Serial.print("/");
      Serial.print(Detections[i].Distance);
      Serial.print("\t");
//      if (Detections[i].Segment == 0) {
//        if (left_edge > right_edge) {
//          right_edge = 16.0;
//          Objects[num_objects].Distance = min_distance;
//          Objects[num_objects].Left_edge = left_edge;
//          Objects[num_objects].Right_edge = right_edge;
//          num_objects++;
//        }
//      }
    }
  }
  if (left_edge > right_edge) {
    right_edge = 16.0;
    Objects[num_objects].Distance = min_distance;
    Objects[num_objects].Left_edge = left_edge;
    Objects[num_objects].Right_edge = right_edge;
    num_objects++;
  }

  Object_call Nearest_obj = Objects[0];
  for (int i = 1; i < num_objects; i++) {
    if (Objects[i].Distance < Nearest_obj.Distance) {
      Nearest_obj = Objects[i];
    }
  }
//  Nearest_obj.Angle = (left_edge + right_edge) / 2 - 8;
//  Serial.print(Nearest_obj.Left_edge);
//  Serial.print("\t");
//  Serial.print(Nearest_obj.Right_edge);
//  Serial.print("\t");
  Serial.print("Nearest/");
  Serial.print(Nearest_obj.Distance);
  Serial.print("/");
  Serial.print(((float) Nearest_obj.Left_edge + (float) Nearest_obj.Right_edge) / 2 - 8);
  Serial.print("\t");
//  Serial.println();
  return Nearest_obj;
}
