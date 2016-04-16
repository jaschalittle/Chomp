#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

struct Object
{
    uint16_t Distance;
    uint8_t Left_edge;
    uint8_t Right_edge;
    float Angle;
    uint16_t Size;
    uint16_t Distance_history[10];
    float Angle_history[10];
    uint8_t Leddar_history_index;
    float Closing_velocity;
    float Angular_velocity;

    // Default constructor
    Object() : Distance(10000), Left_edge(0), Right_edge(16), Angle(0.0) { }
};

#define TRACK_BUFFER_LENGTH 20
struct Track
{
    uint16_t Distance;
    uint16_t Size;
    float Angle;
    uint16_t Distance_history[TRACK_BUFFER_LENGTH];
    float Angle_delta_history[TRACK_BUFFER_LENGTH];
    uint32_t Inter_leddar_times[TRACK_BUFFER_LENGTH];
    uint8_t Leddar_history_index;
    uint16_t Num_obs;
    uint16_t Num_no_obs;
    float Gyro_history[TRACK_BUFFER_LENGTH];
    uint8_t Gyro_history_index;
    float Closing_velocity;
    float Angular_velocity;

    // Default constructor
    Track() : Num_obs(0), Angle(0.0), Size(0) { }
    void reset();
    void update(Object best_match, uint32_t inter_leddar_time);
    void countNoObs(uint32_t inter_leddar_time);
    void estimateVelocity(float our_angular_velocity);
};

// angle deltas over 5 times / total time = avg angular velocity per unit time

int16_t pidSteer(unsigned int num_detections, Detection* detections, uint16_t threshold, int16_t* steer_bias, bool reset_targeting);

#endif  // CHUMP_TARGETING_H
