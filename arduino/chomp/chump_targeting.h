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
    Track() : Num_obs(0), Angle(0.0) { }
    void reset() {
        Distance = 0;
        Size = 0;
        Angle = 0.0;
        for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { 
            Distance_history[i] = 0;
            Angle_delta_history[i] = 0.0;
            Inter_leddar_times[i] = 0;
            Gyro_history[i] = 0.0;
        }
        Leddar_history_index = 0;
        Num_obs = 0;
        Num_no_obs = 0;
        Gyro_history_index = 0;
        Closing_velocity = 0.0;
        Angular_velocity = 0.0;
    }
    void update(Object best_match, uint32_t inter_leddar_time) {
        Distance = best_match.Distance;
        Size = best_match.Size;
        Distance_history[Leddar_history_index] = best_match.Distance;
        Angle_delta_history[Leddar_history_index] = best_match.Angle - Angle;
        Angle = best_match.Angle;
        Inter_leddar_times[Leddar_history_index] = inter_leddar_time;
        Leddar_history_index = (Leddar_history_index + 1) % TRACK_BUFFER_LENGTH;
        Num_obs += 1;
        Num_no_obs = 0;
    }
    void countNoObs(uint32_t inter_leddar_time) {
        Distance_history[Leddar_history_index] = Distance;
        Angle_delta_history[Leddar_history_index] = 0.0;
        Inter_leddar_times[Leddar_history_index] = inter_leddar_time;
        Leddar_history_index = (Leddar_history_index + 1) % TRACK_BUFFER_LENGTH;
        Num_no_obs += 1;
    }
    void estimateVelocity(float our_angular_velocity) {
        // calculate total time of buffered data in ms
        uint32_t total_time = 100;  // five Leddar reads is 100 ms
        // for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { total_time += Inter_leddar_times[i]; Serial.print(Inter_leddar_times[i]); Serial.print(" ");}
        // total_time /= 1000; // in ms
        
        // calculate average enemy angular velocity in robot frame
        Angular_velocity = 0.0;
        for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) {
            Angular_velocity += Angle_delta_history[i];
        }
        Angular_velocity = Angular_velocity / (float) total_time * 1000;  // average radians per s
        
        // estimate how fast we've been spinning in buffered period
        Gyro_history[Gyro_history_index] = our_angular_velocity;
        Gyro_history_index = (Gyro_history_index + 1) % TRACK_BUFFER_LENGTH;
        our_angular_velocity = 0.0;
        for (uint8_t i = 0; i < TRACK_BUFFER_LENGTH; i++) { our_angular_velocity += Gyro_history[i]; }
        our_angular_velocity /= TRACK_BUFFER_LENGTH;  // average radians per sec
        
        // calculate average enemy angular velocity in world frame
        Angular_velocity = Angular_velocity - our_angular_velocity;
    }
};

// angle deltas over 5 times / total time = avg angular velocity per unit time

int16_t pidSteer(unsigned int num_detections, Detection* detections, uint16_t threshold, int16_t* steer_bias, bool reset_targeting);

#endif  // CHUMP_TARGETING_H
