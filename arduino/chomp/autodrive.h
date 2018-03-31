#pragma once

void setDriveControlParams(int16_t p_steer_p,
                           int16_t p_steer_d,
                           int16_t p_drive_p,
                           int16_t p_drive_d);

bool pidSteer(const Track &tracked_object,
              int16_t depth, int16_t *drive_bias, int16_t *steer_bias);
