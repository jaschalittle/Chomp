#ifndef DRIVE_H
#define DRIVE_H

void driveSetup();
void updateDriveHistory(int16_t l_drive_value, int16_t r_drive_value);

// values passed by reference. This routine clamps the values to the maximum
// range of -1000 to 1000, passing a reference ensures that the clamped values
// are stored in the history.
void drive(int16_t &l_drive_value, int16_t &r_drive_value);
int16_t getAvgDriveCommand();
void driveTelem(void);

#endif // DRIVE_H
