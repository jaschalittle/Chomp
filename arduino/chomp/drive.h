#ifndef DRIVE_H
#define DRIVE_H

void driveSetup();
void updateDriveHistory(int16_t l_drive_value, int16_t r_drive_value);
void drive(int16_t &l_drive_value, int16_t &r_drive_value);
int16_t getAvgDriveCommand();

#endif // DRIVE_H
