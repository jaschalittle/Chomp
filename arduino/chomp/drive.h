#ifndef DRIVE_H
#define DRIVE_H

void driveSetup();
void drive(int16_t l_drive_value, int16_t r_drive_value, bool drive_enabled);
int16_t getAvgDriveCommand();

#endif // DRIVE_H
