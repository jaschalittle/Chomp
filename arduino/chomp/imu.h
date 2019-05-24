
enum Orientation {
    ORN_UPRIGHT = 0,
    ORN_LEFT = 1,
    ORN_RIGHT = 2,
    ORN_UNKNOWN = 3
};

void initializeIMU(void);
void processIMU(void);
void telemetryIMU(void);
bool isStationary(void);
bool getOmegaZ(int16_t *omega_z);
enum Orientation getOrientation(void);
void setIMUParameters(
    int8_t dlpf, int32_t imu_period, int32_t stationary_threshold,
    int32_t min_valid_cross, int32_t max_valid_cross, int16_t x_threshold,
    int16_t z_threshold);
