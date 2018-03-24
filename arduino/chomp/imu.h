#define NUM_STABLE_ORIENTATIONS 8

enum Orientation {
    ORN_UPRIGHT = 0,
    ORN_LEFT = 1,
    ORN_RIGHT = 2,
    ORN_FRONT = 3,
    ORN_TOP = 4,
    ORN_TOP_RIGHT = 5,
    ORN_TOP_LEFT = 6,
    ORN_TAIL = 7,
    ORN_UNKNOWN=NUM_STABLE_ORIENTATIONS
};

void initializeIMU(void);
void processIMU(void);
void telemetryIMU(void);
bool isStationary(void);
bool getOmegaZ(int16_t *omega_z);
enum Orientation getOrientation(void);
