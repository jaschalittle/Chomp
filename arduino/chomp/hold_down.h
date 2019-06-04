void holdDownSafe();
void manualHoldDown(bool enable);
bool autoHoldDown(uint32_t start, uint32_t now);
void autoHoldDownEnd();
void restoreHoldDownParameters();
void setHoldDownParameters(uint32_t sample_period, uint32_t start_delay);
uint32_t getAutoholdStartDelay();
