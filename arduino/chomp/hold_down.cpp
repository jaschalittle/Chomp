#include <Arduino.h>
#include "hold_down.h"
#include "pins.h"
#include "telem.h"
#include "sensors.h"

static void saveHoldDownParameters();

static uint32_t tlm_triggered;
static uint16_t sample_index;
static int16_t left_vacuum_trace[128];
static int16_t right_vacuum_trace[128];
struct HoldDownParams {
    uint32_t sample_period;
} __attribute__((packed));

static struct HoldDownParams EEMEM saved_params = {
    .sample_period = 10000
};

static struct HoldDownParams params;


void holdDownSafe()
{
    tlm_triggered = 0;
    sample_index = 0;
    digitalWrite(VACUUM_VALVE_DO, LOW);
    pinMode(VACUUM_VALVE_DO, OUTPUT);
}

static void holdDownEnable(bool enable)
{
    digitalWrite(VACUUM_VALVE_DO, enable);
}

void endHoldDownSample()
{
    if(sample_index > 0)
    {
        if(sample_index < 127)
        {
            left_vacuum_trace[sample_index] = 0;
            left_vacuum_trace[sample_index] = 0;
            sample_index++;
        }
        sendVacuumTelemetry(params.sample_period, 128,
                left_vacuum_trace, right_vacuum_trace);
    }
    sample_index = 0;
    tlm_triggered = 0;
}

static void sampleHoldDownVacuum()
{
    if(sample_index == 0)
    {
        tlm_triggered = micros();
        readVacuum(left_vacuum_trace + sample_index,
                   right_vacuum_trace + sample_index);
        sample_index++;
    }
    else if((sample_index > 0) && (sample_index < 128) &&
            (micros() - tlm_triggered > sample_index * params.sample_period))
    {
        readVacuum(left_vacuum_trace + sample_index,
                   right_vacuum_trace + sample_index);
        sample_index++;
    }
}

void manualHoldDown(bool enable)
{
    holdDownEnable(enable);
    if(enable && isTLMEnabled(TLM_ID_VAC))
    {
        sampleHoldDownVacuum();
    }
    if(!enable && isTLMEnabled(TLM_ID_VAC))
    {
        endHoldDownSample();
    }
}

bool autoHoldDown()
{
    return true;
}

void saveHoldDownParameters() {
    eeprom_write_block(&params, &saved_params, sizeof(struct HoldDownParams));
}

void restoreHoldDownParameters() {
    eeprom_read_block(&params, &saved_params, sizeof(struct HoldDownParams));
}

void setHoldDownParameters(int32_t sample_period)
{
    params.sample_period = sample_period;
    saveHoldDownParameters();
}

