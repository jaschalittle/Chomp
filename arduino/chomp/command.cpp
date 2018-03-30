#include "command.h"
#include "DMASerial.h"
#include "telem.h"

#define MAXIMUM_COMMAND_LENGTH 64
enum Commands {
    CMD_ID_TRATE = 10
};

extern uint32_t telemetry_interval;
extern uint32_t leddar_telemetry_interval;
extern uint32_t drive_telem_interval;
extern uint32_t enabled_telemetry;

const uint16_t CMD_TERMINATOR=0x6666;

template <uint8_t command_id, typename command_inner> struct CommandPacket{
    uint8_t cmd_id;
    command_inner inner;
    uint16_t terminator;
    CommandPacket() : cmd_id(command_id), terminator(CMD_TERMINATOR) {};
} __attribute__((packed));
struct TelemetryRateInner {
    uint32_t small_telem_period;
    uint32_t leddar_telem_period;
    uint32_t drive_telem_period;
    uint32_t enabled_messages;
} __attribute__((packed));
typedef CommandPacket<CMD_ID_TRATE, TelemetryRateInner> TelemetryRateCommand;

static uint8_t command_buffer[MAXIMUM_COMMAND_LENGTH];
static size_t command_length=0;
static bool command_ready = false;
uint16_t command_overrun = 0;
uint16_t invalid_command = 0;
uint8_t last_command = 0;

void serialEvent(void) {
    while(DSerial.available()) {
        uint8_t in = (uint8_t)DSerial.read();
        if(!command_ready &&
            command_length<MAXIMUM_COMMAND_LENGTH) {
            command_buffer[command_length++] = in;
            if(command_length>=MAXIMUM_COMMAND_LENGTH) {
                command_length = 0;
                command_overrun ++;
            }
        }
    }
    if(command_length>2 &&
       command_buffer[command_length-1] == '\x66' &&
       command_buffer[command_length-2] == '\x66') {
        command_ready = true;
    }
}

void handle_commands(void) {
  TelemetryRateCommand *trate_cmd;
  if(command_ready) {
      last_command = command_buffer[0];
      switch(last_command) {
          case CMD_ID_TRATE:
              trate_cmd = (TelemetryRateCommand *)command_buffer;
              telemetry_interval = trate_cmd->inner.small_telem_period;
              leddar_telemetry_interval = trate_cmd->inner.leddar_telem_period;
              drive_telem_interval = trate_cmd->inner.drive_telem_period;
              enabled_telemetry = trate_cmd->inner.enabled_messages;
              debug_print(String("enabled_telemetry=")+String(enabled_telemetry, 16));
              break;
          default:
              invalid_command++;
              break;
      }
      command_length = 0;
      command_ready = false;
  }
}
