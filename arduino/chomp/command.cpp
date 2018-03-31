#include "command.h"
#include "DMASerial.h"
#include "telem.h"
#include "targeting.h"
#include "autodrive.h"
#include "autofire.h"

#define MAXIMUM_COMMAND_LENGTH 64
enum Commands {
    CMD_ID_TRATE = 10,
    CMD_ID_TRKFLT = 11,
    CMD_ID_OBJSEG = 12,
    CMD_ID_AF = 13,
    CMD_ID_ADRV = 14
};

extern uint32_t telemetry_interval;
extern uint32_t leddar_telemetry_interval;
extern uint32_t drive_telem_interval;
extern uint32_t enabled_telemetry;
extern Track tracked_object;

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


struct ObjectSegmentationInner {
    int16_t min_object_size;
    int16_t max_object_size;
    int16_t edge_call_threshold;
} __attribute__((packed));
typedef CommandPacket<CMD_ID_OBJSEG, ObjectSegmentationInner> ObjectSegmentationCommand;


struct AutoFireInner {
    int16_t xtol;
    int16_t ytol;
    int16_t ttol;
} __attribute__((packed));
typedef CommandPacket<CMD_ID_AF, AutoFireInner> AutoFireCommand;


struct AutoDriveInner {
    int16_t steer_p;
    int16_t steer_d;
    int16_t steer_max;
    int16_t gyro_gain;
    int16_t drive_p;
    int16_t drive_d;
    int16_t drive_max;
} __attribute__((packed));
typedef CommandPacket<CMD_ID_ADRV, AutoDriveInner> AutoDriveCommand;


struct TrackingFilterInner {
    int16_t alpha;
    int16_t beta;
    int8_t min_num_updates;
    uint32_t track_lost_dt;
    int16_t max_off_track;
    int16_t max_start_distance;
} __attribute__((packed));
typedef CommandPacket<CMD_ID_TRKFLT, TrackingFilterInner> TrackingFilterCommand;


static uint8_t command_buffer[MAXIMUM_COMMAND_LENGTH];
static size_t command_length=0;
static bool command_ready = false;
uint16_t command_overrun = 0;
uint16_t invalid_command = 0;
uint16_t valid_command = 0;
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
  TrackingFilterCommand *trkflt_cmd;
  ObjectSegmentationCommand *objseg_cmd;
  AutoFireCommand *af_cmd;
  AutoDriveCommand *adrv_cmd;
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
              valid_command++;
              break;
          case CMD_ID_TRKFLT:
              trkflt_cmd = (TrackingFilterCommand *)command_buffer;
              tracked_object.setTrackingFilterParams(trkflt_cmd->inner.alpha,
                                      trkflt_cmd->inner.beta,
                                      trkflt_cmd->inner.min_num_updates,
                                      trkflt_cmd->inner.track_lost_dt,
                                      trkflt_cmd->inner.max_off_track,
                                      trkflt_cmd->inner.max_start_distance);
              valid_command++;
              break;
          case CMD_ID_OBJSEG:
              objseg_cmd = (ObjectSegmentationCommand *)command_buffer;
              setObjectSegmentationParams(
                                      objseg_cmd->inner.min_object_size,
                                      objseg_cmd->inner.max_object_size,
                                      objseg_cmd->inner.edge_call_threshold);
              break;
          case CMD_ID_AF:
              af_cmd = (AutoFireCommand *)command_buffer;
              setAutoFireParams(af_cmd->inner.xtol,
                                af_cmd->inner.ytol,
                                af_cmd->inner.ttol);
              valid_command++;
              break;
          case CMD_ID_ADRV:
              adrv_cmd = (AutoDriveCommand *)command_buffer;
              setDriveControlParams(adrv_cmd->inner.steer_p,
                                    adrv_cmd->inner.steer_d,
                                    adrv_cmd->inner.steer_max,
                                    adrv_cmd->inner.gyro_gain,
                                    adrv_cmd->inner.drive_p,
                                    adrv_cmd->inner.drive_d,
                                    adrv_cmd->inner.drive_max);
              valid_command++;
              break;
          default:
              invalid_command++;
              break;
      }
      command_length = 0;
      command_ready = false;
  }
}
