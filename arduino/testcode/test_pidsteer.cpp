#include <iostream>
#include <string>
#include "chump_targeting.h"
#include "cosmos_listener.h"

int main()
{
    const char * COSMOS="7879";
    int fd = COSMOS_connect("localhost", COSMOS);

    struct timeval stamp;
    char *target=NULL, *packet=NULL;
    uint32_t datalen;
    uint8_t *data=NULL;
    bool reset_targeting=true;
    while(true)
    {
        COSMOS_readpkt(fd, &stamp, &target, &packet, &datalen, &data);
        // std::cout << target << ":" << packet << "(" << datalen << ")" << std::endl;
        if(std::string(target) == "CHOMP" && std::string(packet) == "LEDDARV2")
        {
/*
TELEMETRY CHOMP LEDDARV2 LITTLE_ENDIAN "Leddar telemetry"
    APPEND_ID_ITEM PKTID 8 UINT 15 "Packet ID which must be 2"
    APPEND_ITEM STATE 16 INT "State"
        STATE FAR_ZONE 1 GREEN
        STATE ARM_ZONE 2 YELLOW
        STATE HIT_ZONE 3 RED
        STATE PREDICTIVE_HIT_ZONE 4 RED
    APPEND_ITEM COUNT 16 INT "Count"
    APPEND_ARRAY_ITEM RANGE 16 UINT 256 "Ranges"
        UNITS "centimeters" "cm"
    APPEND_ARRAY_ITEM AMPLITUDE 16 UINT 256 "Reflection amplitudes"
*/
            uint16_t (*range)[LEDDAR_SEGMENTS];
            uint16_t (*amplitude)[LEDDAR_SEGMENTS];
            range = (uint16_t(*)[LEDDAR_SEGMENTS])(data+5);
            amplitude = (uint16_t(*)[LEDDAR_SEGMENTS])(data+5+16);
            Detection det[LEDDAR_SEGMENTS];
            for(size_t i=0;i<LEDDAR_SEGMENTS;i++)
            {
                det[i].Segment = i;
                det[i].Distance = *range[i];
                det[i].Amplitude = *amplitude[i];
            }
            int16_t steer_bias;
            pidSteer(det, 600, &steer_bias, reset_targeting);
            reset_targeting = false;
            std::cout << steer_bias << std::endl;
        } else if(std::string(target) == "CHOMP" && std::string(packet) == "SBS") {
/*
TELEMETRY CHOMP SBS LITTLE_ENDIAN "S.Bus"
    APPEND_ID_ITEM PKTID 8 UINT 12 "Packet ID which must be 12"
    APPEND_ITEM AUTO_SELF_RIGHT 1 UINT "Auto self-right enabled"
    APPEND_ITEM GENTLE_RETRACT 1 UINT "GENTLE_HAMMER_RETRACT"
    APPEND_ITEM GENTLE_FIRE 1 UINT "GENTLE_HAMMER_FIRE"
    APPEND_ITEM FLAME_PULSE 1 UINT "FLAME_PULSE"
    APPEND_ITEM FLAME_CTRL 1 UINT "FLAME_CTRL"
    APPEND_ITEM MANUAL_RETRACT 1 UINT "HAMMER_RETRACT"
    APPEND_ITEM MANUAL_FIRE 1 UINT "HAMMER_FIRE"
    APPEND_ITEM AUTO_HAMMER 1 UINT "AUTO_HAMMER_ENABLE"
    APPEND_ITEM PAD0 4 UINT "PAD"
    APPEND_ITEM MANUAL_SELF_RIGHT_RIGHT 1 UINT "Manual self right right"
    APPEND_ITEM MANUAL_SELF_RIGHT_LEFT 1 UINT "Manual self right left"
    APPEND_ITEM DANGER 1 UINT "DANGER_CTRL"
    APPEND_ITEM PAD1 1 UINT
*/
            struct SBS {
                uint8_t AUTO_SELF_RIGHT:1;
                uint8_t GENTLE_RETRACT:1;
                uint8_t GENTLE_FIRE:1;
                uint8_t FLAME_PULSE:1;
                uint8_t FLAME_CTRL:1;
                uint8_t MANUAL_RETRACT:1;
                uint8_t MANUAL_FIRE:1;
                uint8_t AUTO_HAMMER:1;
                uint8_t PAD0:4;
                uint8_t MANUAL_SELF_RIGHT_RIGHT:1;
                uint8_t MANUAL_SELF_RIGHT_LEFT:1;
                uint8_t DANGER:1;
                uint8_t PAD1:1;
            } __attribute__((__packed__)) *sbs;
            sbs = (SBS*)(data+1);
        }
    }
    return 0;
}
