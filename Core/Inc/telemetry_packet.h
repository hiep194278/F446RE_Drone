#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t header1;    // Fixed sync marker: 0xAA
    uint8_t header2;    // Fixed sync marker: 0x55
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    uint8_t checksum;   // XOR checksum over payload bytes
} CustomTelemetryPacket_t;
#pragma pack(pop)

#endif
