#include "protocol.h"
#include <string.h>

int protocol_pack_binary(const gesture_data_packet_t *packet, uint8_t *buffer, size_t buf_len)
{
    if (buf_len < sizeof(binary_packet_t)) {
        return -1;
    }

    binary_packet_t *bin = (binary_packet_t *)buffer;
    memset(bin, 0, sizeof(binary_packet_t));

    bin->header = 0xAA;
    bin->mode = (uint8_t)packet->mode;
    bin->progress = (uint8_t)(packet->switch_progress * 100);

    for (int i = 0; i < 5; i++) {
        bin->fingers[i] = packet->finger_percent[i];
    }

    bin->pitch = (int16_t)(packet->pitch * 10);
    bin->roll = (int16_t)(packet->roll * 10);
    bin->yaw = (int16_t)(packet->yaw * 10);

    bin->checksum = 0;
    uint8_t *p = (uint8_t *)bin;
    for (size_t i = 0; i < sizeof(binary_packet_t) - 1; i++) {
        bin->checksum += p[i];
    }

    return sizeof(binary_packet_t);
}

void protocol_init_packet(gesture_data_packet_t *packet)
{
    memset(packet, 0, sizeof(gesture_data_packet_t));
    packet->mode = MODE_CAR;
}
