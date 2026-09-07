#include <assert.h>
#include "../serial_imu/src/ch_serial.c"

static int feed(raw_t *raw, const uint8_t *payload, int n, int corrupt)
{
    uint8_t frame[MAXRAWLEN] = {0x5a, 0xa5};
    frame[2] = n & 0xFF;
    frame[3] = n >> 8;
    memcpy(frame + 6, payload, n);
    uint16_t crc = 0;
    crc16_update(&crc, frame, 4);
    crc16_update(&crc, frame + 6, n);
    if (corrupt) crc ^= 1;
    memcpy(frame + 4, &crc, 2);
    int result = 0;
    for (int i = 0; i < n + 6; ++i) result = ch_serial_input(raw, frame[i]);
    return result;
}

int main(void)
{
    raw_t raw = {0};
    uint8_t id[] = {kItemID, 1};
    assert(feed(&raw, id, sizeof(id), 0) == 1);
    assert(feed(&raw, id, sizeof(id), 1) == -1);
    assert(raw.imu[0].id == 1);
    uint8_t truncated[] = {kItemRotationQuat};
    assert(feed(&raw, truncated, sizeof(truncated), 0) == -1);
    uint8_t ids[18];
    for (int i = 0; i < 9; ++i) { ids[2*i] = kItemID; ids[2*i+1] = 1; }
    assert(feed(&raw, ids, sizeof(ids), 0) == -1);
    uint8_t gateway[8] = {KItemGWSOL, 0, MAX_NODE_SIZE + 1};
    assert(feed(&raw, gateway, sizeof(gateway), 0) == -1);
    gateway[2] = 1;
    assert(feed(&raw, gateway, sizeof(gateway), 0) == -1);
    uint8_t quat[17] = {kItemRotationQuat};
    float one = 1;
    memcpy(quat + 1, &one, sizeof(one));
    assert(feed(&raw, quat, sizeof(quat), 0) == 1);
    assert(raw.imu[0].quat[0] == 1);
    assert(feed(&raw, id, sizeof(id), 0) == 1);
    assert(raw.imu[0].quat[0] == 0);  /* Partial packets cannot reuse old fields. */
    uint8_t acceleration[7] = {kItemAccRaw, 1, 0, 2, 0, 3, 0};
    assert(feed(&raw, acceleration, sizeof(acceleration), 0) == 1);
    return 0;
}
