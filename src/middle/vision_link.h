#ifndef VISION_LINK_H_
#define VISION_LINK_H_

#include "zf_common_typedef.h"

#define VISION_LINK_FRAME_SIZE                 (24u)
#define VISION_LINK_MEASUREMENT_TIMEOUT_MS     (80u)
#define VISION_LINK_OFFLINE_TIMEOUT_MS         (100u)

#define VISION_LINK_FLAG_MEASURED_VALID        (0x01u)
#define VISION_LINK_FLAG_PREDICT_ONLY          (0x02u)
#define VISION_LINK_FLAG_TRACKER_READY         (0x04u)
#define VISION_LINK_FLAG_CALIBRATION_VALID     (0x08u)

typedef struct
{
    uint8 flags;
    uint16 sequence;
    uint32 capture_ms;
    int16 position_dmm;
    int16 velocity_mm_s;
    uint8 confidence;
    uint8 lost_frames;
    uint8 processing_ms;
    uint16 boot_id;
    uint32 received_ms;
} vision_link_snapshot_t;

typedef struct
{
    uint8 session_active;
    uint8 link_online;
    uint8 measurement_valid;
    uint16 boot_id;
    uint16 last_sequence;
    uint32 link_age_ms;
    uint32 measurement_age_ms;
    uint32 received_bytes;
    uint32 crc_ok_frames;
    uint32 accepted_frames;
    uint32 valid_measurement_frames;
    uint32 crc_errors;
    uint32 header_errors;
    uint32 semantic_errors;
    uint32 duplicate_frames;
    uint32 backward_frames;
    uint32 sequence_gap_frames;
    uint32 boot_changes;
    uint32 resync_dropped_bytes;
    uint32 uart_rx_overflows;
} vision_link_status_t;

void vision_link_init(void);
void vision_link_process(void);
uint8 vision_link_get_latest_snapshot(vision_link_snapshot_t *snapshot);
uint8 vision_link_get_valid_measurement(vision_link_snapshot_t *snapshot);
uint8 vision_link_take_new_valid_measurement(vision_link_snapshot_t *snapshot);
void vision_link_get_status(vision_link_status_t *status);

#endif
