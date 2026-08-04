#include "vision_link.h"

#include "control_config.h"
#include "uart3_maix_hw.h"
#include "heartbeat.h"

#define VISION_LINK_MAGIC_0                    (0xA5u)
#define VISION_LINK_MAGIC_1                    (0x5Au)
#define VISION_LINK_VERSION                    (0x01u)
#define VISION_LINK_TYPE                       (0x01u)
#define VISION_LINK_FLAG_MASK                  (0x0Fu)
#define VISION_LINK_INVALID_I16                ((int16)-32768)
#define VISION_LINK_POSITION_LIMIT_DMM         (1300)
#define VISION_LINK_VELOCITY_LIMIT_MM_S        (5000)
#define VISION_LINK_AGE_INVALID                (0xFFFFFFFFu)

static uint8 vision_link_buffer[VISION_LINK_FRAME_SIZE];
static uint8 vision_link_buffer_count;

static vision_link_snapshot_t vision_link_latest;
static vision_link_snapshot_t vision_link_measurement;
static volatile uint32 vision_link_latest_generation;
static volatile uint32 vision_link_measurement_generation;
static uint32 vision_link_measurement_taken_generation;

static uint8 vision_link_has_session;
static uint8 vision_link_has_crc_frame;
static uint8 vision_link_has_measurement;
static uint16 vision_link_boot_id;
static uint16 vision_link_last_sequence;
static uint32 vision_link_last_crc_ms;
static uint32 vision_link_last_measurement_ms;

static uint32 vision_link_crc_ok_frames;
static uint32 vision_link_accepted_frames;
static uint32 vision_link_valid_measurement_frames;
static uint32 vision_link_crc_errors;
static uint32 vision_link_header_errors;
static uint32 vision_link_semantic_errors;
static uint32 vision_link_duplicate_frames;
static uint32 vision_link_backward_frames;
static uint32 vision_link_sequence_gap_frames;
static uint32 vision_link_boot_changes;
static uint32 vision_link_resync_dropped_bytes;
static float vision_link_position_offset_m =
    BALANCE_VISION_POSITION_OFFSET_M;

static uint16 vision_link_read_u16_le(const uint8 *data)
{
    return (uint16)((uint16)data[0] | ((uint16)data[1] << 8u));
}

static uint32 vision_link_read_u32_le(const uint8 *data)
{
    return (uint32)data[0] |
        ((uint32)data[1] << 8u) |
        ((uint32)data[2] << 16u) |
        ((uint32)data[3] << 24u);
}

static uint16 vision_link_crc16(const uint8 *data, uint8 length)
{
    uint16 crc = 0xFFFFu;
    uint8 index;
    uint8 bit;

    for (index = 0u; index < length; index++)
    {
        crc ^= (uint16)data[index] << 8u;
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (0u != (crc & 0x8000u)) ?
                (uint16)((crc << 1u) ^ 0x1021u) : (uint16)(crc << 1u);
        }
    }
    return crc;
}

static void vision_link_publish(
    vision_link_snapshot_t *destination,
    volatile uint32 *generation,
    const vision_link_snapshot_t *source)
{
    (*generation)++;
    *destination = *source;
    (*generation)++;
}

static uint8 vision_link_copy_atomic(
    const vision_link_snapshot_t *source,
    const volatile uint32 *generation,
    vision_link_snapshot_t *destination)
{
    uint32 before;
    uint32 after;

    if (NULL == destination)
    {
        return 0u;
    }
    do
    {
        before = *generation;
        if (0u != (before & 1u))
        {
            continue;
        }
        *destination = *source;
        after = *generation;
    } while ((before != after) || (0u != (after & 1u)));
    return 1u;
}

static uint8 vision_link_header_valid(const uint8 *frame)
{
    return ((VISION_LINK_MAGIC_0 == frame[0]) &&
            (VISION_LINK_MAGIC_1 == frame[1]) &&
            (VISION_LINK_VERSION == frame[2]) &&
            (VISION_LINK_TYPE == frame[3]) &&
            (VISION_LINK_FRAME_SIZE == frame[4])) ? 1u : 0u;
}

static uint8 vision_link_payload_valid(
    const uint8 *frame,
    int16 position_dmm,
    int16 velocity_mm_s)
{
    uint8 flags = frame[5];
    uint8 measured = flags & VISION_LINK_FLAG_MEASURED_VALID;
    uint8 predicted = flags & VISION_LINK_FLAG_PREDICT_ONLY;
    uint8 has_numeric_state = (uint8)(measured | predicted);

    if ((0u != (flags & (uint8)(~VISION_LINK_FLAG_MASK))) ||
        ((0u != measured) && (0u != predicted)) ||
        (frame[16] > 100u) || (0u != frame[19]))
    {
        return 0u;
    }
    if (0u != has_numeric_state)
    {
        if ((VISION_LINK_INVALID_I16 == position_dmm) ||
            (VISION_LINK_INVALID_I16 == velocity_mm_s) ||
            (position_dmm < -VISION_LINK_POSITION_LIMIT_DMM) ||
            (position_dmm > VISION_LINK_POSITION_LIMIT_DMM) ||
            (velocity_mm_s < -VISION_LINK_VELOCITY_LIMIT_MM_S) ||
            (velocity_mm_s > VISION_LINK_VELOCITY_LIMIT_MM_S))
        {
            return 0u;
        }
    }
    else if ((VISION_LINK_INVALID_I16 != position_dmm) ||
             (VISION_LINK_INVALID_I16 != velocity_mm_s))
    {
        return 0u;
    }
    return 1u;
}

static void vision_link_resync_rejected(void)
{
    uint8 index;
    uint8 keep_from = VISION_LINK_FRAME_SIZE;
    uint8 keep_count;

    for (index = 1u; index < (VISION_LINK_FRAME_SIZE - 1u); index++)
    {
        if ((VISION_LINK_MAGIC_0 == vision_link_buffer[index]) &&
            (VISION_LINK_MAGIC_1 == vision_link_buffer[index + 1u]))
        {
            keep_from = index;
            break;
        }
    }
    if ((VISION_LINK_FRAME_SIZE == keep_from) &&
        (VISION_LINK_MAGIC_0 == vision_link_buffer[VISION_LINK_FRAME_SIZE - 1u]))
    {
        keep_from = VISION_LINK_FRAME_SIZE - 1u;
    }

    keep_count = (uint8)(VISION_LINK_FRAME_SIZE - keep_from);
    if (keep_count > 0u)
    {
        for (index = 0u; index < keep_count; index++)
        {
            vision_link_buffer[index] = vision_link_buffer[keep_from + index];
        }
    }
    vision_link_resync_dropped_bytes += keep_from;
    vision_link_buffer_count = keep_count;
}

static void vision_link_accept_frame(uint32 now_ms)
{
    vision_link_snapshot_t snapshot;
    uint16 sequence = vision_link_read_u16_le(&vision_link_buffer[6]);
    uint16 boot_id = vision_link_read_u16_le(&vision_link_buffer[20]);
    uint16 delta;
    int16 position_dmm = (int16)vision_link_read_u16_le(&vision_link_buffer[12]);
    int16 velocity_mm_s = (int16)vision_link_read_u16_le(&vision_link_buffer[14]);

    if (0u == vision_link_payload_valid(
            vision_link_buffer, position_dmm, velocity_mm_s))
    {
        vision_link_semantic_errors++;
        return;
    }

    if ((0u == vision_link_has_session) || (boot_id != vision_link_boot_id))
    {
        if (0u != vision_link_has_session)
        {
            vision_link_boot_changes++;
        }
        vision_link_has_session = 1u;
        vision_link_boot_id = boot_id;
        vision_link_last_sequence = sequence;
        vision_link_has_measurement = 0u;
        vision_link_measurement_generation = 0u;
        vision_link_measurement_taken_generation = 0u;
    }
    else
    {
        delta = (uint16)(sequence - vision_link_last_sequence);
        if (0u == delta)
        {
            vision_link_duplicate_frames++;
            return;
        }
        if (delta >= 0x8000u)
        {
            vision_link_backward_frames++;
            return;
        }
        if (delta > 1u)
        {
            vision_link_sequence_gap_frames += (uint32)(delta - 1u);
        }
        vision_link_last_sequence = sequence;
    }

    snapshot.flags = vision_link_buffer[5];
    snapshot.sequence = sequence;
    snapshot.capture_ms = vision_link_read_u32_le(&vision_link_buffer[8]);
    snapshot.position_dmm = position_dmm;
    snapshot.velocity_mm_s = velocity_mm_s;
    snapshot.confidence = vision_link_buffer[16];
    snapshot.lost_frames = vision_link_buffer[17];
    snapshot.processing_ms = vision_link_buffer[18];
    snapshot.boot_id = boot_id;
    snapshot.received_ms = now_ms;
    vision_link_publish(
        &vision_link_latest, &vision_link_latest_generation, &snapshot);
    vision_link_accepted_frames++;

    if (0u != (snapshot.flags & VISION_LINK_FLAG_MEASURED_VALID))
    {
        vision_link_publish(
            &vision_link_measurement,
            &vision_link_measurement_generation,
            &snapshot);
        vision_link_has_measurement = 1u;
        vision_link_last_measurement_ms = now_ms;
        vision_link_valid_measurement_frames++;
    }
}

static void vision_link_consume_byte(uint8 byte, uint32 now_ms)
{
    uint16 received_crc;
    uint16 calculated_crc;

    if (0u == vision_link_buffer_count)
    {
        if (VISION_LINK_MAGIC_0 == byte)
        {
            vision_link_buffer[0] = byte;
            vision_link_buffer_count = 1u;
        }
        else
        {
            vision_link_resync_dropped_bytes++;
        }
        return;
    }
    if (1u == vision_link_buffer_count)
    {
        if (VISION_LINK_MAGIC_1 == byte)
        {
            vision_link_buffer[1] = byte;
            vision_link_buffer_count = 2u;
        }
        else if (VISION_LINK_MAGIC_0 != byte)
        {
            vision_link_buffer_count = 0u;
            vision_link_resync_dropped_bytes += 2u;
        }
        else
        {
            vision_link_resync_dropped_bytes++;
        }
        return;
    }

    vision_link_buffer[vision_link_buffer_count++] = byte;
    if (VISION_LINK_FRAME_SIZE != vision_link_buffer_count)
    {
        return;
    }
    if (0u == vision_link_header_valid(vision_link_buffer))
    {
        vision_link_header_errors++;
        vision_link_resync_rejected();
        return;
    }
    received_crc = vision_link_read_u16_le(&vision_link_buffer[22]);
    calculated_crc = vision_link_crc16(vision_link_buffer, 22u);
    if (received_crc != calculated_crc)
    {
        vision_link_crc_errors++;
        vision_link_resync_rejected();
        return;
    }

    vision_link_crc_ok_frames++;
    vision_link_has_crc_frame = 1u;
    vision_link_last_crc_ms = now_ms;
    vision_link_accept_frame(now_ms);
    vision_link_buffer_count = 0u;
}

void vision_link_init(void)
{
    vision_link_buffer_count = 0u;
    vision_link_latest_generation = 0u;
    vision_link_measurement_generation = 0u;
    vision_link_measurement_taken_generation = 0u;
    vision_link_has_session = 0u;
    vision_link_has_crc_frame = 0u;
    vision_link_has_measurement = 0u;
    vision_link_boot_id = 0u;
    vision_link_last_sequence = 0u;
    vision_link_last_crc_ms = 0u;
    vision_link_last_measurement_ms = 0u;
    vision_link_crc_ok_frames = 0u;
    vision_link_accepted_frames = 0u;
    vision_link_valid_measurement_frames = 0u;
    vision_link_crc_errors = 0u;
    vision_link_header_errors = 0u;
    vision_link_semantic_errors = 0u;
    vision_link_duplicate_frames = 0u;
    vision_link_backward_frames = 0u;
    vision_link_sequence_gap_frames = 0u;
    vision_link_boot_changes = 0u;
    vision_link_resync_dropped_bytes = 0u;
}

void vision_link_process(void)
{
    uint8 byte;
    uint32 now_ms = heartbeat_get_ms();

    while (0u != uart3_maix_hw_read_byte(&byte))
    {
        vision_link_consume_byte(byte, now_ms);
    }
}

uint8 vision_link_get_latest_snapshot(vision_link_snapshot_t *snapshot)
{
    if (0u == vision_link_has_session)
    {
        return 0u;
    }
    return vision_link_copy_atomic(
        &vision_link_latest, &vision_link_latest_generation, snapshot);
}

uint8 vision_link_get_valid_measurement(vision_link_snapshot_t *snapshot)
{
    uint32 now_ms = heartbeat_get_ms();

    if ((0u == vision_link_has_measurement) ||
        ((now_ms - vision_link_last_measurement_ms) >
         VISION_LINK_MEASUREMENT_TIMEOUT_MS))
    {
        return 0u;
    }
    return vision_link_copy_atomic(
        &vision_link_measurement,
        &vision_link_measurement_generation,
        snapshot);
}

uint8 vision_link_take_new_valid_measurement(vision_link_snapshot_t *snapshot)
{
    uint32 generation_before;
    uint32 generation_after;

    do
    {
        generation_before = vision_link_measurement_generation;
        if ((generation_before == vision_link_measurement_taken_generation) ||
            (0u != (generation_before & 1u)) ||
            (0u == vision_link_get_valid_measurement(snapshot)))
        {
            return 0u;
        }
        generation_after = vision_link_measurement_generation;
    } while (generation_before != generation_after);
    vision_link_measurement_taken_generation = generation_after;
    return 1u;
}

void vision_link_set_position_offset_m(float offset_m)
{
    if (offset_m > VISION_LINK_POSITION_OFFSET_LIMIT_M)
    {
        offset_m = VISION_LINK_POSITION_OFFSET_LIMIT_M;
    }
    else if (offset_m < -VISION_LINK_POSITION_OFFSET_LIMIT_M)
    {
        offset_m = -VISION_LINK_POSITION_OFFSET_LIMIT_M;
    }
    vision_link_position_offset_m = offset_m;
}

float vision_link_get_position_offset_m(void)
{
    return vision_link_position_offset_m;
}

float vision_link_correct_position_m(int16 position_dmm)
{
    return (float)position_dmm * 0.0001f +
        vision_link_position_offset_m;
}

void vision_link_get_status(vision_link_status_t *status)
{
    uint32 now_ms;

    if (NULL == status)
    {
        return;
    }
    now_ms = heartbeat_get_ms();
    status->session_active = vision_link_has_session;
    status->link_age_ms = (0u != vision_link_has_crc_frame) ?
        (now_ms - vision_link_last_crc_ms) : VISION_LINK_AGE_INVALID;
    status->measurement_age_ms = (0u != vision_link_has_measurement) ?
        (now_ms - vision_link_last_measurement_ms) : VISION_LINK_AGE_INVALID;
    status->link_online = ((0u != vision_link_has_crc_frame) &&
        (status->link_age_ms <= VISION_LINK_OFFLINE_TIMEOUT_MS)) ? 1u : 0u;
    status->measurement_valid = ((0u != vision_link_has_measurement) &&
        (status->measurement_age_ms <= VISION_LINK_MEASUREMENT_TIMEOUT_MS)) ?
        1u : 0u;
    status->boot_id = vision_link_boot_id;
    status->last_sequence = vision_link_last_sequence;
    status->received_bytes = uart3_maix_hw_get_rx_count();
    status->crc_ok_frames = vision_link_crc_ok_frames;
    status->accepted_frames = vision_link_accepted_frames;
    status->valid_measurement_frames = vision_link_valid_measurement_frames;
    status->crc_errors = vision_link_crc_errors;
    status->header_errors = vision_link_header_errors;
    status->semantic_errors = vision_link_semantic_errors;
    status->duplicate_frames = vision_link_duplicate_frames;
    status->backward_frames = vision_link_backward_frames;
    status->sequence_gap_frames = vision_link_sequence_gap_frames;
    status->boot_changes = vision_link_boot_changes;
    status->resync_dropped_bytes = vision_link_resync_dropped_bytes;
    status->uart_rx_overflows = uart3_maix_hw_get_rx_overflow_count();
}
