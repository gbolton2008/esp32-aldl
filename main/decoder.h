#ifndef DECODER_H
#define DECODER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LOGIC0_PULSE_US      1111u
#define LOGIC1_PULSE_US      4167u
#define THRESHOLD_US         2639u
#define MIN_VALID_US          300u
#define MERGE_THRESHOLD_US   8000u
#define MAX_VALID_US        13500u
#define MAX_SEPARATORS        12u
#define SYNC_ONES_NEEDED      8u
#define PAYLOAD_BYTES        25u

#define PC_GLITCH   ((uint8_t)0)
#define PC_LOGIC_0  ((uint8_t)1)
#define PC_LOGIC_1  ((uint8_t)2)
#define PC_IDLE_GAP ((uint8_t)3)
#define PC_MERGED   ((uint8_t)4)

#define DS_HUNT_SYNC   ((uint8_t)0)
#define DS_AWAIT_START ((uint8_t)1)
#define DS_READ_BITS   ((uint8_t)2)

struct BtFrame {
    uint8_t data[PAYLOAD_BYTES];
    uint8_t len;
};

struct DecoderContext {
    uint8_t  state;
    uint8_t  sync_count;
    uint8_t  bit_count;
    uint8_t  current_byte;
    uint8_t  byte_count;
    uint8_t  separator_count;
    uint8_t  frame[PAYLOAD_BYTES];
    uint32_t frame_errors;
    uint32_t frames_decoded;
    uint32_t bytes_this_frame;
};

struct RingBuffer {
    volatile uint32_t data[256];
    volatile uint16_t head;
    volatile uint16_t tail;
};

#define RB_MASK ((uint16_t)255u)

// Callbacks for hardware bridging (e.g. FreeRTOS queueing, hardware logging)
typedef void (*enqueue_frame_cb_t)(const uint8_t *frame_data, uint8_t len);
typedef void (*print_frame_cb_t)(uint32_t frames_decoded, const uint8_t *frame_data);

// Initialize decoder callbacks
void decoder_init(enqueue_frame_cb_t enqueue_cb, print_frame_cb_t print_cb);

// Core decoding interface
uint8_t classify_pulse(uint32_t us);
void reset_decoder(struct DecoderContext *ctx_ptr);
void process_pulse(struct DecoderContext *ctx_ptr, uint32_t pulse_us);

// Ring buffer inline helpers
static inline void rb_push(struct RingBuffer *rb_ptr, uint32_t v) {
    uint16_t next = (rb_ptr->head + 1u) & RB_MASK;
    if (next == rb_ptr->tail) return;
    rb_ptr->data[rb_ptr->head] = v;
    __asm__ __volatile__("" ::: "memory");
    rb_ptr->head = next;
}

static inline bool rb_pop(struct RingBuffer *rb_ptr, uint32_t *out) {
    if (rb_ptr->tail == rb_ptr->head) return false;
    *out = rb_ptr->data[rb_ptr->tail];
    __asm__ __volatile__("" ::: "memory");
    rb_ptr->tail = (rb_ptr->tail + 1u) & RB_MASK;
    return true;
}

#endif // DECODER_H
