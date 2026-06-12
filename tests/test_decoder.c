#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "decoder.h"

// Simple testing harness definitions
static int tests_run = 0;
static int tests_failed = 0;

#define RUN_TEST(test) do { \
    printf("Running %s...\n", #test); \
    tests_run++; \
    int failed_before = tests_failed; \
    test(); \
    if (tests_failed == failed_before) { \
        printf(" -> %s passed.\n", #test); \
    } else { \
        printf(" -> %s FAILED.\n", #test); \
    } \
} while (0)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        printf("   [FAIL] Line %d: %s (condition: %s)\n", __LINE__, msg, #cond); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_INT_EQ(expected, actual, msg) do { \
    if ((expected) != (actual)) { \
        printf("   [FAIL] Line %d: %s (expected %d, got %d)\n", __LINE__, msg, (int)(expected), (int)(actual)); \
        tests_failed++; \
        return; \
    } \
} while(0)

// Globals to track frame outputs from decoder callbacks
static uint8_t last_enqueued_frame[PAYLOAD_BYTES];
static uint8_t last_enqueued_len = 0;
static int enqueue_count = 0;
static int print_count = 0;

static void mock_enqueue_frame(const uint8_t *frame_data, uint8_t len) {
    memcpy(last_enqueued_frame, frame_data, len);
    last_enqueued_len = len;
    enqueue_count++;
}

static void mock_print_frame(uint32_t frames_decoded, const uint8_t *frame_data) {
    print_count++;
}

// ---------------------------------------------------------------------------
// ── TEST CASES ─────────────────────────────────────────────────────────────
// ---------------------------------------------------------------------------

static void test_ring_buffer(void) {
    struct RingBuffer rb;
    memset(&rb, 0, sizeof(rb));

    uint32_t out = 0;
    // Empty buffer pop should return false
    ASSERT_TRUE(!rb_pop(&rb, &out), "Pop on empty ring buffer should return false");

    // Push and pop single value
    rb_push(&rb, 12345u);
    ASSERT_TRUE(rb_pop(&rb, &out), "Pop on non-empty ring buffer should return true");
    ASSERT_INT_EQ(12345u, out, "Popped value should match pushed value");
    ASSERT_TRUE(!rb_pop(&rb, &out), "Ring buffer should be empty after single pop");

    // Push multiple and verify FIFO order
    rb_push(&rb, 10u);
    rb_push(&rb, 20u);
    rb_push(&rb, 30u);
    
    ASSERT_TRUE(rb_pop(&rb, &out), "Pop 1");
    ASSERT_INT_EQ(10u, out, "Value 1");
    ASSERT_TRUE(rb_pop(&rb, &out), "Pop 2");
    ASSERT_INT_EQ(20u, out, "Value 2");
    ASSERT_TRUE(rb_pop(&rb, &out), "Pop 3");
    ASSERT_INT_EQ(30u, out, "Value 3");
    ASSERT_TRUE(!rb_pop(&rb, &out), "Empty check");

    // Test buffer capacity/limit
    // RB_MASK is 255 (size 256). We can hold at most 255 items before next == tail.
    for (uint32_t i = 0; i < 300; i++) {
        rb_push(&rb, i);
    }
    // Buffer head should have stopped advancing when it hit tail - 1.
    // Let's verify that we can pop elements without infinite loop.
    int count = 0;
    while (rb_pop(&rb, &out)) {
        count++;
    }
    ASSERT_TRUE(count <= 255, "Buffer must drop elements on overflow instead of corrupting pointers");
}

static void test_classify_pulse(void) {
    // MIN_VALID_US = 300
    // THRESHOLD_US = 2639
    // MERGE_THRESHOLD_US = 8000
    // MAX_VALID_US = 13500

    ASSERT_INT_EQ(PC_GLITCH, classify_pulse(100), "Less than MIN_VALID_US is glitch");
    ASSERT_INT_EQ(PC_LOGIC_0, classify_pulse(1000), "Typical logical 0 (1.11ms) is logic 0");
    ASSERT_INT_EQ(PC_LOGIC_1, classify_pulse(4000), "Typical logical 1 (4.16ms) is logic 1");
    ASSERT_INT_EQ(PC_MERGED, classify_pulse(10000), "Between MERGE_THRESHOLD_US and MAX_VALID_US is merged");
    ASSERT_INT_EQ(PC_IDLE_GAP, classify_pulse(15000), "Greater than MAX_VALID_US is idle gap");
}

static void test_reset_decoder(void) {
    struct DecoderContext ctx;
    ctx.state = DS_READ_BITS;
    ctx.sync_count = 5;
    ctx.bit_count = 3;
    ctx.current_byte = 0xAA;
    ctx.byte_count = 10;
    ctx.separator_count = 2;
    ctx.frame_errors = 4;
    ctx.frames_decoded = 42; // Frames decoded should NOT be reset

    reset_decoder(&ctx);

    ASSERT_INT_EQ(DS_HUNT_SYNC, ctx.state, "Reset state should be DS_HUNT_SYNC");
    ASSERT_INT_EQ(0, ctx.sync_count, "sync_count reset");
    ASSERT_INT_EQ(0, ctx.bit_count, "bit_count reset");
    ASSERT_INT_EQ(0, ctx.byte_count, "byte_count reset");
    ASSERT_INT_EQ(0, ctx.separator_count, "separator_count reset");
    ASSERT_INT_EQ(0, ctx.frame_errors, "frame_errors reset");
    ASSERT_INT_EQ(42, ctx.frames_decoded, "frames_decoded should persist across reset");
}

static void test_sync_hunting(void) {
    struct DecoderContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.state = DS_HUNT_SYNC;

    // Feed logical 0, should stay in HUNT
    process_pulse(&ctx, 1111);
    ASSERT_INT_EQ(DS_HUNT_SYNC, ctx.state, "Logical 0 does not trigger sync");

    // Feed 7 logical 1s, should stay in HUNT
    for (int i = 0; i < 7; i++) {
        process_pulse(&ctx, 4167);
    }
    ASSERT_INT_EQ(DS_HUNT_SYNC, ctx.state, "7 logic 1s are not enough for sync");

    // Feed 8th logical 1, should change state to DS_AWAIT_START
    process_pulse(&ctx, 4167);
    ASSERT_INT_EQ(DS_AWAIT_START, ctx.state, "8 logic 1s triggers transition to DS_AWAIT_START");
}

// Helper to simulate feeding a bit (0 or 1) to the decoder
static void feed_simulated_bit(struct DecoderContext *ctx, int bit) {
    if (bit == 0) {
        // Send a logic 0 pulse
        process_pulse(ctx, 1111);
    } else {
        // Send a logic 1 pulse
        process_pulse(ctx, 4167);
    }
}

// Helper to send a start bit (0)
static void feed_start_bit(struct DecoderContext *ctx) {
    feed_simulated_bit(ctx, 0);
}

// Helper to send a full byte: start bit + 8 bits
static void feed_byte(struct DecoderContext *ctx, uint8_t byte_val) {
    // 1. Send start bit (0)
    feed_start_bit(ctx);

    // 2. Send 8 data bits (MSB first)
    for (int i = 7; i >= 0; i--) {
        int bit = (byte_val >> i) & 1;
        feed_simulated_bit(ctx, bit);
    }
}

static void test_decode_frame(void) {
    struct DecoderContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.state = DS_HUNT_SYNC;
    
    enqueue_count = 0;
    print_count = 0;
    memset(last_enqueued_frame, 0, sizeof(last_enqueued_frame));

    // 1. Sync
    for (int i = 0; i < 8; i++) {
        process_pulse(&ctx, 4167);
    }
    ASSERT_INT_EQ(DS_AWAIT_START, ctx.state, "Sync lock established");

    // 2. Feed 25 distinct bytes (e.g. 0x01, 0x02, ..., 0x19)
    for (uint8_t val = 1; val <= 25; val++) {
        feed_byte(&ctx, val);
    }

    // 3. Verify frame enqueued and callbacks triggered
    ASSERT_INT_EQ(1, enqueue_count, "One frame should be enqueued");
    ASSERT_INT_EQ(1, print_count, "One frame should be printed");
    ASSERT_INT_EQ(PAYLOAD_BYTES, last_enqueued_len, "Payload size should be 25 bytes");

    // 4. Verify contents
    for (uint8_t i = 0; i < 25; i++) {
        ASSERT_INT_EQ(i + 1, last_enqueued_frame[i], "Decoded data byte mismatch");
    }

    // 5. Decoder should have reset back to HUNT state
    ASSERT_INT_EQ(DS_HUNT_SYNC, ctx.state, "Decoder should reset back to DS_HUNT_SYNC after full frame");
}

static void test_merged_pulse(void) {
    struct DecoderContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.state = DS_HUNT_SYNC;

    enqueue_count = 0;
    
    // 1. Sync
    for (int i = 0; i < 8; i++) {
        process_pulse(&ctx, 4167);
    }

    // 2. Feed first byte up to bit 6
    feed_start_bit(&ctx);
    // Send 6 logical 0 bits
    for (int i = 0; i < 6; i++) {
        feed_simulated_bit(&ctx, 0);
    }
    
    // At this point: bit_count = 6
    // We want the next pulses to represent bit 7 and bit 8.
    // Instead of two separate pulses, we feed a merged pulse representing:
    // a logical 1 (4167us) followed directly by a logical 1 (4167us) without a transition.
    // Total merged pulse duration: 4167 + 4167 = 8334us.
    // Let's pass 9000us which should classify as PC_MERGED (> 8000us).
    // Hidden estimation: us - LOGIC1_PULSE_US = 9000 - 4167 = 4833us.
    // 4833us >= THRESHOLD_US (2639) -> classifies as hidden logical 1, followed by logical 1.
    process_pulse(&ctx, 9000);

    // This single process_pulse should have pushed two bits (1 then 1), 
    // completing the 8 data bits of the first byte!
    ASSERT_INT_EQ(0, ctx.bit_count, "Byte should be completed by the merged pulse");
    ASSERT_INT_EQ(1, ctx.byte_count, "Byte count should increment to 1");
    
    // The byte assembled should be: start (0), then six 0s, then two 1s.
    // Value = 0b00000011 = 0x03.
    ASSERT_INT_EQ(0x03, ctx.frame[0], "Merged pulse decoded byte mismatch");
}

// ---------------------------------------------------------------------------
// ── MAIN ENTRY ─────────────────────────────────────────────────────────────
// ---------------------------------------------------------------------------

int main(void) {
    printf("==========================================\n");
    printf(" Starting ALDL Host Decoder Unit Tests   \n");
    printf("==========================================\n");

    // Bind callbacks
    decoder_init(mock_enqueue_frame, mock_print_frame);

    RUN_TEST(test_ring_buffer);
    RUN_TEST(test_classify_pulse);
    RUN_TEST(test_reset_decoder);
    RUN_TEST(test_sync_hunting);
    RUN_TEST(test_decode_frame);
    RUN_TEST(test_merged_pulse);

    printf("\n==========================================\n");
    printf(" Test Summary: %d run, %d failed.\n", tests_run, tests_failed);
    printf("==========================================\n");

    return (tests_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
