#include "decoder.h"

static enqueue_frame_cb_t s_enqueue_cb = NULL;
static print_frame_cb_t s_print_cb = NULL;

void decoder_init(enqueue_frame_cb_t enqueue_cb, print_frame_cb_t print_cb) {
    s_enqueue_cb = enqueue_cb;
    s_print_cb = print_cb;
}

uint8_t classify_pulse(uint32_t us) {
    if (us < MIN_VALID_US)       return PC_GLITCH;
    if (us > MAX_VALID_US)       return PC_IDLE_GAP;
    if (us > MERGE_THRESHOLD_US) return PC_MERGED;
    if (us < THRESHOLD_US)       return PC_LOGIC_0;
    return PC_LOGIC_1;
}

void reset_decoder(struct DecoderContext *ctx_ptr) {
    ctx_ptr->state           = DS_HUNT_SYNC;
    ctx_ptr->sync_count      = 0;
    ctx_ptr->bit_count       = 0;
    ctx_ptr->byte_count      = 0;
    ctx_ptr->separator_count = 0;
    ctx_ptr->frame_errors    = 0;
    ctx_ptr->bytes_this_frame = 0;
}

static void feed_bit(struct DecoderContext *ctx_ptr, uint8_t pc) {
    switch (ctx_ptr->state) {
        case DS_HUNT_SYNC:
            if (pc == PC_LOGIC_1) {
                ctx_ptr->sync_count++;
                if (ctx_ptr->sync_count >= SYNC_ONES_NEEDED) {
                    ctx_ptr->sync_count       = 0;
                    ctx_ptr->byte_count       = 0;
                    ctx_ptr->bit_count        = 0;
                    ctx_ptr->separator_count  = 0;
                    ctx_ptr->frame_errors     = 0;
                    ctx_ptr->bytes_this_frame = 0;
                    ctx_ptr->state            = DS_AWAIT_START;
                }
            } else {
                ctx_ptr->sync_count = 0;
            }
            break;

        case DS_AWAIT_START:
            if (pc == PC_LOGIC_0) {
                ctx_ptr->current_byte    = 0;
                ctx_ptr->bit_count       = 0;
                ctx_ptr->separator_count = 0; 
                ctx_ptr->state           = DS_READ_BITS;
            } else {
                ctx_ptr->separator_count++;
                if (ctx_ptr->separator_count > MAX_SEPARATORS) {
                    reset_decoder(ctx_ptr);
                }
            }
            break;

        case DS_READ_BITS: {
            uint8_t bit_val  = (pc == PC_LOGIC_1) ? 1u : 0u;
            ctx_ptr->current_byte = (uint8_t)((ctx_ptr->current_byte << 1) | bit_val);
            ctx_ptr->bit_count++;

            if (ctx_ptr->bit_count == 8) {
                ctx_ptr->frame[ctx_ptr->byte_count] = ctx_ptr->current_byte;
                ctx_ptr->bytes_this_frame++;
                ctx_ptr->bit_count = 0;
                ctx_ptr->byte_count++;
                
                if (ctx_ptr->byte_count >= PAYLOAD_BYTES) {
                    ctx_ptr->frames_decoded++;
                    
                    if (s_print_cb) {
                        s_print_cb(ctx_ptr->frames_decoded, ctx_ptr->frame);
                    }
                    if (s_enqueue_cb) {
                        s_enqueue_cb(ctx_ptr->frame, PAYLOAD_BYTES);
                    }
                    
                    reset_decoder(ctx_ptr);
                } else {
                    ctx_ptr->separator_count = 0;
                    ctx_ptr->state = DS_AWAIT_START;
                }
            }
            break;
        }
        default:
            reset_decoder(ctx_ptr);
            break;
    }
}

void process_pulse(struct DecoderContext *ctx_ptr, uint32_t pulse_us) {
    uint8_t pc = classify_pulse(pulse_us);
    if (pc == PC_GLITCH) return;

    if (pc == PC_IDLE_GAP) {
        reset_decoder(ctx_ptr);
        return;
    }

    if (pc == PC_MERGED) {
        uint32_t hidden_est  = pulse_us - LOGIC1_PULSE_US;
        uint8_t  hidden_bit  = (hidden_est >= THRESHOLD_US) ? PC_LOGIC_1 : PC_LOGIC_0;
        feed_bit(ctx_ptr, hidden_bit);
        feed_bit(ctx_ptr, PC_LOGIC_1); 
        return;
    }

    feed_bit(ctx_ptr, pc);
}
