#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#define ALDL_PIN             GPIO_NUM_4

#define LOGIC0_PULSE_US      1111u
#define LOGIC1_PULSE_US      4167u
#define THRESHOLD_US         2639u
#define MIN_VALID_US          300u
#define MERGE_THRESHOLD_US   8000u
#define MAX_VALID_US        13500u
#define MAX_SEPARATORS        12u
#define SYNC_ONES_NEEDED      8u
#define PAYLOAD_BYTES        25u

#define BT_DEVICE_NAME       "ESP32-ALDL"
#define BT_QUEUE_DEPTH        4u

#define PC_GLITCH   ((uint8_t)0)
#define PC_LOGIC_0  ((uint8_t)1)
#define PC_LOGIC_1  ((uint8_t)2)
#define PC_IDLE_GAP ((uint8_t)3)
#define PC_MERGED   ((uint8_t)4)

#define DS_HUNT_SYNC   ((uint8_t)0)
#define DS_AWAIT_START ((uint8_t)1)
#define DS_READ_BITS   ((uint8_t)2)

static const char *TAG = "ALDL";

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

static struct RingBuffer      rb;
static struct DecoderContext  ctx;
static QueueHandle_t          bt_queue = NULL;

static uint32_t spp_handle = 0;
static bool     bt_connected = false;

#define RB_MASK ((uint16_t)255u)

static inline void IRAM_ATTR rb_push(uint32_t v) {
    uint16_t next = (rb.head + 1u) & RB_MASK;
    if (next == rb.tail) return;
    rb.data[rb.head] = v;
    __asm__ __volatile__("" ::: "memory");
    rb.head = next;
}

static inline bool rb_pop(uint32_t *out) {
    if (rb.tail == rb.head) return false;
    *out = rb.data[rb.tail];
    __asm__ __volatile__("" ::: "memory");
    rb.tail = (rb.tail + 1u) & RB_MASK;
    return true;
}

static volatile uint64_t isr_fall_us = 0;

static void IRAM_ATTR aldl_gpio_isr(void* arg) {
    uint64_t now = (uint64_t)esp_timer_get_time();
    if (gpio_get_level((gpio_num_t)ALDL_PIN) == 0) {
        isr_fall_us = now;
    } else {
        if (isr_fall_us != 0) {
            rb_push((uint32_t)(now - isr_fall_us));
            isr_fall_us = 0;
        }
    }
}

static uint8_t classify_pulse(uint32_t us) {
    if (us < MIN_VALID_US)       return PC_GLITCH;
    if (us > MAX_VALID_US)       return PC_IDLE_GAP;
    if (us > MERGE_THRESHOLD_US) return PC_MERGED;
    if (us < THRESHOLD_US)       return PC_LOGIC_0;
    return PC_LOGIC_1;
}

static void reset_decoder(void) {
    ctx.state           = DS_HUNT_SYNC;
    ctx.sync_count      = 0;
    ctx.bit_count       = 0;
    ctx.byte_count      = 0;
    ctx.separator_count = 0;
    ctx.frame_errors    = 0;
    ctx.bytes_this_frame = 0;
}

static void enqueue_frame(void) {
    struct BtFrame f;
    memcpy(f.data, ctx.frame, PAYLOAD_BYTES);
    f.len = PAYLOAD_BYTES;
    if (xQueueSend(bt_queue, &f, 0) != pdTRUE) {
        ESP_LOGW(TAG, "BT queue full");
    }
}

static void print_frame(void) {
    char hex_str[ PAYLOAD_BYTES * 3 + 1 ];
    int offset = 0;
    for (uint8_t i = 0; i < PAYLOAD_BYTES; i++) {
        offset += sprintf(hex_str + offset, "%02X ", ctx.frame[i]);
    }
    ESP_LOGI(TAG, "[FRAME #%lu] %s", ctx.frames_decoded, hex_str);
}

static void feed_bit(uint8_t pc) {
    switch (ctx.state) {
        case DS_HUNT_SYNC:
            if (pc == PC_LOGIC_1) {
                ctx.sync_count++;
                if (ctx.sync_count >= SYNC_ONES_NEEDED) {
                    ctx.sync_count       = 0;
                    ctx.byte_count       = 0;
                    ctx.bit_count        = 0;
                    ctx.separator_count  = 0;
                    ctx.frame_errors     = 0;
                    ctx.bytes_this_frame = 0;
                    ctx.state            = DS_AWAIT_START;
                }
            } else {
                ctx.sync_count = 0;
            }
            break;

        case DS_AWAIT_START:
            if (pc == PC_LOGIC_0) {
                ctx.current_byte    = 0;
                ctx.bit_count       = 0;
                ctx.separator_count = 0; 
                ctx.state           = DS_READ_BITS;
            } else {
                ctx.separator_count++;
                if (ctx.separator_count > MAX_SEPARATORS) {
                    reset_decoder();
                }
            }
            break;

        case DS_READ_BITS: {
            uint8_t bit_val  = (pc == PC_LOGIC_1) ? 1u : 0u;
            ctx.current_byte = (uint8_t)((ctx.current_byte << 1) | bit_val);
            ctx.bit_count++;

            if (ctx.bit_count == 8) {
                ctx.frame[ctx.byte_count] = ctx.current_byte;
                ctx.bytes_this_frame++;
                ctx.bit_count = 0;
                ctx.byte_count++;
                
                if (ctx.byte_count >= PAYLOAD_BYTES) {
                    ctx.frames_decoded++;
                    print_frame();
                    enqueue_frame();
                    reset_decoder();
                } else {
                    ctx.separator_count = 0;
                    ctx.state = DS_AWAIT_START;
                }
            }
            break;
        }
        default:
            reset_decoder();
            break;
    }
}

static void process_pulse(uint32_t pulse_us) {
    uint8_t pc = classify_pulse(pulse_us);
    if (pc == PC_GLITCH) return;

    if (pc == PC_IDLE_GAP) {
        reset_decoder();
        return;
    }

    if (pc == PC_MERGED) {
        uint32_t hidden_est  = pulse_us - LOGIC1_PULSE_US;
        uint8_t  hidden_bit  = (hidden_est >= THRESHOLD_US) ? PC_LOGIC_1 : PC_LOGIC_0;
        feed_bit(hidden_bit);
        feed_bit(PC_LOGIC_1); 
        return;
    }

    feed_bit(pc);
}

static void btTransmitTask(void* pvParameters) {
    struct BtFrame f;
    
    // The 2-byte hard-sync header that ALDLDroid will lock onto
    uint8_t tx_buffer[PAYLOAD_BYTES + 2];
    tx_buffer[0] = 0xAA; 
    tx_buffer[1] = 0x55;
    
    for (;;) {
        if (xQueueReceive(bt_queue, &f, portMAX_DELAY) == pdTRUE) {
            if (bt_connected && spp_handle != 0) {
                // Copy the 25 decoded bytes immediately after the header
                memcpy(&tx_buffer[2], f.data, f.len);
                // Transmit the 27-byte locked packet
                esp_spp_write(spp_handle, f.len + 2, tx_buffer);
            }
        }
    }
}

static void aldlDecodeTask(void* pvParameters) {
    uint32_t pulse_us = 0;
    for (;;) {
        bool did_work = false;
        while (rb_pop(&pulse_us)) {
            process_pulse(pulse_us);
            did_work = true;
        }
        if (!did_work) vTaskDelay(1);
    }
}

static void statusTask(void* pvParameters) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "[STATUS] frames=%lu bt=%s",
                 ctx.frames_decoded,
                 bt_connected ? "UP" : "waiting");
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
        spp_handle = 0;
        bt_connected = false;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        esp_bt_dev_set_device_name(BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d, handle=%lu",
                 param->data_ind.len, (unsigned long)param->data_ind.handle);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        spp_handle = param->srv_open.handle;
        bt_connected = true;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
        break;

    default:
        break;
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    if ((ret = esp_spp_enhanced_init(&spp_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " ESP32 ALDL Bridge — GM 1227170 Fiero 2.8  ");
    ESP_LOGI(TAG, " 160-baud PWM — AA55 Hard Sync Active       ");
    ESP_LOGI(TAG, "============================================");

    memset(&rb,  0, sizeof(rb));
    memset(&ctx, 0, sizeof(ctx));
    ctx.state = DS_HUNT_SYNC;
    
    gpio_config_t io = {};
    io.intr_type    = GPIO_INTR_ANYEDGE;
    io.mode         = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << ALDL_PIN);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add((gpio_num_t)ALDL_PIN, aldl_gpio_isr, NULL);

    bt_queue = xQueueCreate(BT_QUEUE_DEPTH, sizeof(struct BtFrame));

    xTaskCreatePinnedToCore(aldlDecodeTask, "aldlDecode", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(btTransmitTask, "btTx",       4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(statusTask,     "status",     2048, NULL, 1, NULL, 0);
}
