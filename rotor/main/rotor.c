/**
 * Rotor Controller: control LEDs on rotating POV board
 */

#include <stdio.h>
#include <sys/time.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "font8x8_basic.h"

#define ESP_INTR_FLAG_DEFAULT 0

#define NUM_POV_LEDS 8

// GPIOs 
#define BLINK_GPIO     8
#define ROTATION_GPIO 10
#define POV_LED_GPIO   0

#define MIN_ROUNDS 10
#define STABLE_ROUNDS 100

#define NUM_PIXELS 320

#define USE_TIMER_FOR_PIXEL

// estimated rotation time 32000 us
#define ROTATION_SENSOR_HOLD_OFF_US 10000

#define ROTATION_WEIGHT 8

// current position of rotor 0..(NUM_PIXELS-1)
static volatile uint16_t pixel_pos;

// timer to update LEDs
static gptimer_handle_t gptimer;
static gptimer_alarm_config_t alarm_config;

// number of rounds since start
static volatile uint32_t round_count;

// measures rotation time
static volatile uint32_t rotation_us;

// time per pixel in 10ns
static volatile uint32_t pixel_time_10ns;

// alarm for next pixel in 10ns
static volatile uint32_t next_pixel_10ns;

// current buffer used for LEDs
static uint8_t framebuffer_live[NUM_PIXELS];

// prepared update
static uint8_t framebuffer_next[NUM_PIXELS];

// update was requested
static volatile int16_t update_requested_at_x;


static void update_leds(uint8_t value){
    gpio_set_level(BLINK_GPIO, value & 1);
    uint8_t i;
    for (i=0;i<NUM_POV_LEDS;i++){
        gpio_set_level(POV_LED_GPIO + i, (value & (1<<i)) == 0);
    }
}

static void update_framebuffer(void){
    memcpy(framebuffer_live, framebuffer_next, sizeof(framebuffer_next));
}

static void update_leds_from_framebuffer(uint16_t x){
    // update LEDs
    update_leds(framebuffer_live[NUM_PIXELS - 1 - x]);

    // udpate framebuffer if requested here
    if (update_requested_at_x == x){
        update_framebuffer();
        update_requested_at_x = -1;
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg){

    uint64_t timer_us;
    gptimer_get_raw_count(gptimer, &timer_us);

    // ignore second edge
    if (timer_us < ROTATION_SENSOR_HOLD_OFF_US) return;

    // round counter
    round_count++;

    // ignore first MIN_ROUNDS, then start with last measurement and then use weighted average
    if (round_count < MIN_ROUNDS) {
        return;
    } else if (round_count < STABLE_ROUNDS){
        rotation_us = timer_us;
    } else {
        // weighted average
        rotation_us = (rotation_us * (ROTATION_WEIGHT-1) + timer_us) / ROTATION_WEIGHT;
    }

    // update rotation and pixel time
    gptimer_set_raw_count(gptimer, 0);
    pixel_time_10ns = rotation_us * 100 / NUM_PIXELS;

#ifdef USE_TIMER_FOR_PIXEL
    // start display
    pixel_pos = 0;
    next_pixel_10ns = 0;

    // set LEDs
    update_leds_from_framebuffer(pixel_pos);

    // sechdeul next alarm
    pixel_pos++;
    next_pixel_10ns += pixel_time_10ns;
    alarm_config.alarm_count = next_pixel_10ns / 100;
    gptimer_set_alarm_action(gptimer, &alarm_config);
#endif
}

#ifdef USE_TIMER_FOR_PIXEL
static bool IRAM_ATTR timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // set LEDs
    update_leds_from_framebuffer(pixel_pos);
    pixel_pos++;

    // sechdule next alarm
    if (pixel_pos < NUM_PIXELS){
        // sechdeul next alarm
        next_pixel_10ns += pixel_time_10ns;
        alarm_config.alarm_count = next_pixel_10ns / 100;
        gptimer_set_alarm_action(timer, &alarm_config);
    }

    return pdFALSE;
}
#endif

static void clear_screen(void){
    memset(framebuffer_next, 0, NUM_PIXELS);
}

static void draw_char(uint16_t x_pos, uint8_t character){
    uint8_t row;
    uint8_t col;
    uint8_t * char_data = font8x8_basic[character];
    for (col=0;col<8;col++){
        uint8_t value = 0;
        for (row=0;row<8;row++){
            if ((char_data[row] & (1 << col)) != 0){
                value |= (1<<row);
            }
        }
        framebuffer_next[(x_pos+col) % NUM_PIXELS] = value;
    }
}

static void draw_string(const char * text, uint16_t x_pos){
    uint16_t i;
    clear_screen();
    for (i=0;i<strlen(text);i++){
        draw_char(x_pos + i * 8, text[i]);
    }

    // request update when 'cursor' is 5 pixels left from the string
    update_requested_at_x = (NUM_PIXELS - 1 - x_pos + 5) % NUM_PIXELS;
}

void app_main(void)
{
    // configure board LED and turn off
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    // init POV LEDs and turn off
    uint16_t i;
    for (i=0;i<8;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i, GPIO_MODE_OUTPUT);
        gpio_set_level(i, 1);
    }

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_XTAL,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // edge interrupt for zero, timer is used to update LEDs
#ifdef USE_TIMER_FOR_PIXEL
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
#endif
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    /* configure rotation sensor */
    gpio_reset_pin(ROTATION_GPIO);
    gpio_set_direction(ROTATION_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(ROTATION_GPIO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ROTATION_GPIO, gpio_isr_handler, NULL);
    gpio_intr_enable(ROTATION_GPIO);

    uint16_t rotation_offset = 0;

    while (1){

#ifdef USE_TIMER_FOR_PIXEL

        // next update possible
        if (update_requested_at_x < 0){

            // draw text
            draw_string("Hello World!", rotation_offset);
            
            // move text one pixel to the left
            if (rotation_offset == 0){
                rotation_offset = NUM_PIXELS;
            }  
            rotation_offset--;

        }

        // wait a bit
        vTaskDelay(10 / portTICK_PERIOD_MS);

#else
        // wait for startup
        while (round_count < MIN_ROUNDS){
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        uint64_t current_ticks;
        gptimer_get_raw_count(gptimer, &current_ticks);
        pixel_pos = current_ticks * 100 / pixel_time_100ns;
        update_leds_from_framebuffer(pixel_pos);
#endif
    }
}
