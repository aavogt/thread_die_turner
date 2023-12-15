// based on ESP-IDF hello_world
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "rom/ets_sys.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "esp_clk_tree.h"

#define HX_SCK 10 
#define HX_DT 6

// L298N motor control connects to GPIO 19 and 0
#define INA 19
#define INB 0

// EC11 rotary encoder connects to GPIO 7, 11, and 5
#define ROT_A 7
#define ROT_B 2
#define BUTTON 5

// 1544 RPM (60*1000 / rpm)
#define SAT_PERIOD_MS 39
#define FWD_SAT_PERIODS 8
#define REV_PERIODS 10


// add ets_delay_us
#define DELAY ets_delay_us(20)
uint32_t hx711_read(void)
{
    uint32_t count = 0;
    gpio_set_level(HX_SCK, 0);
    while (gpio_get_level(HX_DT)) {
        printf("Waiting for HX711 to be ready\n");
        vTaskDelay(10);
    }
    portDISABLE_INTERRUPTS();
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX_SCK, 1);
        DELAY;
        count = count << 1;
        gpio_set_level(HX_SCK, 0);
        DELAY;
        if (gpio_get_level(HX_DT)) {
            count++;
        }
    }
    // set gain to 128
    gpio_set_level(HX_SCK, 1);
    DELAY;
    count = count ^ 0x800000;
    gpio_set_level(HX_SCK, 0);
    DELAY;
    portENABLE_INTERRUPTS();
    return count;
}




QueueHandle_t rotation_queue;
TaskHandle_t rot_task_hdl;

void IRAM_ATTR rot_isr(void *arg) {
        xTaskNotifyFromISR(rot_task_hdl, (uint32_t) arg, eSetValueWithOverwrite, NULL);
}

void rot_task(void *arg) {
    uint8_t last_rot_a, last_rot_b;
    bool a_first = true;
    uint32_t gpio_num;
    last_rot_a = gpio_get_level(ROT_A);
    last_rot_b = gpio_get_level(ROT_B);
    while (true) {
        if (xTaskNotifyWait(0, ULONG_MAX, &gpio_num, portMAX_DELAY)) {
            uint8_t rot_a = gpio_get_level(ROT_A);
            uint8_t rot_b = gpio_get_level(ROT_B);
            if (rot_a != last_rot_a && rot_b != last_rot_b) {
                a_first = gpio_num == ROT_B;
                last_rot_a = rot_a;
                last_rot_b = rot_b;
                xQueueSend(rotation_queue, (void*) &a_first, (TickType_t) 10);
            }
        }
    }
}

// configures the EC11 rotary encoder gpio, glitch_filter, interrupt and task
// leading to a rotation_queue being filled with true and false for steps in
// each direction
//
// compare with https://github.com/jaytlang/him use of gpio_filter
// which interestingly passes goto labels instead of function pointers
void ec11_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ROT_A) | (1ULL << ROT_B),
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_isr_handler_add(ROT_A, rot_isr, (void *) ROT_A);
    gpio_isr_handler_add(ROT_B, rot_isr, (void *) ROT_B);
    rotation_queue = xQueueCreate(10, sizeof(bool));
    xTaskCreate(rot_task, "rot_task", 2048, NULL, tskIDLE_PRIORITY, &rot_task_hdl);
}



#define SET_INA(x) ramp_duty(LEDC_CHANNEL_1, x)
#define SET_INB(x) ramp_duty(LEDC_CHANNEL_0, x)

/// a gradual change might make it less jarring,
/// but updating the duty in a loop made it easy to end up with the motor
/// turning too far
/// ledc_set_fade_with_time might be better
void ramp_duty(ledc_channel_t channel, uint32_t new) {
        uint32_t old = ledc_get_duty(LEDC_LOW_SPEED_MODE, channel);
        if (new == old) return;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, new);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// parameters to describe what happens
// when the hx711_read exceeds the tare by .dead,
// the speed is a constant .creep_fwd_duty up to .creep_fwd
// then it rises linearly until the hx_read reaches .max_fwd
// reverse is full speed
//
// I should also have a time limit? Or ideally a distance limit.
struct bands {
        uint32_t dead, creep_fwd, creep_fwd_duty, saturate_duty, saturate_fwd;
};

// adjust multiplies the full-speed rotation time
float adjust = 1.0;
// reads the rotation_queue and increments/decrements "adjust" within bounds
void set_adjust() {
    while (true) {
        bool rot;
        int status = xQueueReceive(rotation_queue, &rot, portMAX_DELAY);
        if (status == errQUEUE_EMPTY) continue;
        if (rot) {
            adjust -= 0.1;
        } else {
            adjust += 0.1;
        }
        adjust = adjust < 0.1 ? 0.1 : adjust;
        adjust = adjust > 10.0 ? 10.0 : adjust;
    }
}

void app_main(void) {

    uint32_t resolution = 10; // consider ledc_find_suitable_duty_resolution
                              // but esp_clk_tree_src_get_freq_hz needs a clock
                              // and I'm not sure which one to use.
    struct bands bands = {
            .dead = 10000,
            .creep_fwd = 10000,
            .creep_fwd_duty = 1<<5,
            .saturate_fwd = 25000,
            .saturate_duty = 1<<resolution
    };


    // LEDC drives pins INA and INB
    ledc_timer_config_t ledc_timer = {
        .freq_hz = 60,                      // higher frequencies are smoother
                                            // but the minimum speed is lower
                                            // with a low frequency
        .duty_resolution = resolution,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = INA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
    ledc_channel.gpio_num = INB;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel_config(&ledc_channel);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    // set GPIO18 low, this is ground for the INB optocoupler
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<18),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(18, 0);
    

    // initialize HX711 serial interface
    io_conf.pin_bit_mask = (1ULL<<HX_SCK);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL<<HX_DT);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // here I will get a sense of how much pressure
    // can be applied, and then translate that into PWM duty
    // min/max are 8148131 8554261
    // how big is the dead band? Can it shrink or grow?

#define HX_ZERO 8000000
    uint32_t tare = 0;
    for (int i=0; i<10; i++) {
        uint32_t val = hx711_read();
        tare += val - HX_ZERO;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    tare /= 10;

    SET_INA(0);
    SET_INB(0);
    while(true) {
        
        int32_t diff, live;
        uint32_t val;
        val = hx711_read();

        diff = (int32_t)(val - HX_ZERO) - (int32_t)tare;
        live = diff - bands.dead;
        // set creep if live is positive and less than creep_fwd
        // then when live becomes greater than creep_fwd, set to saturate
        // for (28ms per rotation) * 6 rotations
        // then reverse for 28ms * 10 rotations
        if (live > 0) {
                printf("creep fwd\n");
                SET_INA(bands.creep_fwd_duty);
                SET_INB(0);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                while(true) { // wait for live to exceed creep_fwd
                        val = hx711_read();
                        diff = (int32_t)(val - HX_ZERO) - (int32_t)tare;
                        live = diff - bands.dead;
                        if (live < 0) {
                                printf("break creep fwd\n");
                                SET_INA(0);
                                break;
                        }
                        if (live > bands.saturate_fwd) {
                                printf("break creep fwd to saturate\n");
                                SET_INA(bands.saturate_duty);
                                vTaskDelay(FWD_SAT_PERIODS * SAT_PERIOD_MS / portTICK_PERIOD_MS);
                                SET_INA(0); // coast for 100 ms which avoids a brownout
                                vTaskDelay(100 / portTICK_PERIOD_MS);
                                SET_INB(bands.saturate_duty); // reverse
                                vTaskDelay(REV_PERIODS * SAT_PERIOD_MS / portTICK_PERIOD_MS);
                                SET_INB(0);
                                vTaskDelay(400 / portTICK_PERIOD_MS);
                                break;
                        }
                        vTaskDelay(28 / portTICK_PERIOD_MS);
                }
    };
    vTaskDelay(75 / portTICK_PERIOD_MS);
    printf("%li,%li,%li,%li\n", (long int)diff, (long int) live, (long int)val, (long int)tare);

    }
}
