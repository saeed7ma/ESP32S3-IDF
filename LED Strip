#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

#define BLINK_GPIO GPIO_NUM_18 // Replace with your GPIO number
static uint8_t s_led_state = 0; // LED state (on/off)
static led_strip_handle_t led_strip;

void configure_led(void) {
    ESP_LOGI("LED", "Configuring addressable LED using RMT backend...");

    // Configure LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // Number of LEDs
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz resolution
        .flags.with_dma = false,
    };

    // Initialize the LED strip with RMT backend
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip); // Clear the LED
}

void blink_led(void) {
    if (s_led_state) {
        // Set the LED color (RGB)
        led_strip_set_pixel(led_strip, 0, 16, 16, 16); // Dim white
        led_strip_refresh(led_strip); // Apply the changes
    } else {
        // Turn off the LED
        led_strip_clear(led_strip);
    }
}

void app_main(void) {
    configure_led(); // Configure the LED strip

    while (1) {
        ESP_LOGI("LED", "Turning the LED %s!", s_led_state ? "ON" : "OFF");
        blink_led(); // Blink the LED
        s_led_state = !s_led_state; // Toggle state
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1-second delay
    }
}
