/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Onboard_RGB_LED.h"

#if AP_NOTIFY_ONBOARD_RGB_LED_ENABLED

#ifndef AP_NOTIFY_ONBOARD_RGB_LED_NUMBERS
#define AP_NOTIFY_ONBOARD_RGB_LED_NUMBERS 1
#endif

#ifndef AP_NOTIFY_ONBOARD_RGB_LED_PIN
#error "define AP_NOTIFY_ONBOARD_RGB_LED_PIN"
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution

static uint8_t led_strip_pixels[AP_NOTIFY_ONBOARD_RGB_LED_NUMBERS * 3];

static const rmt_symbol_word_t ws2812_zero = {
    .duration0 = (uint16_t)(0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0H=0.3us
    .level0 = 1,
    .duration1 = (uint16_t)(0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T0L=0.9us
    .level1 = 0,
};

static const rmt_symbol_word_t ws2812_one = {
    .duration0 = (uint16_t)(0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1H=0.9us
    .level0 = 1,
    .duration1 = (uint16_t)(0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000), // T1L=0.3us
    .level1 = 0,
};

//reset defaults to 50uS
static const rmt_symbol_word_t ws2812_reset = {
    .duration0 = (uint16_t)(RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2),
    .level0 = 1,
    .duration1 = (uint16_t)(RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2),
    .level1 = 0,
};

static size_t encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // We need a minimum of 8 symbol spaces to encode a byte. We only
    // need one to encode a reset, but it's simpler to simply demand that
    // there are 8 symbol spaces free to write anything.
    if (symbols_free < 8) {
        return 0;
    }

    // We can calculate where in the data we are from the symbol pos.
    // Alternatively, we could use some counter referenced by the arg
    // parameter to keep track of this.
    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        // Encode a byte
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            if (data_bytes[data_pos]&bitmask) {
                symbols[symbol_pos++] = ws2812_one;
            } else {
                symbols[symbol_pos++] = ws2812_zero;
            }
        }
        // We're done; we should have written 8 symbols.
        return symbol_pos;
    } else {
        //All bytes already are encoded.
        //Encode the reset, and we're done.
        symbols[0] = ws2812_reset;
        *done = 1; //Indicate end of the transaction.
        return 1; //we only wrote one symbol
    }
}

#endif

Onboard_RGB_LED::Onboard_RGB_LED(void) :
    RGBLed(0x00, 0xFF, 0x7F, 0x33)
{
}

bool Onboard_RGB_LED::init(void)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = AP_NOTIFY_ONBOARD_RGB_LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback
        //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &simple_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));

#endif

    return true;
}

bool Onboard_RGB_LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    for (int led = 0; led < AP_NOTIFY_ONBOARD_RGB_LED_NUMBERS; led++) {
        led_strip_pixels[led * 3 + 0] = green;
        led_strip_pixels[led * 3 + 1] = red;
        led_strip_pixels[led * 3 + 2] = blue;
    }
    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, simple_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

#endif

    return true;
}

#endif  // AP_NOTIFY_ONBOARD_RGB_LED_ENABLED
