#include "display_system.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define DISPLAY_H_RES                        720
#define DISPLAY_V_RES                        1280
#define DISPLAY_MIPI_LANE_NUM                2
#define DISPLAY_MIPI_LANE_BITRATE_MBPS       1000
#define DISPLAY_MIPI_PHY_PWR_LDO_CHAN        3
#define DISPLAY_MIPI_PHY_PWR_LDO_VOLTAGE_MV  2500
#define DISPLAY_PIXEL_SIZE_BYTES             sizeof(uint16_t)
#define DISPLAY_DRAW_BUFFER_BYTES            (DISPLAY_H_RES * DISPLAY_V_RES * DISPLAY_PIXEL_SIZE_BYTES)

#define DISPLAY_I2C_PORT                     I2C_NUM_1
#define DISPLAY_I2C_SDA_GPIO                 7
#define DISPLAY_I2C_SCL_GPIO                 8
#define DISPLAY_I2C_FREQ_HZ                  100000
#define DISPLAY_I2C_TIMEOUT_MS               50

#define DISPLAY_BACKLIGHT_ADDR               0x45
#define DISPLAY_BACKLIGHT_REG                0x96
#define DISPLAY_BACKLIGHT_FULL               0xFF

#define DISPLAY_TOUCH_ADDR_PRIMARY           0x14
#define DISPLAY_TOUCH_ADDR_ALTERNATE         0x5D
#define DISPLAY_TOUCH_PRODUCT_ID_REG         0x8140
#define DISPLAY_TOUCH_STATUS_REG             0x814E
#define DISPLAY_TOUCH_POINT1_REG             0x8150

#define SEG_A                                (1U << 0)
#define SEG_B                                (1U << 1)
#define SEG_C                                (1U << 2)
#define SEG_D                                (1U << 3)
#define SEG_E                                (1U << 4)
#define SEG_F                                (1U << 5)
#define SEG_G                                (1U << 6)

#define FONT_5X7_WIDTH                       5
#define FONT_5X7_HEIGHT                      7

typedef struct {
    bool controller_detected;
    bool touching;
    uint8_t points;
    uint16_t x;
    uint16_t y;
    uint8_t raw_status;
    uint8_t raw_points;
    uint16_t raw_x;
    uint16_t raw_y;
} display_touch_state_t;

typedef struct {
    bool initialized;
    esp_ldo_channel_handle_t ldo_handle;
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_panel_io_handle_t dbi_io;
    esp_lcd_panel_handle_t panel;
    uint16_t *draw_buffer;
    SemaphoreHandle_t refresh_done_sem;
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t backlight_dev;
    i2c_master_dev_handle_t touch_dev;
    bool backlight_available;
    bool touch_available;
    uint8_t touch_address;
    char rendered_temp_text[16];
    bool rendered_temp_valid;
    display_touch_state_t rendered_touch;
    display_touch_state_t logged_touch;
    esp_err_t last_touch_error;
} display_context_t;

static const char *TAG = "DisplaySystem";
static display_context_t s_display = {0};

static uint16_t display_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}

static int display_abs_diff_u16(uint16_t lhs, uint16_t rhs)
{
    return (lhs > rhs) ? (lhs - rhs) : (rhs - lhs);
}

static bool display_touch_equal(const display_touch_state_t *lhs, const display_touch_state_t *rhs)
{
    return lhs->controller_detected == rhs->controller_detected &&
           lhs->touching == rhs->touching &&
           lhs->points == rhs->points &&
           lhs->x == rhs->x &&
           lhs->y == rhs->y &&
           lhs->raw_status == rhs->raw_status &&
           lhs->raw_points == rhs->raw_points &&
           lhs->raw_x == rhs->raw_x &&
           lhs->raw_y == rhs->raw_y;
}

IRAM_ATTR static bool display_notify_refresh_done(esp_lcd_panel_handle_t panel,
                                                  esp_lcd_dpi_panel_event_data_t *edata,
                                                  void *user_ctx)
{
    SemaphoreHandle_t refresh_done_sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t need_yield = pdFALSE;

    (void)panel;
    (void)edata;

    if (refresh_done_sem != NULL) {
        xSemaphoreGiveFromISR(refresh_done_sem, &need_yield);
    }

    return (need_yield == pdTRUE);
}

static void display_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (s_display.draw_buffer == NULL || w <= 0 || h <= 0) {
        return;
    }

    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > DISPLAY_H_RES) {
        w = DISPLAY_H_RES - x;
    }
    if (y + h > DISPLAY_V_RES) {
        h = DISPLAY_V_RES - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }

    for (int row = y; row < y + h; ++row) {
        uint16_t *line = s_display.draw_buffer + (row * DISPLAY_H_RES) + x;
        for (int col = 0; col < w; ++col) {
            line[col] = color;
        }
    }
}

static void display_fill_screen(uint16_t color)
{
    display_fill_rect(0, 0, DISPLAY_H_RES, DISPLAY_V_RES, color);
}

static void display_draw_circle(int cx, int cy, int radius, uint16_t color)
{
    if (radius <= 0) {
        return;
    }

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if ((dx * dx) + (dy * dy) <= (radius * radius)) {
                display_fill_rect(cx + dx, cy + dy, 1, 1, color);
            }
        }
    }
}

static const uint8_t *display_font5x7_for_char(char ch)
{
    static const uint8_t glyph_space[FONT_5X7_HEIGHT] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    static const uint8_t glyph_c[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E,
    };
    static const uint8_t glyph_d[FONT_5X7_HEIGHT] = {
        0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C,
    };
    static const uint8_t glyph_e[FONT_5X7_HEIGHT] = {
        0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F,
    };
    static const uint8_t glyph_h[FONT_5X7_HEIGHT] = {
        0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11,
    };
    static const uint8_t glyph_k[FONT_5X7_HEIGHT] = {
        0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11,
    };
    static const uint8_t glyph_m[FONT_5X7_HEIGHT] = {
        0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11,
    };
    static const uint8_t glyph_o[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E,
    };
    static const uint8_t glyph_p[FONT_5X7_HEIGHT] = {
        0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10,
    };
    static const uint8_t glyph_r[FONT_5X7_HEIGHT] = {
        0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11,
    };
    static const uint8_t glyph_t[FONT_5X7_HEIGHT] = {
        0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    };

    switch (ch) {
    case ' ':
        return glyph_space;
    case 'C':
        return glyph_c;
    case 'D':
        return glyph_d;
    case 'E':
        return glyph_e;
    case 'H':
        return glyph_h;
    case 'K':
        return glyph_k;
    case 'M':
        return glyph_m;
    case 'O':
        return glyph_o;
    case 'P':
        return glyph_p;
    case 'R':
        return glyph_r;
    case 'T':
        return glyph_t;
    default:
        return NULL;
    }
}

static void display_draw_font5x7_char(int x, int y, int scale, char ch, uint16_t color)
{
    const uint8_t *glyph = display_font5x7_for_char(ch);
    if (glyph == NULL || scale <= 0) {
        return;
    }

    for (int row = 0; row < FONT_5X7_HEIGHT; ++row) {
        for (int col = 0; col < FONT_5X7_WIDTH; ++col) {
            if (glyph[row] & (1U << (FONT_5X7_WIDTH - 1 - col))) {
                display_fill_rect(x + (col * scale), y + (row * scale), scale, scale, color);
            }
        }
    }
}

static int display_measure_font5x7_text_width(const char *text, int scale, int spacing)
{
    size_t len = strlen(text);
    if (len == 0) {
        return 0;
    }

    return ((int)len * FONT_5X7_WIDTH * scale) + (((int)len - 1) * spacing);
}

static void display_draw_font5x7_text_centered(int center_x, int y, int scale, int spacing, const char *text, uint16_t color)
{
    int text_width = display_measure_font5x7_text_width(text, scale, spacing);
    int cursor_x = center_x - (text_width / 2);

    for (size_t i = 0; text[i] != '\0'; ++i) {
        display_draw_font5x7_char(cursor_x, y, scale, text[i], color);
        cursor_x += (FONT_5X7_WIDTH * scale) + spacing;
    }
}

static uint8_t display_segments_for_char(char ch)
{
    switch (ch) {
    case '0':
        return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case '1':
        return SEG_B | SEG_C;
    case '2':
        return SEG_A | SEG_B | SEG_D | SEG_E | SEG_G;
    case '3':
        return SEG_A | SEG_B | SEG_C | SEG_D | SEG_G;
    case '4':
        return SEG_B | SEG_C | SEG_F | SEG_G;
    case '5':
        return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case '6':
        return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '7':
        return SEG_A | SEG_B | SEG_C;
    case '8':
        return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '9':
        return SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    case '-':
        return SEG_G;
    case 'C':
        return SEG_A | SEG_D | SEG_E | SEG_F;
    default:
        return 0;
    }
}

static void display_draw_large_char(int x, int y, int w, int h, char ch, uint16_t color)
{
    if (ch == '.') {
        int dot = (w / 5);
        if (dot < 6) {
            dot = 6;
        }
        display_draw_circle(x + w - dot, y + h - dot, dot / 2, color);
        return;
    }

    uint8_t segments = display_segments_for_char(ch);
    if (segments == 0) {
        return;
    }

    int thickness = w / 6;
    if (thickness < 8) {
        thickness = 8;
    }
    int mid_y = y + (h / 2) - (thickness / 2);
    int upper_h = (h / 2) - thickness - (thickness / 2);
    int lower_h = upper_h;
    int inner_w = w - (2 * thickness);
    if (inner_w < thickness) {
        inner_w = thickness;
    }
    if (upper_h < thickness) {
        upper_h = thickness;
    }

    if (segments & SEG_A) {
        display_fill_rect(x + thickness, y, inner_w, thickness, color);
    }
    if (segments & SEG_D) {
        display_fill_rect(x + thickness, y + h - thickness, inner_w, thickness, color);
    }
    if (segments & SEG_G) {
        display_fill_rect(x + thickness, mid_y, inner_w, thickness, color);
    }
    if (segments & SEG_F) {
        display_fill_rect(x, y + thickness, thickness, upper_h, color);
    }
    if (segments & SEG_B) {
        display_fill_rect(x + w - thickness, y + thickness, thickness, upper_h, color);
    }
    if (segments & SEG_E) {
        display_fill_rect(x, mid_y + thickness, thickness, lower_h, color);
    }
    if (segments & SEG_C) {
        display_fill_rect(x + w - thickness, mid_y + thickness, thickness, lower_h, color);
    }
}

static void display_normalize_touch(display_touch_state_t *touch)
{
    bool raw_in_bounds = (touch->x < DISPLAY_H_RES) && (touch->y < DISPLAY_V_RES);
    bool swapped_in_bounds = (touch->y < DISPLAY_H_RES) && (touch->x < DISPLAY_V_RES);

    if (!raw_in_bounds && swapped_in_bounds) {
        uint16_t old_x = touch->x;
        touch->x = touch->y;
        touch->y = old_x;
    }

    if (touch->x >= DISPLAY_H_RES) {
        touch->x = DISPLAY_H_RES - 1;
    }
    if (touch->y >= DISPLAY_V_RES) {
        touch->y = DISPLAY_V_RES - 1;
    }
}

static esp_err_t display_i2c_add_device(uint16_t address, i2c_master_dev_handle_t *device)
{
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = DISPLAY_I2C_FREQ_HZ,
        .scl_wait_us = 0,
    };

    return i2c_master_bus_add_device(s_display.i2c_bus, &device_config, device);
}

static esp_err_t display_set_backlight(uint8_t value)
{
    if (!s_display.backlight_available || s_display.backlight_dev == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t payload[2] = {DISPLAY_BACKLIGHT_REG, value};
    return i2c_master_transmit(s_display.backlight_dev, payload, sizeof(payload), DISPLAY_I2C_TIMEOUT_MS);
}

static esp_err_t display_touch_read_reg(uint16_t reg, uint8_t *data, size_t len)
{
    if (!s_display.touch_available || s_display.touch_dev == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU)};
    return i2c_master_transmit_receive(s_display.touch_dev, reg_buf, sizeof(reg_buf), data, len, DISPLAY_I2C_TIMEOUT_MS);
}

static esp_err_t display_touch_write_reg_u8(uint16_t reg, uint8_t value)
{
    if (!s_display.touch_available || s_display.touch_dev == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t payload[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFU), value};
    return i2c_master_transmit(s_display.touch_dev, payload, sizeof(payload), DISPLAY_I2C_TIMEOUT_MS);
}

static void display_log_touch_state(const display_touch_state_t *touch)
{
    if (!touch->controller_detected) {
        return;
    }

    if (touch->raw_status != s_display.logged_touch.raw_status ||
        touch->raw_points != s_display.logged_touch.raw_points ||
        touch->raw_x != s_display.logged_touch.raw_x ||
        touch->raw_y != s_display.logged_touch.raw_y) {
        ESP_LOGI(TAG, "Touch raw: status=0x%02X points=%u raw_x=%u raw_y=%u",
                 touch->raw_status, touch->raw_points, touch->raw_x, touch->raw_y);
    }

    if (!s_display.logged_touch.touching && touch->touching) {
        ESP_LOGI(TAG, "Touch down: x=%u y=%u points=%u", touch->x, touch->y, touch->points);
    } else if (s_display.logged_touch.touching && !touch->touching) {
        ESP_LOGI(TAG, "Touch up");
    } else if (touch->touching &&
               (display_abs_diff_u16(touch->x, s_display.logged_touch.x) > 16 ||
                display_abs_diff_u16(touch->y, s_display.logged_touch.y) > 16 ||
                touch->points != s_display.logged_touch.points)) {
        ESP_LOGI(TAG, "Touch move: x=%u y=%u points=%u", touch->x, touch->y, touch->points);
    }

    s_display.logged_touch = *touch;
}

static esp_err_t display_poll_touch(display_touch_state_t *touch)
{
    memset(touch, 0, sizeof(*touch));
    touch->controller_detected = s_display.touch_available;

    if (!s_display.touch_available) {
        return ESP_OK;
    }

    uint8_t status = 0;
    esp_err_t err = display_touch_read_reg(DISPLAY_TOUCH_STATUS_REG, &status, 1);
    if (err != ESP_OK) {
        return err;
    }

    touch->raw_status = status;
    uint8_t points = status & 0x0FU;
    touch->raw_points = points;
    if ((status & 0x80U) == 0 || points == 0) {
        if ((status & 0x80U) != 0) {
            (void)display_touch_write_reg_u8(DISPLAY_TOUCH_STATUS_REG, 0x00);
        }
        return ESP_OK;
    }

    uint8_t point_buf[8] = {0};
    err = display_touch_read_reg(DISPLAY_TOUCH_POINT1_REG, point_buf, sizeof(point_buf));
    if (err != ESP_OK) {
        return err;
    }

    touch->touching = true;
    touch->points = points;
    touch->raw_x = (uint16_t)(((uint16_t)point_buf[2] << 8) | point_buf[1]);
    touch->raw_y = (uint16_t)(((uint16_t)point_buf[4] << 8) | point_buf[3]);
    touch->x = touch->raw_x;
    touch->y = touch->raw_y;
    display_normalize_touch(touch);

    (void)display_touch_write_reg_u8(DISPLAY_TOUCH_STATUS_REG, 0x00);
    return ESP_OK;
}

static void display_format_temperature(char *buffer, size_t buffer_len, float temperature_c, bool valid)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--.--C");
        return;
    }

    snprintf(buffer, buffer_len, "%.2fC", temperature_c);
}

static void display_draw_temperature(const char *temp_text, uint16_t color)
{
    size_t char_count = strlen(temp_text);
    if (char_count == 0) {
        return;
    }

    int gap = 12;
    int available_width = DISPLAY_H_RES - 96;
    int char_w = (available_width - ((int)(char_count - 1) * gap)) / (int)char_count;
    if (char_w > 92) {
        char_w = 92;
    }
    if (char_w < 46) {
        char_w = 46;
    }

    int char_h = char_w * 2;
    if (char_h > 240) {
        char_h = 240;
    }

    int total_width = ((int)char_count * char_w) + (((int)char_count - 1) * gap);
    int start_x = (DISPLAY_H_RES - total_width) / 2;
    int start_y = (DISPLAY_V_RES - char_h) / 2;

    for (size_t i = 0; i < char_count; ++i) {
        display_draw_large_char(start_x + ((int)i * (char_w + gap)), start_y, char_w, char_h, temp_text[i], color);
    }
}

static void display_draw_error_message(void)
{
    const uint16_t error_color = display_rgb565(255, 236, 236);
    const uint16_t hint_color = display_rgb565(255, 205, 205);
    const uint16_t accent = display_rgb565(220, 88, 88);

    display_draw_circle(DISPLAY_H_RES / 2, 300, 42, accent);
    display_fill_rect((DISPLAY_H_RES / 2) - 8, 248, 16, 72, display_rgb565(12, 18, 28));
    display_fill_rect((DISPLAY_H_RES / 2) - 8, 332, 16, 16, display_rgb565(12, 18, 28));
    display_draw_font5x7_text_centered(DISPLAY_H_RES / 2, 430, 10, 10, "TEMP ERROR", error_color);
    display_draw_font5x7_text_centered(DISPLAY_H_RES / 2, 560, 8, 8, "CHECK RTD", hint_color);
}

static const uint8_t *display_font5x7_for_debug_char(char ch)
{
    static const uint8_t glyph_0[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E,
    };
    static const uint8_t glyph_1[FONT_5X7_HEIGHT] = {
        0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E,
    };
    static const uint8_t glyph_2[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F,
    };
    static const uint8_t glyph_3[FONT_5X7_HEIGHT] = {
        0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E,
    };
    static const uint8_t glyph_4[FONT_5X7_HEIGHT] = {
        0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02,
    };
    static const uint8_t glyph_5[FONT_5X7_HEIGHT] = {
        0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E,
    };
    static const uint8_t glyph_6[FONT_5X7_HEIGHT] = {
        0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E,
    };
    static const uint8_t glyph_7[FONT_5X7_HEIGHT] = {
        0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08,
    };
    static const uint8_t glyph_8[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E,
    };
    static const uint8_t glyph_9[FONT_5X7_HEIGHT] = {
        0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E,
    };
    static const uint8_t glyph_colon[FONT_5X7_HEIGHT] = {
        0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00,
    };
    static const uint8_t glyph_s[FONT_5X7_HEIGHT] = {
        0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E,
    };
    static const uint8_t glyph_x[FONT_5X7_HEIGHT] = {
        0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11,
    };
    static const uint8_t glyph_y[FONT_5X7_HEIGHT] = {
        0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04,
    };

    switch (ch) {
    case '0':
        return glyph_0;
    case '1':
        return glyph_1;
    case '2':
        return glyph_2;
    case '3':
        return glyph_3;
    case '4':
        return glyph_4;
    case '5':
        return glyph_5;
    case '6':
        return glyph_6;
    case '7':
        return glyph_7;
    case '8':
        return glyph_8;
    case '9':
        return glyph_9;
    case ':':
        return glyph_colon;
    case 'S':
        return glyph_s;
    case 'X':
        return glyph_x;
    case 'Y':
        return glyph_y;
    default:
        return display_font5x7_for_char(ch);
    }
}

static void display_draw_debug_char(int x, int y, int scale, char ch, uint16_t color)
{
    const uint8_t *glyph = display_font5x7_for_debug_char(ch);
    if (glyph == NULL || scale <= 0) {
        return;
    }

    for (int row = 0; row < FONT_5X7_HEIGHT; ++row) {
        for (int col = 0; col < FONT_5X7_WIDTH; ++col) {
            if (glyph[row] & (1U << (FONT_5X7_WIDTH - 1 - col))) {
                display_fill_rect(x + (col * scale), y + (row * scale), scale, scale, color);
            }
        }
    }
}

static void display_draw_debug_text(int x, int y, int scale, int spacing, const char *text, uint16_t color)
{
    int cursor_x = x;

    for (size_t i = 0; text[i] != '\0'; ++i) {
        display_draw_debug_char(cursor_x, y, scale, text[i], color);
        cursor_x += (FONT_5X7_WIDTH * scale) + spacing;
    }
}

static void display_draw_touch_debug_panel(const display_touch_state_t *touch)
{
    const uint16_t panel_bg = display_rgb565(20, 30, 42);
    const uint16_t panel_border = display_rgb565(56, 166, 255);
    const uint16_t label = display_rgb565(210, 226, 242);
    char line1[32] = {0};
    char line2[32] = {0};
    char line3[32] = {0};

    snprintf(line1, sizeof(line1), "D:%u T:%u P:%u",
             touch->controller_detected ? 1U : 0U,
             touch->touching ? 1U : 0U,
             touch->points);
    snprintf(line2, sizeof(line2), "X:%04u Y:%04u", touch->x, touch->y);
    snprintf(line3, sizeof(line3), "S:%02X", touch->raw_status);

    display_fill_rect(60, 60, 280, 122, panel_bg);
    display_fill_rect(60, 60, 280, 4, panel_border);
    display_fill_rect(60, 178, 280, 4, panel_border);
    display_fill_rect(60, 60, 4, 122, panel_border);
    display_fill_rect(336, 60, 4, 122, panel_border);
    display_draw_debug_text(78, 78, 4, 3, line1, label);
    display_draw_debug_text(78, 112, 4, 3, line2, label);
    display_draw_debug_text(78, 146, 4, 3, line3, label);
}

static void display_render(const char *temp_text, bool temp_valid, const display_touch_state_t *touch)
{
    const uint16_t background = display_rgb565(12, 18, 28);
    const uint16_t rail = display_rgb565(28, 44, 64);
    const uint16_t accent = display_rgb565(56, 166, 255);
    const uint16_t good = display_rgb565(42, 194, 126);
    const uint16_t warn = display_rgb565(245, 183, 77);
    const uint16_t bad = display_rgb565(220, 88, 88);
    const uint16_t text_color = temp_valid ? display_rgb565(245, 248, 252) : warn;

    display_fill_screen(background);
    display_fill_rect(0, 0, DISPLAY_H_RES, 28, rail);
    display_fill_rect(0, DISPLAY_V_RES - 28, DISPLAY_H_RES, 28, rail);
    display_fill_rect(36, 96, 14, DISPLAY_V_RES - 192, accent);
    display_fill_rect(DISPLAY_H_RES - 50, 96, 14, DISPLAY_V_RES - 192, accent);
    display_draw_touch_debug_panel(touch);

    uint16_t status_color = bad;
    if (touch->controller_detected) {
        status_color = touch->touching ? warn : good;
    }
    display_draw_circle(DISPLAY_H_RES - 52, 52, 18, status_color);

    if (temp_valid) {
        display_draw_temperature(temp_text, text_color);
    } else {
        display_draw_error_message();
    }

    if (touch->controller_detected && touch->touching) {
        display_draw_circle(touch->x, touch->y, 18, accent);
        display_draw_circle(touch->x, touch->y, 10, display_rgb565(255, 255, 255));
    }
}

static esp_err_t display_submit_buffer(void)
{
    if (s_display.panel == NULL || s_display.draw_buffer == NULL || s_display.refresh_done_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    (void)xSemaphoreTake(s_display.refresh_done_sem, 0);

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_draw_bitmap(s_display.panel, 0, 0, DISPLAY_H_RES, DISPLAY_V_RES, s_display.draw_buffer),
        TAG,
        "draw bitmap failed");

    if (xSemaphoreTake(s_display.refresh_done_sem, pdMS_TO_TICKS(250)) != pdTRUE) {
        ESP_LOGW(TAG, "Timed out waiting for display refresh completion");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t display_init_i2c_devices(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = DISPLAY_I2C_PORT,
        .sda_io_num = DISPLAY_I2C_SDA_GPIO,
        .scl_io_num = DISPLAY_I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_display.i2c_bus), TAG, "create I2C bus failed");

    if (i2c_master_probe(s_display.i2c_bus, DISPLAY_BACKLIGHT_ADDR, DISPLAY_I2C_TIMEOUT_MS) == ESP_OK) {
        ESP_RETURN_ON_ERROR(display_i2c_add_device(DISPLAY_BACKLIGHT_ADDR, &s_display.backlight_dev), TAG, "add backlight device failed");
        s_display.backlight_available = true;
        ESP_LOGI(TAG, "Backlight controller detected at 0x%02X", DISPLAY_BACKLIGHT_ADDR);
    } else {
        ESP_LOGW(TAG, "Backlight controller 0x%02X not detected, display may stay dark", DISPLAY_BACKLIGHT_ADDR);
    }

    const uint8_t touch_addresses[] = {DISPLAY_TOUCH_ADDR_PRIMARY, DISPLAY_TOUCH_ADDR_ALTERNATE};
    for (size_t i = 0; i < sizeof(touch_addresses); ++i) {
        uint8_t addr = touch_addresses[i];
        if (i2c_master_probe(s_display.i2c_bus, addr, DISPLAY_I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_RETURN_ON_ERROR(display_i2c_add_device(addr, &s_display.touch_dev), TAG, "add touch device failed");
            s_display.touch_available = true;
            s_display.touch_address = addr;
            break;
        }
    }

    if (s_display.touch_available) {
        uint8_t product_id[4] = {0};
        esp_err_t err = display_touch_read_reg(DISPLAY_TOUCH_PRODUCT_ID_REG, product_id, sizeof(product_id));
        if (err == ESP_OK) {
            char product_id_str[5] = {0};
            for (size_t i = 0; i < sizeof(product_id); ++i) {
                product_id_str[i] = isprint((unsigned char)product_id[i]) ? (char)product_id[i] : '.';
            }
            ESP_LOGI(TAG, "Touch controller detected at 0x%02X, product ID: %s", s_display.touch_address, product_id_str);
        } else {
            ESP_LOGW(TAG, "Touch controller detected at 0x%02X, but product ID read failed: %s",
                     s_display.touch_address, esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "GT9271 touch controller not detected on 0x%02X/0x%02X",
                 DISPLAY_TOUCH_ADDR_PRIMARY, DISPLAY_TOUCH_ADDR_ALTERNATE);
    }

    return ESP_OK;
}

static esp_err_t display_init_panel(void)
{
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = DISPLAY_MIPI_PHY_PWR_LDO_CHAN,
        .voltage_mv = DISPLAY_MIPI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_config, &s_display.ldo_handle), TAG, "enable MIPI DSI PHY power failed");

    esp_lcd_dsi_bus_config_t dsi_bus_config = ILI9881C_PANEL_BUS_DSI_2CH_CONFIG();
    dsi_bus_config.num_data_lanes = DISPLAY_MIPI_LANE_NUM;
    dsi_bus_config.lane_bit_rate_mbps = DISPLAY_MIPI_LANE_BITRATE_MBPS;
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&dsi_bus_config, &s_display.dsi_bus), TAG, "create DSI bus failed");

    esp_lcd_dbi_io_config_t dbi_config = ILI9881C_PANEL_IO_DBI_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(s_display.dsi_bus, &dbi_config, &s_display.dbi_io), TAG, "create DSI DBI IO failed");

    esp_lcd_dpi_panel_config_t dpi_config = ILI9881C_720_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
    dpi_config.in_color_format = LCD_COLOR_FMT_RGB565;

    ili9881c_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = s_display.dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = DISPLAY_MIPI_LANE_NUM,
        },
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9881c(s_display.dbi_io, &panel_config, &s_display.panel), TAG, "create ILI9881C panel failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_display.panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_display.panel), TAG, "panel init failed");

    s_display.refresh_done_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_display.refresh_done_sem != NULL, ESP_ERR_NO_MEM, TAG, "create refresh semaphore failed");

    esp_lcd_dpi_panel_event_callbacks_t callbacks = {
        .on_color_trans_done = display_notify_refresh_done,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_dpi_panel_register_event_callbacks(s_display.panel, &callbacks, s_display.refresh_done_sem),
        TAG,
        "register panel callbacks failed");

    s_display.draw_buffer = heap_caps_calloc(1, DISPLAY_DRAW_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_display.draw_buffer != NULL, ESP_ERR_NO_MEM, TAG, "allocate draw buffer failed");

    esp_err_t err = esp_lcd_panel_disp_on_off(s_display.panel, true);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        return err;
    }

    return ESP_OK;
}

esp_err_t DisplaySystem_Init(void)
{
    if (s_display.initialized) {
        return ESP_OK;
    }

    memset(&s_display, 0, sizeof(s_display));
    s_display.last_touch_error = ESP_OK;

    ESP_RETURN_ON_ERROR(display_init_i2c_devices(), TAG, "display I2C init failed");
    ESP_RETURN_ON_ERROR(display_init_panel(), TAG, "display panel init failed");

    if (s_display.backlight_available) {
        ESP_RETURN_ON_ERROR(display_set_backlight(DISPLAY_BACKLIGHT_FULL), TAG, "set backlight failed");
    }

    display_touch_state_t initial_touch = {
        .controller_detected = s_display.touch_available,
    };
    display_format_temperature(s_display.rendered_temp_text, sizeof(s_display.rendered_temp_text), 0.0f, false);
    display_render(s_display.rendered_temp_text, false, &initial_touch);
    ESP_RETURN_ON_ERROR(display_submit_buffer(), TAG, "draw initial UI failed");
    s_display.rendered_temp_valid = false;
    s_display.rendered_touch = initial_touch;
    s_display.logged_touch = initial_touch;

    s_display.initialized = true;
    ESP_LOGI(TAG, "Display initialized: %dx%d portrait draw path ready", DISPLAY_H_RES, DISPLAY_V_RES);
    return ESP_OK;
}

esp_err_t DisplaySystem_Update(float temperature_c, bool temperature_valid)
{
    if (!s_display.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    display_touch_state_t touch = {0};
    esp_err_t touch_err = display_poll_touch(&touch);
    if (touch_err != ESP_OK) {
        touch.controller_detected = s_display.touch_available;
        if (s_display.last_touch_error != touch_err) {
            ESP_LOGW(TAG, "Touch read failed: %s", esp_err_to_name(touch_err));
        }
    } else if (s_display.last_touch_error != ESP_OK) {
        ESP_LOGI(TAG, "Touch read recovered");
    }
    s_display.last_touch_error = touch_err;

    display_log_touch_state(&touch);

    char temp_text[16] = {0};
    display_format_temperature(temp_text, sizeof(temp_text), temperature_c, temperature_valid);

    if (strcmp(temp_text, s_display.rendered_temp_text) == 0 &&
        s_display.rendered_temp_valid == temperature_valid &&
        display_touch_equal(&touch, &s_display.rendered_touch)) {
        return ESP_OK;
    }

    display_render(temp_text, temperature_valid, &touch);
    ESP_RETURN_ON_ERROR(display_submit_buffer(), TAG, "submit display buffer failed");
    strncpy(s_display.rendered_temp_text, temp_text, sizeof(s_display.rendered_temp_text) - 1);
    s_display.rendered_temp_text[sizeof(s_display.rendered_temp_text) - 1] = '\0';
    s_display.rendered_temp_valid = temperature_valid;
    s_display.rendered_touch = touch;
    return ESP_OK;
}
