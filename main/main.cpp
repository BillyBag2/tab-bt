#include "analyser.h"
#include "bsp/esp-bsp.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

extern "C" void app_main()
{
    ESP_LOGI("tab-bt", "Startup internal RAM: free=%u largest=%u; PSRAM free=%u",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(bsp_feature_enable(BSP_FEATURE_WIFI, true));
    bsp_display_cfg_t display_config{};
    display_config.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    display_config.buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT;
    display_config.double_buffer = true;
    display_config.flags.buff_dma = true;
    display_config.flags.buff_spiram = true;
    if (!bsp_display_start_with_config(&display_config)) {
        ESP_LOGE("tab-bt", "Display initialization failed");
        return;
    }
    analyser::start();
    bsp_display_lock(0);
    analyser::create_ui();
    bsp_display_unlock();
    ESP_ERROR_CHECK(bsp_display_backlight_on());
}
