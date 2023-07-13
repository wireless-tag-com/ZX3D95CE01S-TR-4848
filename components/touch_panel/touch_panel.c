// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_log.h"
#include "touch_panel.h"
#include "ft5x06.h"

static const char *TAG = "touch panel driver";

#define TOUCH_CHECK(a, str, ret)  if(!(a)) {                                      \
        return (ret);                                                                   \
    }

extern touch_panel_driver_t ft5x06_default_driver;

esp_err_t touch_panel_find_driver(touch_panel_controller_t controller, touch_panel_driver_t *out_driver)
{
    TOUCH_CHECK(NULL != out_driver, "Pointer of Touch driver is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    switch (controller) {
    case TOUCH_PANEL_CONTROLLER_FT5X06:
        *out_driver = ft5x06_default_driver;
        break;
    default:
        ret = ESP_ERR_NOT_FOUND;
        break;
    }

    return ret;
}
