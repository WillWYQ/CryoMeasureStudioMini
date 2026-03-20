#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t DisplaySystem_Init(void);
esp_err_t DisplaySystem_Update(float temperature_c, bool temperature_valid);

#ifdef __cplusplus
}
#endif
