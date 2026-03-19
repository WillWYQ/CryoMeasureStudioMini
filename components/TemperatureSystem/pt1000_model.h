#pragma once

#include "TemperatureSystem.h"

TempSystemPT1000Model PT1000_DefaultModel(void);
float PT1000_TemperatureFromResistance(float r_ohms, const TempSystemPT1000Model *model);
