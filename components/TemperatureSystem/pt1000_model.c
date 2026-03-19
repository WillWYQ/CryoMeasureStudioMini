#include "pt1000_model.h"

#include <math.h>

#define PT1000_MIN_TEMP_C (-200.0f)
#define PT1000_MAX_TEMP_C (850.0f)

static float pt1000_r_from_t(float t, const TempSystemPT1000Model *m)
{
    float t2 = t * t;
    if (t >= 0.0f) {
        return m->r0_ohms * (1.0f + m->a * t + m->b * t2);
    }
    return m->r0_ohms * (1.0f + m->a * t + m->b * t2 + m->c * (t - 100.0f) * t2 * t);
}

static float pt1000_dr_dt(float t, const TempSystemPT1000Model *m)
{
    if (t >= 0.0f) {
        return m->r0_ohms * (m->a + 2.0f * m->b * t);
    }
    return m->r0_ohms *
           (m->a +
            2.0f * m->b * t +
            m->c * (4.0f * t * t * t - 300.0f * t * t));
}

TempSystemPT1000Model PT1000_DefaultModel(void)
{
    TempSystemPT1000Model model = {
        .r0_ohms = 1000.0f,
        .a = 3.9083e-3f,
        .b = -5.775e-7f,
        .c = -4.183e-12f,
    };
    return model;
}

float PT1000_TemperatureFromResistance(float r_ohms, const TempSystemPT1000Model *model)
{
    float t = 0.0f;

    t = (r_ohms - model->r0_ohms) / (model->r0_ohms * model->a);
    if (!isfinite(t)) {
        t = 0.0f;
    }

    for (int i = 0; i < 10; ++i) {
        float f = pt1000_r_from_t(t, model) - r_ohms;
        float df = pt1000_dr_dt(t, model);
        float dt = f / df;
        t -= dt;
        if (t < PT1000_MIN_TEMP_C) {
            t = PT1000_MIN_TEMP_C;
        } else if (t > PT1000_MAX_TEMP_C) {
            t = PT1000_MAX_TEMP_C;
        }
        if (fabsf(dt) < 0.001f) {
            break;
        }
    }

    return t;
}
