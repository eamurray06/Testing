#include "step_detector.h"

#include <math.h>
#include <stddef.h>

#define LP_ALPHA 0.08f
#define MIN_THRESHOLD 0.22f
#define THRESHOLD_SCALE 1.25f
#define REFRACTORY_MS 260u

static float compute_stddev(const float *arr, uint8_t n) {
    if (n < 2) return 0.0f;

    float mean = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        mean += arr[i];
    }
    mean /= (float)n;

    float var = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        float d = arr[i] - mean;
        var += d * d;
    }
    var /= (float)n;
    return sqrtf(var);
}

void step_detector_init(StepDetector *sd) {
    if (sd == NULL) return;
    sd->lp = 0.0f;
    sd->prev_hp = 0.0f;
    sd->peak = 0.0f;
    sd->window_index = 0;
    sd->window_count = 0;
    sd->last_step_ms = 0;
    sd->steps = 0;

    for (uint8_t i = 0; i < 40; ++i) {
        sd->hp_window[i] = 0.0f;
    }
}

uint8_t step_detector_update(StepDetector *sd, uint32_t timestamp_ms, float ax, float ay, float az, float *threshold_out) {
    if (sd == NULL) return 0;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    sd->lp += LP_ALPHA * (mag - sd->lp);
    float hp = mag - sd->lp;

    sd->hp_window[sd->window_index] = hp;
    sd->window_index = (uint8_t)((sd->window_index + 1) % 40);
    if (sd->window_count < 40) {
        sd->window_count++;
    }

    float sigma = compute_stddev(sd->hp_window, sd->window_count);
    float threshold = sigma * THRESHOLD_SCALE;
    if (threshold < MIN_THRESHOLD) threshold = MIN_THRESHOLD;
    if (threshold_out != NULL) *threshold_out = threshold;

    sd->peak *= 0.96f;
    if (hp > sd->peak) sd->peak = hp;

    uint8_t crossed_up = (sd->prev_hp < threshold && hp >= threshold);
    uint8_t refractory_ok = (timestamp_ms - sd->last_step_ms) >= REFRACTORY_MS;
    uint8_t peak_ok = sd->peak > threshold * 1.08f;

    uint8_t step = 0;
    if (crossed_up && refractory_ok && peak_ok) {
        sd->steps++;
        sd->last_step_ms = timestamp_ms;
        sd->peak = 0.0f;
        step = 1;
    }

    sd->prev_hp = hp;
    return step;
}
