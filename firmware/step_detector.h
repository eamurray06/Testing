#ifndef STEP_DETECTOR_H
#define STEP_DETECTOR_H

#include <stdint.h>

typedef struct {
    float lp;
    float prev_hp;
    float peak;
    float hp_window[40];
    uint8_t window_index;
    uint8_t window_count;
    uint32_t last_step_ms;
    uint32_t steps;
} StepDetector;

void step_detector_init(StepDetector *sd);
uint8_t step_detector_update(StepDetector *sd, uint32_t timestamp_ms, float ax, float ay, float az, float *threshold_out);

#endif
