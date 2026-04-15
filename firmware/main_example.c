#include <stdio.h>
#include <stdint.h>

#include "step_detector.h"

// Replace these with your IMU and timer functions.
extern uint32_t millis(void);
extern void imu_read_g(float *ax, float *ay, float *az);

int main(void) {
    StepDetector detector;
    step_detector_init(&detector);

    while (1) {
        float ax, ay, az, threshold;
        uint32_t ts = millis();
        imu_read_g(&ax, &ay, &az);

        uint8_t is_step = step_detector_update(&detector, ts, ax, ay, az, &threshold);
        // Stream raw data back to laptop: timestamp_ms,ax,ay,az
        printf("%lu,%.4f,%.4f,%.4f\n", (unsigned long)ts, ax, ay, az);

        if (is_step) {
            printf("STEP,%lu,%lu,thr=%.3f\n", (unsigned long)ts, (unsigned long)detector.steps, threshold);
        }
    }

    return 0;
}
