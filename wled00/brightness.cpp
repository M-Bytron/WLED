//
// Created by ata on 11/6/25.
//

#include "brightness.h"
#include "config.h"

int temp_factor = 0;
int preferredBrightness = 0;

int calcBrightness(int temp, int brightness) {
    const int STEP = 255 * BDP;

    int diff = temp - MIN_TEMP;

    if (diff > 0) {
        if (diff > temp_factor) {
            int decrease = (diff - temp_factor) * STEP;
            brightness -= decrease;
            temp_factor = diff;
        } else {
            temp_factor = diff;
        }
    }
    else if (diff < 0) {
        temp_factor = 0;
        brightness = preferredBrightness;
    }
    if (brightness < 0 ) {
        brightness = 0;
    }
    return brightness;
}