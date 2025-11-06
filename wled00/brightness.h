//
// Created by ata on 11/6/25.
//

#ifndef WLED_BRIGHTNESS_H
#define WLED_BRIGHTNESS_H

extern int temp_factor;
extern int preferredBrightness;

int calcBrightness(int current, int target);

#endif //WLED_BRIGHTNESS_H