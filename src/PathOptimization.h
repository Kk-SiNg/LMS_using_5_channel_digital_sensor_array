/*
 * PathOptimization.h
 * LSRB path simplification with distances in mm.
 */

#pragma once
#include <Arduino.h>
#include "Config.h"

class PathOptimization {
public:
    PathOptimization();

    // Optimizes the path string and its corresponding segment distances (mm).
    // pathLength is updated in-place.
    void optimize(String &path, float segments[], int &pathLength);
};