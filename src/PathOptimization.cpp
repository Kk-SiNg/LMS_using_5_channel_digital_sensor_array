/*
 * PathOptimization.cpp
 * LSRB path simplification.
 * Segment distances are now in mm (float).
 * When combining 3 segments, distances are properly summed.
 */

#include "PathOptimization.h"

PathOptimization::PathOptimization() {}

void PathOptimization::optimize(String &path, float segments[], int &pathLength) {
    String newPath = "";
    newPath.reserve(pathLength + 1);

    float newSegments[MAX_PATH_LENGTH];
    int newIndex = 0;

    int i = 0;
    while (i < pathLength) {
        if (i <= pathLength - 3) {
            String sub = path.substring(i, i + 3);

            // Combined distance: sum all 3 segments being merged
            float combinedDist = segments[i] + segments[i+1] + segments[i+2];

            char replacement = 0;

            if      (sub == "LBR") replacement = 'B';
            else if (sub == "LBS") replacement = 'R';
            else if (sub == "RBL") replacement = 'B';
            else if (sub == "SBL") replacement = 'R';
            else if (sub == "SBS") replacement = 'B';
            else if (sub == "LBL") replacement = 'S';
            else if (sub == "RBR") replacement = 'S';
            else if (sub == "SBR") replacement = 'L';
            else if (sub == "RBS") replacement = 'L';

            if (replacement) {
                newPath += replacement;
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
        }

        // No optimization, copy current
        newPath += path[i];
        newSegments[newIndex] = segments[i];
        i++;
        newIndex++;
    }

    // Copy back
    path = newPath;
    pathLength = newIndex;
    for (int k = 0; k < newIndex && k < MAX_PATH_LENGTH; k++) {
        segments[k] = newSegments[k];
    }
}