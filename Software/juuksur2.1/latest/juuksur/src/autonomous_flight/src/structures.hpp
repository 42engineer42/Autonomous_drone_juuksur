#pragma once

#include "ttmath.h"
#include <stdint.h>

struct MapNode {
    V3 pos;
    V2 dir;
    int prev;
    int next;
};

struct LocParticle {
    float weight;
    float heading;
    V2 pos; // x y position
};

struct IVec2 {
    int x,y;
    IVec2(int x, int y) {
        this->x = x;
        this->y = y;
    }
};

enum ContourCenterFlag {
    Contour_Center_Flag_Partial = 1<<0,
    Contour_Center_Flag_HasDirection = 1<<1
};

struct ContourCenter {
    V2 imageDirection;
    V2 imagePoint; // image pixel coordinates
    V3 camPoint; // camera space coordinates in cm
    V3 worldPoint; // world space coordinates in cm
    uint32_t flags; // contour partial because of screen edge
};

