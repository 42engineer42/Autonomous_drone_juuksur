#include "params.hpp"

// max difference in length allowed for 2 adjacent contours (previous and next)
float MAX_TRACK_VERT_DIST_DIF = 50.0f;
// max horizontal speed allowed
int MAX_SPEED = 100.0f;
int MAX_ANGLE_SPEED_DIF = 40.0f;
// min cos angle for contours to be connected
float MAX_TRACK_VERT_COS_ANGLE = 0.75f;
// max speed in directional mode (cm/s)
// how aggressively should we follow track (directional mode)
float MAX_DIRECTIONAL_SPEED = 30.0f;

float MAX_OFFSET_FROM_TRACK = 30.0f;
float HORIZONTAL_FOV_RAD = 1.085595f;
float VERTICAL_FOV_RAD = 0.851720675f;
// camera native sensor size
int NATIVE_WIDTH = 3280;
int NATIVE_HEIGHT = 2464;
// camera downscale factor (no fraction allowed!!!!)
int DOWNSCALE_DIVIDER = 16; // we downscale to 205x154 from 3280x2464

//const int DOWNSCALE_DIVIDER = 2;
float FOCAL_LENGTH = 3.04f; // in mm
float PIXELS_PER_MM = 892.857142857f; // at native resolution
// how far from target should be max speed (linearly reduces closer)
float X_ERROR_RANGE = 60.0f; // pitch multiplier will be x error divided by this and clamped to -1..1
float Y_ERROR_RANGE = 60.0f;
// min and max cm^2 area for contour to be considered as part of track
// in reality should be around 90-150
float MIN_CONTOUR_AREA = 40.0f;
float MAX_CONTOUR_AREA = 300.0f;

V2 IMAGE_CENTER = (V2){(float)(NATIVE_WIDTH/DOWNSCALE_DIVIDER/2), (float)(NATIVE_HEIGHT/DOWNSCALE_DIVIDER/2)};
V3 UP_DIRECTION = (V3){0.0f, 0.0f, 1.0f};

int PARTICLE_COUNT = 1000;
int RANDOM_PARTICLES = 0;

float HVEL_KI = 0.03f;
// last real drone tested value 0.6
//const float KP = 0.6f;
float HVEL_KP = 1.0f;
int MAX_CONTROL_ROLL = 100;
int MAX_CONTROL_PITCH = 100;
float MAX_ACCELERATION = 15.0f;

V3 ROLL_AXIS = (V3){0.0f, 1.0f, 0.0f};
V3 PITCH_AXIS = (V3){1.0f, 0.0f, 0.0f};
