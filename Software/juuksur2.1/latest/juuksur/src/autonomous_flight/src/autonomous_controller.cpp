
#include "autonomous_controller.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>

#include <std_msgs/UInt16.h>
#include <algorithm>
#include <numeric>
#include <chrono>

#define TTMATH_IMPLEMENTATION
#include "ttmath.h"
#undef TTMATH_IMPLEMENTATION


// 1 - robotex track with map, 2 - deltaX, 3 - dumb dashed line following
#define MODE 2

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SIGNF(x) ((x) >= 0.0f ? 1.0f : -1.0f)

#define LOG_VELOCITY 0

#define DEBUG_DRAW

AutonomousController::AutonomousController(std::function<void(cv::Mat&, cv::Mat&)> debugImg) {
    // most likely simulation
#if defined(__x86_64__) || defined(__i386__)
	float virtualPixelsPerMm = PIXELS_PER_MM / DOWNSCALE_DIVIDER;
	float fx = virtualPixelsPerMm * FOCAL_LENGTH;
	float cx = (float)(NATIVE_WIDTH/(DOWNSCALE_DIVIDER*2)); // basically half width (theoretical!)
	float cy = (float)(NATIVE_HEIGHT/(DOWNSCALE_DIVIDER*2)); 
	// camera space to screen space
	this->cam2Screen = { // column major order
		fx, 0.0f, 0.0f, 0.0f, fx, 0.0f, cx, cy, 1.0f
	};
	this->screen2Cam = {
		1.0f/fx, 0.0f, 0.0f, 0.0f, 1.0f/fx, 0.0f, -cx/fx, -cy/fx, 1.0f
	};

#else // real drone? use values from calibration
    // these were calibrated on half native resolution
    const float divd = DOWNSCALE_DIVIDER/2;
    float fx = 1288.56369 / divd;
    float fy = 1287.03306 / divd;
    float cx = 807.664125 / divd;
    float cy = 605.320103 / divd;
	// camera space to screen space
	this->cam2Screen = { // column major order
		fx, 0.0f, 0.0f, 0.0f, fy, 0.0f, cx, cy, 1.0f
	};
	this->screen2Cam = {
		1.0f/fx, 0.0f, 0.0f, 0.0f, 1.0f/fy, 0.0f, -cx/fx, -cy/fy, 1.0f
	};
#endif
    //printf("cam matrix \r\n%f %f %f\r\n%f %f %f\r\n%f %f %f\r\n", fx, 0.0f, cx, 0.0f, fx, cy, 0.0f, 0.0f, 1.0f);
	this->contourCenters2 = new std::vector<ContourCenter>();
	this->prevContourCenters2 = new std::vector<ContourCenter>();
	this->debugImg = debugImg;

    Quat q1, q2, q, invQ;
    quat_angle_axis(&q1, 180.0f, (V3){1.0f, 0.0f, 0.0f});
    quat_angle_axis(&q2, 90.0f, (V3){0.0f, 0.0f, 1.0f});
    quat_mul(&q, q1, q2);
    quat_angle_axis(&q1, -180.0f, (V3){1.0f, 0.0f, 0.0f});
    quat_angle_axis(&q2, -90.0f, (V3){0.0f, 0.0f, 1.0f});
    quat_mul(&invQ, q2, q1);
    loc.init(this->cam2Screen, q, invQ);

    mode = MODE;

#if LOG_VELOCITY == 1
	logfile.open("evel-kkkkk.txt", std::fstream::out);
	logfile << "\"time\",\"Vx\",\"Vy\",\"Vz\"\n";
	logfile << std::fixed;
#endif

}

AutonomousController::~AutonomousController() {
    delete this->contourCenters2;
    delete this->prevContourCenters2;
#if LOG_VELOCITY == 1
    logfile.close();
#endif
}

void AutonomousController::reset() {
    this->controlPitch = 1500.0f;
    this->controlRoll = 1500.0f;
    this->targetIntegralX = 0.0f;
    this->targetIntegralY = 0.0f;
    this->targetSIntegralX = 0.0f;
    this->targetSIntegralY = 0.0f;
    this->targetVIntegralX = 0.0f;
    this->targetVIntegralY = 0.0f;
    this->contourCenters2->clear();
    this->prevContourCenters2->clear();
    this->contours.clear();
    this->trackIndices.clear();

    this->targetDirection = (V2){0.0f, -1.0f};
    this->cpToGroundS = IMAGE_CENTER;
    this->active = false;
    this->movement = (V2){0.0f, 0.0f};
    memset(hMovAvg, 0, sizeof(hMovAvg));

    this->startTime = std::chrono::steady_clock::now();
    loc.reset();
}

// start autonomous steering
void AutonomousController::start() {
    this->reset();
    this->active = true;
    lastFrame = std::chrono::steady_clock::now();
}

// stop autonomous steering
void AutonomousController::stop() {
    this->controlPitch = 1500.0f;
    this->controlRoll = 1500.0f;
    this->active = false;
}

bool AutonomousController::isActive() {
    return this->active;
}

// callculate mathematical infinite plane based on orientation (assuming flat floor)
V4 AutonomousController::calculateGroundPlane(float dist, float roll, float pitch) {
    // convert ground plane to camera space (camera at 0.0, 0.0, 0.0)
    // when roll and pitch are 0, camera is looking towards y- (world space)
    // meaning ground plane normal is (0, 1, 0)
    // converted to camera space it is (0, 0, -1) (camera is looking towards z+)
    // in camera space roll and pitch rotate around x and y axis
    // thus we have to rotate the plane normal by pitch and roll
    V3 groundNormal = UP_DIRECTION;
    groundNormal = v3_scale(groundNormal, -1.0f);
    Quat rollQ, pitchQ, rotQ;
    quat_angle_axis(&rollQ, roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, pitch, PITCH_AXIS);
    quat_mul(&rotQ, pitchQ, rollQ);
    quat_v3_mul_dir(&groundNormal, rotQ, groundNormal);
    // plane equation
    return (V4){groundNormal.x, groundNormal.y, groundNormal.z, dist};
}

// calculate 3x3 rotation matrix from camera space to world space
void AutonomousController::camToWorldRotation(Mat3 *dst, float roll, float pitch) {
    Quat rollQ, pitchQ, rotQ;
    quat_angle_axis(&rollQ, -roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -pitch, PITCH_AXIS);
    quat_mul(&rotQ, rollQ, pitchQ);
    mat3_rotation(dst, &rotQ);
}

// convert world coordinate to screen coordinate
static V2 worldPointToScreen(V3 worldPos, Mat3 *camToScreen) {
    V3 ret;
    mat3_v3_mul(&ret, camToScreen, worldPos);
    ret = v3_inv_scale(ret, ret.z);
    return (V2) {ret.x, ret.y};
}

// calculate closest point to ground
static bool calculateCPToGround(V3 *res, V4 groundPlane) {
    if(ray_intersect_plane(res, (Ray){V3_ZERO, (V3){-groundPlane.x, -groundPlane.y, -groundPlane.z}}, groundPlane)) { // closest point to ground
        return true;
    }
    return false;
}

// convert screen coordinate to world coordinate assuming flat floor
static bool projectFromScreenToGround(V3 *res, V2 screenPoint, V4 groundPlane, Mat3 *screenToCam) {
    // homogenous screen coordinate
    V3 screenH = (V3) {screenPoint.x, screenPoint.y, 1.0};
    V3 rayDir;
    mat3_v3_mul(&rayDir, screenToCam, screenH);
    v3_normalize(&rayDir); // not sure if necessary
    Ray ray;
    ray.origin = (V3){0.0f, 0.0f, 0.0f};
    ray.dir = rayDir;
    if(ray_intersect_plane(res, ray, groundPlane)) {
        return true;
    }
    return false;
}


// find closest track contour in image coordinates
IVec2 AutonomousController::findClosestContourCenter(IVec2 imagePoint) {
    int count = contourCenters2->size();
    float min = 99999.0f;
    int minIdx = -1;
    for(int i = 0; i < count; i++) {
        float xdif = (*contourCenters2)[i].imagePoint.x - imagePoint.x;
        float ydif = (*contourCenters2)[i].imagePoint.y - imagePoint.y;
        float sqDist = sqrtf(xdif*xdif + ydif*ydif);
        if(sqDist < min) {
            minIdx = i;
            min = sqDist;
        }
    }
    V2 p = (*contourCenters2)[minIdx].imagePoint;
    return minIdx >= 0 ? (IVec2){(int)p.x, (int)p.y} : (IVec2){(int)IMAGE_CENTER.x, (int)IMAGE_CENTER.y};
}

// same as above but return index instead of coordinate
int AutonomousController::findClosestContourCenterIdx(IVec2 imagePoint) {
    int count = contourCenters2->size();
    float min = 99999.0f;
    int minIdx = -1;
    for(int i = 0; i < count; i++) {
        float xdif = (*contourCenters2)[i].imagePoint.x - imagePoint.x;
        float ydif = (*contourCenters2)[i].imagePoint.y - imagePoint.y;
        float sqDist = sqrtf(xdif*xdif + ydif*ydif);
        if(sqDist < min) {
            minIdx = i;
            min = sqDist;
        }
    }
    return minIdx;
}


bool compv2(V2 f, V2 s) {
    return f.x*f.x+f.y*f.y < s.x*s.x+s.y*s.y;
}

// alternative to calculatemedianmovement
// calculates optical flow from features, converts it to world space and then finds average
V2 AutonomousController::calculateOpticalFlow(cv::Mat from, cv::Mat to, cv::Mat debugImage, Mat3* screenToWorld, V4 groundPlane, float dt) {
    if(from.size() != to.size()) {
        return (V2){0.0f, 0.0f};
    }
    static std::vector<cv::Point2f> p0, p1;
    p0.clear();
    p1.clear();
    goodFeaturesToTrack(from, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04f);
    if(p0.size() == 0) {
	    return (V2){0.0f, 0.0f};
    }
    static std::vector<uchar> status;
    status.clear();
    cv::Mat err;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03f);
    cv::calcOpticalFlowPyrLK(from, to, p0, p1, status, err, cv::Size(15,15), 1, criteria);
    static std::vector<cv::Point2f> good_new;
    static std::vector<V2> diffs;
    good_new.clear();
    diffs.clear();
    V2 sum = (V2){0.0f, 0.0f};
    V2 sumS = (V2){0.0f, 0.0f};
    int count = 0;
    int countS = 0;
    for(uint i = 0; i < p0.size(); i++) {
        if(status[i] == 1) {
            V3 p0Cam, p1Cam;
            sumS.x += p1[i].x - p0[i].x;
            sumS.y += p1[i].y - p0[i].y;
            countS++;
            if(projectFromScreenToGround(&p0Cam, (V2){p0[i].x, p0[i].y}, groundPlane, &this->screenToWorldPrev)) {
                if(projectFromScreenToGround(&p1Cam, (V2){p1[i].x, p1[i].y}, groundPlane, screenToWorld)) {
                    V3 flow;
                    v3_sub(&flow, p1Cam, p0Cam);
                    // we calculate average flow in world space
                    sum.x += flow.x;
                    sum.y += flow.y;
                    diffs.push_back((V2){flow.x, flow.y});
                    count++;
                }
            }
            good_new.push_back(p1[i]);
#ifdef DEBUG_DRAW
            cv::line(debugImage, p1[i], p0[i], cv::Scalar(252, 0, 173)); 
#endif
        }
    }
    //std::sort(diffs.begin(), diffs.end(), compv2);
   
#if 1
    if(count > 0) {
        sumS.x /= (float)countS;
        sumS.y /= (float)countS;
        this->screenMovement = sumS;
        sum.x /= (float)count;
        sum.y /= (float)count;
        return sum;
    } else {
        return (V2){0.0f, 0.0f};
    }
#else
    // RANSAC
    float t = 10.0f;
    int n = 4;
    float d = 0.5f;
    int iters = 100;

    if(diffs.size() < 4) { 
        return (V2){0.0f, 0.0f};
    }
    
    V2 bestFit = diffs[0];
    float bestErr = 99999999.0f;

    std::vector<V2> maybeInliers;
    std::vector<int> pool(diffs.size());
    std::vector<V2> alsoInliers;

    for(int it = 0; it < iters; it++) {
        maybeInliers.clear();

        pool.resize(diffs.size());
        std::iota(std::begin(pool), std::end(pool), 0);
        std::random_shuffle(pool.begin(), pool.end());

        V2 maybeModel = (V2){0.0f, 0.0f};
        for(int i = 0; i < n; i++) {
            V2 mi = diffs[pool.back()];
            maybeInliers.push_back(mi);
            maybeModel = maybeModel + mi;
            pool.pop_back();
        }
        // TODO: least squares instead of average?
        maybeModel.x /= (float)n;
        maybeModel.y /= (float)n;

        alsoInliers.clear();
        for(int i = 0; i < pool.size(); i++) {
            int idx = pool[i];
            V2 errorV = diffs[idx] - maybeModel;
            float error = errorV.x*errorV.x + errorV.y*errorV.y;
            if(error < t) {
                alsoInliers.push_back(diffs[idx]);
            }
        }

        if((float)alsoInliers.size()/(float)diffs.size() >= d) {
            int count = 0;
            V2 betterModel = (V2){0.0f, 0.0f};
            for(int i = 0; i < maybeInliers.size(); i++) {
                betterModel = betterModel + maybeInliers[i];
                count++;
            }
            for(int i = 0; i < alsoInliers.size(); i++) {
                betterModel = betterModel + alsoInliers[i];
                count++;
            }
            betterModel.x /= (float)count;
            betterModel.y /= (float)count;
            float error = 0.0f;
            for(int i = 0 ; i < maybeInliers.size(); i++) {
                V2 errorV = maybeInliers[i] - betterModel;
                error += errorV.x*errorV.x + errorV.y*errorV.y;
            }
            for(int i = 0; i < alsoInliers.size(); i++) {
                V2 errorV = alsoInliers[i] - betterModel;
                error += errorV.x*errorV.x + errorV.y*errorV.y;
            }
            if(error < bestErr) {
                bestErr = error;
                bestFit = betterModel;
            }
        }
    }
    if(bestFit.x == diffs[0].x && bestFit.y == diffs[0].y) {
        printf("ransac failed!\r\n");
    }
    return bestFit;

    /*
    if(diffs.size() >= 5) 
        return diffs[diffs.size()/2];
    else if(diffs.size() > 0)
        return diffs[0];
    else
        return (V2){0.0f, 0.0f};*/
#endif
}

V2 calculateMedianMovement(std::vector<ContourCenter>* fromv, std::vector<ContourCenter>* tov) {
    // NOTE: this is APPROXIMATION
    // TODO: this doesn't take into account perspective change due to height change
    static std::vector<V2> diffs;
    diffs.clear();
    for(int i = 0; i < fromv->size(); i++) {
        int closest = -1;
        float minDist2 = FLT_MAX;
        if((*fromv)[i].flags&Contour_Center_Flag_Partial)
            continue;
        V3 from = (*fromv)[i].worldPoint; 
        for(int j = 0; j < tov->size(); j++) {
            V3 to = (*tov)[j].worldPoint;
            V3 dsub;
            v3_sub(&dsub, from, to);
            float dist2 = dsub.x*dsub.x + dsub.y*dsub.y;
            if(dist2 < minDist2) {
                closest = j;
                minDist2 = dist2;
            }
        }
        V3 to = (*tov)[closest].worldPoint;
        if(!((*tov)[closest].flags & Contour_Center_Flag_Partial)) {
            diffs.push_back((V2){to.x-from.x, to.y-from.y});
        }
    }
    std::sort(diffs.begin(), diffs.end(), compv2);
    if(diffs.size() >= 5) 
        return diffs[diffs.size()/2];
    else if(diffs.size() > 0)
        return diffs[0];
    else
        return (V2){0.0f, 0.0f};
}

// attempts to connect dashed track contours
void AutonomousController::updateTrackEdges(cv::Mat& rgbImage) {
    this->trackIndices.clear();

    int fittingEdgeCount = 0;

    std::vector<int> closest;
    for(int i = 0; i < contourCenters2->size(); i++) {
        closest.push_back(i);
    }
    // find triples of track contours that are in similar direction (and create 2 edges between them)
    for(int j = 0; j < contourCenters2->size(); j++) {
        V2 cur = (*contourCenters2)[j].imagePoint;
        // sort track contours by distance
        std::sort(closest.begin(), closest.end(), 
                [this, cur](const int& a, const int& b) -> bool
            {
                V2 difa, difb;
                v2_sub(&difa, (*contourCenters2)[a].imagePoint, cur);
                v2_sub(&difb, (*contourCenters2)[b].imagePoint, cur);
                return v2_dot(difa, difa) < v2_dot(difb, difb);
            }
        );
        //assert(closest[0] == j); // TODO: why is this not a valid assumption (assert fails sometimes)
        if(closest.size() >= 3) {
            // try to find pairs
            V2 toFirst;
            for(int l = 0; l < 2; l++) { // try 2 different directions
                int fidx = 1;
                if(l == 1) {
                    fidx = -1;
                    v2_sub(&toFirst, (*contourCenters2)[closest[1]].imagePoint, cur);
                    v2_normalize(&toFirst);
                    // find index with direction different than first was
                    for(int m = 2; m < closest.size()-1; m++) {
                        V2 toM;
                        v2_sub(&toM, (*contourCenters2)[closest[m]].imagePoint, cur);
                        v2_normalize(&toM);
                        if(fabs(v2_dot(toFirst, toM)) < MAX_TRACK_VERT_COS_ANGLE) {
                            fidx = m;
                            break;
                        }
                    }
                }
                // try to find pairing vertex to which direction is opposite but similar angle
                for(int k = 2; k < closest.size() && fidx != -1; k++) {
                    if(k == fidx) {
                        continue;
                    }
                    v2_sub(&toFirst, (*contourCenters2)[closest[fidx]].imagePoint, cur);
                    V2 toSecond;
                    v2_sub(&toSecond, (*contourCenters2)[closest[k]].imagePoint, cur);
                    float distDif = fabs(v2_len(toFirst) - v2_len(toSecond));
                    v2_normalize(&toFirst);
                    v2_normalize(&toSecond);
                    V2 p = (*contourCenters2)[closest[k]].imagePoint;
                    if(v2_dot(toFirst, toSecond) < -MAX_TRACK_VERT_COS_ANGLE) {
                        if(distDif < MAX_TRACK_VERT_DIST_DIF) {
                            trackIndices.push_back(j);
                            trackIndices.push_back(closest[fidx]);
                            trackIndices.push_back(j);
                            trackIndices.push_back(closest[k]);
                            // is this edge one we would be likely to move along?
                            if(fabs(v2_dot(toFirst, this->targetDirection)) > MAX_TRACK_VERT_COS_ANGLE
                                    || fabs(v2_dot(toSecond, this->targetDirection)) > MAX_TRACK_VERT_COS_ANGLE) {
                                fittingEdgeCount++;
                            }
                        }
                        break; // either matches or was too far, only getting worse from here
                    }
                }
            }
        }
    }


    // no track edges found, use more desperate measures 
    // (connect any 2 points with line in right direction)
    if(fittingEdgeCount == 0) {
        for(int i = 0; i < contourCenters2->size(); i++) {
            for(int j = 0; j < contourCenters2->size(); j++) {
                if(i == j) {
                    continue;
                }
                V2 start = (*contourCenters2)[i].imagePoint;
                V2 end = (*contourCenters2)[j].imagePoint;
                V2 dif;
                v2_sub(&dif, end, start);
                v2_normalize(&dif); 
                float ddot = v2_dot(dif, this->targetDirection);
                if(ddot < 0.0f) {
                    v2_sub(&dif, start, end);
                    v2_normalize(&dif);
                    ddot = -ddot;
                }
                if(ddot > MAX_TRACK_VERT_COS_ANGLE) {
                    this->trackIndices.push_back(i);
                    this->trackIndices.push_back(j);
                }
            }
        }
    }
    
    // debug draw
    for(int i = 0; i < trackIndices.size(); i+=2) {
        V2 p1 = (*contourCenters2)[trackIndices[i]].imagePoint;
        V2 p2 = (*contourCenters2)[trackIndices[i+1]].imagePoint;
#ifdef DEBUG_DRAW
        cv::line(rgbImage, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 255, 255));
#endif
    }
}

// find goal point on track (for PID), was used for dumb line following
bool AutonomousController::findWaypoint(cv::Mat& rgbImage, V2 *wp, V2 cpOnTrack) {
    V2 ints[2];
    V2 bestDirection = this->targetDirection;
    float bestDot = -FLT_MAX;
    bool found = false;
    // intersect track with circle then pick the one in best direction compared to current estimated target direction
    for(int i = 0; i < trackIndices.size(); i+=2) {
        V2 start, end, segDir;
        start = (*contourCenters2)[trackIndices[i]].imagePoint;
        end = (*contourCenters2)[trackIndices[i+1]].imagePoint;
        v2_sub(&segDir, end, start);
        v2_normalize(&segDir);
        if(fabs(v2_dot(segDir, this->targetDirection)) < MAX_TRACK_VERT_COS_ANGLE) {
            continue;
        }
        int intCount = lineSegCircleIntersection(ints, cpOnTrack, 30.0f, start, end);
        for(int j = 0; j < intCount; j++) {
            cv::RotatedRect ellipse(cv::Point(ints[j].x, ints[j].y), cv::Size(3,3), 0);
#ifdef DEBUG_DRAW
            cv::ellipse(rgbImage, ellipse, cv::Scalar(0, 0, 255), 3);
#endif
            V2 dir;
            v2_sub(&dir, ints[j], cpToGroundS);
            v2_normalize(&dir);
            float cdot = v2_dot(dir, this->targetDirection);
            if(cdot > bestDot && cdot > MAX_TRACK_VERT_COS_ANGLE) {
                bestDirection = dir;
                bestDot = cdot;
                found = true;
                *wp = ints[j];
            }
        }
    }
    // extrapolate from closest point on track (along the line segment)
    if(!found) {
        V2 cpOnTrack;
        int cpEdge;
        if(findClosestPointOnTrack(rgbImage, &cpOnTrack, &cpEdge)) {
            V2 start = (*contourCenters2)[trackIndices[cpEdge]].imagePoint;
            V2 end = (*contourCenters2)[trackIndices[cpEdge+1]].imagePoint;
            V2 dir;
            v2_sub(&dir, end, start);
            v2_normalize(&dir);
            float cdot = v2_dot(dir, this->targetDirection);
            if(cdot < 0.0f) { // flip
                v2_sub(&dir, start, end);
                v2_normalize(&dir);
                cdot = -cdot;
            }
            if(cdot > MAX_TRACK_VERT_COS_ANGLE) {
                found = true;
                dir = v2_scale(dir, 30.0f);
                v2_add(wp, cpOnTrack, dir);
                cv::RotatedRect ellipse(cv::Point(wp->x, wp->y), cv::Size(3,3), 0);
#ifdef DEBUG_DRAW
                cv::ellipse(rgbImage, ellipse, cv::Scalar(0, 0, 255), 3);
#endif
            }
        }
    }
    return found;
}

// calculate closest track point to drone
bool AutonomousController::findClosestPointOnTrack(cv::Mat& rgbImage, V2 *point, int* edge) {
    float bestD2 = FLT_MAX;
    bool found = false;
    for(int i = 0; i < trackIndices.size(); i+=2) {
        V2 start, end, dif, segDir;
        start = (*contourCenters2)[trackIndices[i]].imagePoint;
        end = (*contourCenters2)[trackIndices[i+1]].imagePoint;

        v2_sub(&segDir, end, start);
        v2_normalize(&segDir);
        // dont consider segments that are in wrong direction
        if(fabs(v2_dot(segDir, this->targetDirection)) < MAX_TRACK_VERT_COS_ANGLE) {
            continue;
        }

        V2 closest = closest_point_on_line_seg(cpToGroundS, start, end);
        v2_sub(&dif, cpToGroundS, closest);
        float d2 = dif.x*dif.x + dif.y*dif.y;
        if(d2 < bestD2) {
            found = true;
            bestD2 = d2;
            *point = closest;
            *edge = i;
        }
    }
#ifdef DEBUG_DRAW
    cv::RotatedRect ellipse(cv::Point(point->x, point->y), cv::Size(3,3), 0);
    cv::ellipse(rgbImage, ellipse, cv::Scalar(0, 255, 255), 3);
#endif
    return found;
}

// find furthest visible track point from current drone position
bool AutonomousController::findFurthestConnectedNode(cv::Mat& rgbImage, V2 *point, int* retNode, V2 *dir) {
    // TODO: 1. find connected line segments (to closest point)
    V2 cpOnTrack;
    int edge;
    if(!findClosestPointOnTrack(rgbImage, &cpOnTrack, &edge)) {
        return false;
    }
    V2 start, otherV;
    V2 best = cpOnTrack;;
    V2 segDirCorrected = this->targetDirection;

    bool found;
    // walk track along connected nodes as far as possible
    do {
        V2 toEdge;
        v2_sub(&toEdge, best, cpOnTrack);
        float dist2ToEdge = v2_dot(toEdge, toEdge);
        found = false;
        for(int i = 0; i < trackIndices.size(); i++) {
            start = (*contourCenters2)[trackIndices[i]].imagePoint;
            int other = (i&1) ? i-1 : i+1;
            if(trackIndices[i] == trackIndices[edge]) {
                V2 toOtherEnd;
                otherV = (*contourCenters2)[trackIndices[other]].imagePoint;
                V2 segDir;
                v2_sub(&segDir, otherV, start);
                v2_normalize(&segDir);
                segDirCorrected = (v2_dot(segDir, this->targetDirection) > 0.0f ? segDir : v2_scale(segDir, -1.0f));
                v2_sub(&toOtherEnd, otherV, cpOnTrack);
                float dist2ToOtherEnd = v2_dot(toOtherEnd, toOtherEnd);
                if(dist2ToOtherEnd > dist2ToEdge 
                        && v2_dot(toOtherEnd, this->targetDirection) > 0.0f
                        && fabs(v2_dot(segDir, this->targetDirection)) > MAX_TRACK_VERT_COS_ANGLE) {
                    best = otherV;
                    edge = other;
                    found = true;
                    *dir = segDirCorrected;
                    break;
                }
            }
        }
    } while(found);
    *retNode = edge;
    *point = best;
    return true;
}

// additional track contour filtering (based on shape and size)
int AutonomousController::filterContour(cv::Mat debugImage, cv::RotatedRect r, double area, float matchedRectArea, float aspect) {
    if(area < MIN_CONTOUR_AREA) { // too small
        cv::ellipse(debugImage, r, cv::Scalar(0, 100, 255), 3);
        return -1;
    }
    if(area > MAX_CONTOUR_AREA) { // too big
        cv::ellipse(debugImage, r, cv::Scalar(0, 255, 100), 3);
        return -2;
    }
    float precentage = area / matchedRectArea;
    if(precentage < 0.6f) { 
         // not enough like a rectangle
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, r, cv::Scalar(0, 0, 255), 3);
#endif
        return -3;
    }
    if(aspect < 2.5f) {
        return -4;
    }
    return 0;
}

// find and process track contours from image
void AutonomousController::updateContours(cv::Mat& grayFrame) {
    // calculate projected pixel size in cm^2 (about 0.18 at 1.5m height)
    // TODO: THIS IS COMPLETELY WRONG, SEE DeltaImage() for correct multiplier
    float projectedWidth = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(HORIZONTAL_FOV_RAD/2.0f);
    float projectedPixelArea = projectedWidth/(NATIVE_WIDTH/(float)DOWNSCALE_DIVIDER);
    //grayFrame.convertTo(grayFrame, CV_8UC1, 1.0);
    // remove old contours
    this->contours.clear(); 
    findContours(grayFrame, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    this->contourCenters2->clear();
    static std::vector<ContourCenter> centerCanditates;
    centerCanditates.clear();
    // TODO: no magic constants!
    // find center of each contour
    for(int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i])*projectedPixelArea;
        auto rect = cv::minAreaRect(contours[i]);
        float minAreaRectArea = rect.size.width*rect.size.height*projectedPixelArea;
        float aspect = rect.size.height > rect.size.width ? rect.size.height/rect.size.width : rect.size.width/rect.size.height;
        // how well does a bounding rectangle enclose this?
        float precentage = area / minAreaRectArea;
        cv::Moments m = moments(contours[i]);

        cv::Vec4f line;

        if(m.m00 != 0) {
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            cv::RotatedRect ellipse(cv::Point(cx, cy), cv::Size(3,3), 0);
            int filterResult = filterContour(debugImage, ellipse, area, minAreaRectArea, aspect);
            // skip too small contours
            if(filterResult == -1) {
                continue;
            }

            cv::fitLine(contours[i], line, cv::DIST_L2, 0, 0.01, 0.01);
            auto start = cv::Point(line[2], line[3]);
            cv::line(debugImage, start, start + cv::Point(line[0]*30.0f, line[1]*30.0f), cv::Scalar(200, 255, 100), 1);

            ContourCenter c;
            c.flags = 0;
            cv::Rect bounds = cv::boundingRect(contours[i]);

            // if it touches image edges it might be partial
            c.flags |= (bounds.x == 0 || bounds.x+bounds.width == grayFrame.cols || bounds.y == 0 || bounds.y+bounds.height == grayFrame.rows) ? Contour_Center_Flag_Partial : 0;

            c.imagePoint = (V2){(float)cx, (float)cy};
            c.imageDirection = (V2){line[0], line[1]};
            projectFromScreenToGround(&c.camPoint, (V2){(float)cx, (float)cy}, groundPlane, &this->screen2Cam);
            projectFromScreenToGround(&c.worldPoint, (V2){(float)cx, (float)cy}, groundPlane, &screenToWorld);

            cv::Point2f rpts[4];
            rect.points(rpts);
            for(int k = 0; k < 4; k++) {
#ifdef DEBUG_DRAW
                //cv::line(debugImage, rpts[k], rpts[(k+1)%4], cv::Scalar(0, 0, 255), 1, 8);
#endif
            }

            if(filterResult <= -3 || filterResult == -2 ) {
                centerCanditates.push_back(c);
            } else {
                this->contourCenters2->push_back(c);
#ifdef DEBUG_DRAW
                cv::ellipse(debugImage, ellipse, cv::Scalar(255, 0, 0), 3);
#endif
            }
        }
    }

#define CENTER_DIST_FROM_CONTOUR 45.0f
    // find center if it exists
    for(int i = 0; i < centerCanditates.size(); i++) {
        // TODO: find intersecting middle contour (by checking adjacent contours)
        float min = FLT_MAX;
        V3 dif;
        // find closest accepted contour
        for(int j = 0; j < this->contourCenters2->size(); j++) {
            auto contour = &(*this->contourCenters2)[j];
            dif = contour->camPoint - centerCanditates[i].camPoint;
            float d2 = v3_dot(dif, dif);
            if(d2 < min) {
                min = d2;
            }
        }
        v3_normalize(&dif);
        // find closest accepted contour ~60 deg different direction
        bool valid = false;
        if(min != FLT_MAX && sqrtf(min) < CENTER_DIST_FROM_CONTOUR) {
            min = FLT_MAX;
            for(int j = 0; j < this->contourCenters2->size(); j++) {
                auto contour = &(*this->contourCenters2)[j];
                V3 dif2 = contour->camPoint - centerCanditates[i].camPoint;
                float d2 = v3_dot(dif2, dif2);
                v3_normalize(&dif2);
                float dot = fabs(v3_dot(dif2, dif));
                if(dot > 0.34f && dot < 0.65 && sqrtf(d2) < CENTER_DIST_FROM_CONTOUR && d2 < min) {
                    // TODO: find min and compare distance?
                    valid = true;
                    break;
                }
            }
        }
        if(valid) {
            auto cont = &centerCanditates[i];
            contourCenters2->push_back(*cont);
            cv::RotatedRect ellipse(cv::Point(cont->imagePoint.x, cont->imagePoint.y), cv::Size(4,4), 0);
            cv::ellipse(debugImage, ellipse, cv::Scalar(255, 0, 255), 3);
            break;
        }
    }
    updateTrackEdges(debugImage);
}

// calculate current horizontal speed
// TODO: when not using fixed timestep dividing by current dt is wrong!
void AutonomousController::calculateMovement(cv::Mat& grayFrame, float dt) {
    movement = (V2){0.0f, 0.0f};
    V4 groundPlaneWorld = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
#if 1 // use optical flow ?
    // this requires a lot more compute (40ms on Pi3 vs <1ms), but potentially works over
    // any surface that has trackable features
    // also seems to be more accurate
    V2 curMovement = calculateOpticalFlow(grayFramePrev, grayFrame, debugImage, &screenToWorld, groundPlaneWorld, dt);
#else // from contour movement
    // calculate approximate ground-projected movement
    V2 curMovement = (V2){0.0f, 0.0f};
    if(prevContourCenters2->size() > 0 && contourCenters2->size() > 0) {
        curMovement = calculateMedianMovement(prevContourCenters2, contourCenters2);
    }
#endif
    hMovAvg[(movAvgIdx++)%H_MOV_AVG_COUNT] = curMovement;
    for(int i = 0; i < H_MOV_AVG_COUNT; i++) {
        movement.x += hMovAvg[i].x;
        movement.y += hMovAvg[i].y;
    }
    movement.x /= (float)H_MOV_AVG_COUNT;
    movement.y /= (float)H_MOV_AVG_COUNT;

    movement.x /= dt;
    movement.y /= dt;
}

// find goal point (for PID)
V2 AutonomousController::findTargetPoint() {
    // choose target point to move towards
    V2 bestTarget, waypoint, cpOnTrack;
    int dummy;
    findClosestPointOnTrack(debugImage, &bestTarget, &dummy);
    bool cpot = findClosestPointOnTrack(debugImage, &cpOnTrack, &dummy);
    if(findWaypoint(debugImage, &waypoint, cpOnTrack)) {
        bestTarget = waypoint;
        if(cpot) { // update direction
            V2 dir;
            v2_sub(&dir, waypoint, cpOnTrack);
            if(dir.x*dir.x + dir.y*dir.y > 0.001f) {
                v2_normalize(&dir);
                this->targetDirection = dir;
            }
        }
    } else if(cpot) {
        bestTarget = cpOnTrack;
        // using closest point on track
    } else if(this->contourCenters2->size() > 0) {
    bestTarget = (*this->contourCenters2)[0].imagePoint;
    } else {
        // couldn't even find track
        bestTarget = cpToGroundS; // attempt to stand still
    }
    return bestTarget;
}

// find direction to move towards (dumb line following based on track and drone position)
V2 AutonomousController::findTargetDirection() {
    V2 cpOnTrack;
    int cpEdge;
    if(findClosestPointOnTrack(debugImage, &cpOnTrack, &cpEdge)) {
        V2 start = (*contourCenters2)[trackIndices[cpEdge]].imagePoint;
        V2 end = (*contourCenters2)[trackIndices[cpEdge+1]].imagePoint;
        V2 ret = (V2){0.0f, 0.0f};
        v2_sub(&ret, end, start);
        v2_normalize(&ret);
        float dot = v2_dot(ret, this->targetDirection);
        if(dot >= MAX_TRACK_VERT_COS_ANGLE) {
            return ret;
        } else if(dot < 0.0f && fabs(dot) >= MAX_TRACK_VERT_COS_ANGLE) {
            v2_sub(&ret, start, end);
            v2_normalize(&ret);
            return ret;
        }
    }
    return this->targetDirection; 
}

// move towards point on camera image
void AutonomousController::controlMovementToTarget(V2 targetPosS, IVec2 curPosS, double dt, float maxSpeed) {
    V2 droneMovement = (V2){-movement.x, -movement.y};
    // to world coordinates
    V3 targetPosW = (V3){targetPosS.x, targetPosS.y, 0.0f};
    V3 curPosW = (V3){(float)curPosS.x, (float)curPosS.y, 0.0f};
    V4 groundPlane = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
    projectFromScreenToGround(&targetPosW, targetPosS, groundPlane, &this->screenToWorld);
    projectFromScreenToGround(&curPosW, (V2){(float)curPosS.x, (float)curPosS.y}, groundPlane, &this->screenToWorld);

    // x velocity pid
    float errorx = targetPosW.x-curPosW.x;
    this->targetIntegralX += dt*errorx;
    // old value 2.0f
    float targetxspeed = 1.0f*errorx + 0.0f*this->targetIntegralX;
    //float targetxspeed = 0.0f;
    targetxspeed = CLAMP(targetxspeed, -maxSpeed, maxSpeed);

    // y velocity pid
    float errory = targetPosW.y-curPosW.y;
    this->targetIntegralY += dt*errory;
    // old value 2.0f
    float targetyspeed = 1.0f*errory + 0.0f*this->targetIntegralY;
    //float targetyspeed = -20.0f;
    targetyspeed = CLAMP(targetyspeed, -maxSpeed, maxSpeed);
    // pitch pid

    V2 vdir = (V2){targetxspeed, targetyspeed};
    float speed = v2_len(vdir);
    if(speed > 0.01f) {
        v2_normalize(&vdir);
    }
    
    velCtrl.update(vdir, speed, droneMovement, dt);
#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetxspeed, curPosS.y+targetyspeed), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x), curPosS.y+(droneMovement.y)), cv::Scalar(0, 255, 255));
#endif
}

// move along track based on current position on map
void AutonomousController::controlMovementFromLocation(IVec2 curPosS, double dt) {
    V2 droneMovement = (V2){-movement.x, -movement.y};
    // forward with 0 heading on image
    V2 forward = (V2){0.0f, -1.0f};
    V2 down = (V2){1.0f, 0.0f};
    // TODO: this assumes that heading is always the same but it might not be (gyro heading drifts)
    float heading = -90.0f * loc.forwardT;
    // in map coordinates
    V2 targetLocation = loc.getPositionOnMap(4.5f); // 4.0 - look ahead (1.0 = distance between contours)
    V2 currentLocation = (V2){loc.lastEstimation.x, loc.lastEstimation.y};
    V2 dirToTarget = targetLocation - currentLocation;
    dirToTarget = v2_rotate(dirToTarget, TT_DEG2RAD_F*heading);
    v2_normalize(&dirToTarget);

    V2 screenDir;
    screenDir.x = dirToTarget.x * forward.x + dirToTarget.y * down.x;
    screenDir.y = dirToTarget.x * forward.y + dirToTarget.y * down.y;

    // for going slower in turns
    //float velMultiplier = loc.getVelocityMultiplier(0.0f);
    float velMultiplier = 1.0f;
    float speed;
    if(loc.forwardDetermined) {
        speed = MAX_SPEED * velMultiplier;
    } else {
        speed = 0.0f;
    }

    this->targetDirection = screenDir;

    velCtrl.update(screenDir, speed, droneMovement, dt);

#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+screenDir.x*velCtrl.curVelocity, curPosS.y+screenDir.y*velCtrl.curVelocity), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x), curPosS.y+(droneMovement.y)), cv::Scalar(0, 255, 255));
    //cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(toTrack.x*10), curPosS.y+(toTrack.y*10)), cv::Scalar(0, 80, 255));
    //cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(targetDMap.x*10), curPosS.y+(targetDMap.y*10)), cv::Scalar(0, 255, 80));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(targetDirection.x*10), curPosS.y+(targetDirection.y*10)), cv::Scalar(0, 255, 0));
#endif
}

// move towards image space direction
void AutonomousController::controlMovementToDirection(V2 targetDirection, IVec2 curPosS, V2 biasPoint, double dt) {
    bool stable = distanceToGround >= 100 && distanceToGround <= 200;
    if(!stable) {
        this->controlPitch = 1500;
        this->controlRoll = 1500;
        return;
    }	
    // drone movement is opposite to optical flow
    V2 droneMovement = (V2){-movement.x, -movement.y};
    V2 targetVelocity = targetDirection;

    // from current position to track
    V2 offsetFromBias;
    v2_sub(&offsetFromBias, biasPoint, (V2){(float)curPosS.x, (float)curPosS.y});
    float biasScale = CLAMP(v2_len(offsetFromBias)/MAX_OFFSET_FROM_TRACK, 0.0f, 1.0f)*0.25f;
    v2_normalize(&offsetFromBias);
    float det = targetDirection.x*offsetFromBias.y-offsetFromBias.x*targetDirection.y;
    //printf("bias %f det %f\r\n", biasScale, det);
    float angle = (det>=0.0f?1.0f:-1.0f)*biasScale;
    targetVelocity.x = targetVelocity.x*cosf(angle)-targetVelocity.y*sinf(angle);
    targetVelocity.y = targetVelocity.x*sinf(angle)+targetVelocity.y*cosf(angle);

    V2 mdir = droneMovement;
    v2_normalize(&mdir);
    float targetSpeed =  MAX_DIRECTIONAL_SPEED*fabs(v2_dot(mdir, targetVelocity));
    //targetVelocity = v2_scale(targetVelocity, targetSpeed);
    //printf("speed %f\r\n", targetSpeed);

#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetVelocity.x, curPosS.y+targetVelocity.y), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x), curPosS.y+(droneMovement.y)), cv::Scalar(0, 255, 255));
#endif
    velCtrl.update(targetVelocity, targetSpeed, droneMovement, dt);
}

void AutonomousController::drawTelemetry(cv::Mat& debugImage) {
    char buf[256];
    auto textColor = cv::Scalar(0, 255, 0);
    sprintf(buf, "HEI: %4d", (int)distanceToGround);
    cv::putText(debugImage, buf, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "PIC: %.3f", pitch);
    cv::putText(debugImage, buf, cv::Point(20, 45), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "ROL: %.3f", roll);
    cv::putText(debugImage, buf, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "THR: %4d", (int)throttle);
    cv::putText(debugImage, buf, cv::Point(20, 75), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "SPD: %.2f", v2_len(movement));
    cv::putText(debugImage, buf, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "SPDS: %.2f", v2_len(screenMovement));
    cv::putText(debugImage, buf, cv::Point(20, 105), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "CROL: %4d", velCtrl.controlRoll);
    cv::putText(debugImage, buf, cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
    sprintf(buf, "CPIC: %4d", velCtrl.controlPitch);
    cv::putText(debugImage, buf, cv::Point(20, 135), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
}

void AutonomousController::deltaImage(cv::Mat& grayFrame, double dt) {
    threshold(grayFrame, this->binaryFrame, 150, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    cvtColor(this->binaryFrame, this->debugImage, CV_GRAY2BGR);
    // TODO: find the ground vehicle 
   
    V3 cpToGround; // closest point to ground
    // closest point to ground in image coordinates
    if(calculateCPToGround(&cpToGround, groundPlane)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(cpToGroundS.x, cpToGroundS.y);
        cv::Size ellipseSize(3,3);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(255, 255, 0), 3);
#endif
    } else {
        printf("closest point to ground is off screen!\r\n");
    }

    // total horizontal width of visible area
    float projectedWidth = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(HORIZONTAL_FOV_RAD/2.0f)*2.0f;
    float pixelWidthCm = projectedWidth/(NATIVE_WIDTH/(float)DOWNSCALE_DIVIDER);
    float projectedHeight = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(VERTICAL_FOV_RAD/2.0f)*2.0f;
    float pixelHeightCm = projectedHeight/(NATIVE_HEIGHT/(float)DOWNSCALE_DIVIDER);
    float pixelArea = pixelWidthCm*pixelHeightCm;
    this->contours.clear();
    findContours(binaryFrame, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    struct MarkerContour {
        float score;
        V2 imagePoint;
    };

    std::vector<MarkerContour> candidateContours;

    for(int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        double approxRealArea = area * pixelArea;
        auto rect = cv::minAreaRect(contours[i]);
        cv::Moments m = moments(contours[i]);
        if(m.m00 != 0 && approxRealArea > 1000.0f) {
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            cv::RotatedRect ellipse(cv::Point(cx, cy), cv::Size(3,3), 0);
            cv::Rect bounds = cv::boundingRect(contours[i]);
            cv::Point2f rpts[4];
            rect.points(rpts);
            cv::ellipse(debugImage, ellipse, cv::Scalar(255, 0, 0), 3);
            // try to find platform marker circle
            if(contours[i].size() > 5) {
                auto boundEllipse = cv::fitEllipse(contours[i]);
                auto boundArea = M_PI_4 * boundEllipse.size.width * boundEllipse.size.height;
                auto zeroArea = m.m00;
                std::vector<cv::Point> approx;
                cv::approxPolyDP(contours[i], approx, 5, true);
                double match = area / boundArea;
                double perimeter = cv::arcLength(contours[i], true);
                double circularity = (4.0*M_PI*area)/(perimeter*perimeter);
                // NOTE: the marker diameter should be at least 60cm or around 3000 cm^2
                double realArea = pixelArea * boundArea;
                if(match > 0.8 && circularity > 0.7 && realArea > 1800.0f && realArea < 4000.0f) {
                    printf("area %f match %f circ %f\r\n", realArea, match, circularity);
                    cv::ellipse(debugImage, boundEllipse, cv::Scalar(255, 255, 0), 1);
                        
                    for(int k = 0; k < 4; k++) {
                        cv::line(debugImage, rpts[k], rpts[(k+1)%4], cv::Scalar(0, 0, 255), 1, 8);
                    }
                    cv::drawContours(debugImage, contours, i, cv::Scalar(0, 100, 200));
                    candidateContours.push_back((MarkerContour){
                                (float)(match*circularity),
                                (V2){(float)cx, (float)cy}
                            }
                        );
                }
            }
        }
    }

    // for some reason writing this is easier than using std library
    bool found = false;
    float maxScore = -999999.0f;
    MarkerContour bestMarker;
    for(int i = 0; i < candidateContours.size(); i++) {
        if(maxScore < candidateContours[i].score) {
            found = true;
            maxScore = candidateContours[i].score;
            bestMarker = candidateContours[i];
        }
    }


    IVec2 curPosS(cpToGroundS.x, cpToGroundS.y);
    V2 target = (V2){(float)curPosS.x, (float)curPosS.y};

    if(dt > 0.001) {
        calculateMovement(grayFrame, dt);
    } else {
        movement = (V2){0.0f, 0.0f};
    }

    // ground vehicle marker was found!
    if(found) {
        // draw marker on debug view as red dot
        cv::RotatedRect ellipse(cv::Point(bestMarker.imagePoint.x, bestMarker.imagePoint.y), cv::Size(3,3), 0);
        cv::ellipse(debugImage, ellipse, cv::Scalar(0, 0, 255), 3);

        target = bestMarker.imagePoint;
        controlMovementToTarget(target, curPosS, dt, 10.0f);
    }

    /*for(int i = 0; i < circles.size();  i++) {
w    cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle(debugImage, center, 1, cv::Scalar(0, 100, 100), 2);
        // circle outline
        int radius = c[2];
        cv::circle(debugImage, center, radius, cv::Scalar(255, 0, 255), 1);
    }*/

#ifdef DEBUG_DRAW
    cv::resize(debugImage, debugImageSmall, cv::Size(), 1.0, 1.0, cv::INTER_LINEAR);
    drawTelemetry(debugImageSmall);
#endif // DEBUG_DRAW
#ifdef DEBUG_DRAW
    this->debugImg(debugImageSmall, loc.image);
#endif

    grayFramePrev = grayFrame;
}

void AutonomousController::followImage(cv::Mat& grayFrame, double dt) {
    threshold(grayFrame, this->binaryFrame, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cvtColor(this->binaryFrame, this->debugImage, CV_GRAY2BGR);
    updateContours(this->binaryFrame);

    V3 cpToGround; // closest point to ground
    // closest point to ground in image coordinates
    if(calculateCPToGround(&cpToGround, groundPlane)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(cpToGroundS.x, cpToGroundS.y);
        cv::Size ellipseSize(3,3);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(255, 255, 0), 3);
#endif
    } else {
        printf("closest point to ground is off screen!\r\n");
    }

    // calculate horizontal movement 
    if(dt > 0.001) {
        calculateMovement(grayFrame, dt);
    } else {
        movement = (V2){0.0f, 0.0f};
    }
    IVec2 groundProj(cpToGroundS.x, cpToGroundS.y);

    this->targetDirection = findTargetDirection();
    IVec2 curPosS(cpToGroundS.x, cpToGroundS.y);
#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetDirection.x*100, curPosS.y+targetDirection.y*100), cv::Scalar(0, 255, 0));
#endif // DEBUG_DRAW
    V2 moveDir = movement;
    v2_normalize(&moveDir);
    
    V2 cpOnTrack;
    int dummy;

    if(!findClosestPointOnTrack(debugImage, &cpOnTrack, &dummy)) {
        cpOnTrack = (V2){(float)curPosS.x, (float)curPosS.y};
    }

    V2 furthestPoint = cpOnTrack;
    V2 furthestDirection = this->targetDirection;
    if(findFurthestConnectedNode(debugImage, &furthestPoint, &dummy, &furthestDirection)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(furthestPoint.x, furthestPoint.y);
        cv::Size ellipseSize(3,3);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(80, 155, 80), 3);
#endif // DEBUG_DRAW
    }
    controlMovementToDirection(furthestDirection, curPosS, cpOnTrack, dt);
    //printf("m %f %f\r\n", movement.x, movement.y);

#ifdef DEBUG_DRAW
    cv::resize(debugImage, debugImageSmall, cv::Size(), 1.0, 1.0, cv::INTER_LINEAR);
    drawTelemetry(debugImageSmall);
#endif // DEBUG_DRAW

#ifdef DEBUG_DRAW
    this->debugImg(debugImageSmall, loc.image);
#endif

    auto temp = prevContourCenters2;
    prevContourCenters2 = contourCenters2;
    contourCenters2 = temp;
    grayFramePrev = grayFrame;
}

void AutonomousController::robotexImage(cv::Mat& grayFrame, double dt) {
    threshold(grayFrame, this->binaryFrame, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cvtColor(this->binaryFrame, this->debugImage, CV_GRAY2BGR);
    updateContours(this->binaryFrame);

    V3 cpToGround; // closest point to ground
    // closest point to ground in image coordinates
    if(calculateCPToGround(&cpToGround, groundPlane)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(cpToGroundS.x, cpToGroundS.y);
        cv::Size ellipseSize(3,3);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(255, 255, 0), 3);
#endif
    } else {
        printf("closest point to ground is off screen!\r\n");
    }


    // calculate horizontal movement 
    if(dt > 0.001) {
        calculateMovement(grayFrame, dt);
    } else {
        movement = (V2){0.0f, 0.0f};
    }
    IVec2 groundProj(cpToGroundS.x, cpToGroundS.y);
    int idx = findClosestContourCenterIdx(groundProj);
    V2 bestTarget = cpToGroundS;
    if(idx >= 0) {
        bestTarget = (*this->contourCenters2)[idx].imagePoint;
    }
    // closest point to ground (set point error is to this)
    if(loc.forwardDetermined) {
        controlMovementFromLocation(groundProj, dt);
    } else {
        controlMovementToTarget(bestTarget, groundProj, dt, MAX_SPEED);
    }
#ifdef DEBUG_DRAW
    cv::resize(debugImage, debugImageSmall, cv::Size(), 1.0, 1.0, cv::INTER_LINEAR);
    drawTelemetry(debugImageSmall);
#endif // DEBUG_DRAW
    if(distanceToGround > 35.0f) {
        V2 droneWorldMovement = (V2){movement.y, -movement.x};
        loc.update(distanceToGround, this->roll, this->pitch, 0.0f, droneWorldMovement, cpToGroundS, dt, *this->contourCenters2);
    }

#ifdef DEBUG_DRAW
    this->debugImg(debugImageSmall, loc.image);
#endif

    auto temp = prevContourCenters2;
    prevContourCenters2 = contourCenters2;
    contourCenters2 = temp;
    grayFramePrev = grayFrame;
}

// callback when new image is captured by camera
void AutonomousController::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    /*if(!this->active) {
        lastFrame = std::chrono::steady_clock::now();
        return;
    }*/

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    Mat3 camToWorldR;
    camToWorldRotation(&camToWorldR, this->roll, this->pitch);
    mat3_mul(&this->screenToWorld, &camToWorldR, &this->screen2Cam);
    V4 groundPlaneWorld = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
    // ground plane in camera space
    this->groundPlane = calculateGroundPlane(this->distanceToGround, this->roll, this->pitch);

    // TODO: real dt
    double dt = 1.0 / 20.0;
    /*std::chrono::duration<double> dif = start - lastFrame;
    dt = dif.count();*/

    // get cv Mat
    cv_bridge::CvImagePtr cvPtrMono = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat grayFrame = cvPtrMono->image;
    assert(grayFrame.size().width == NATIVE_WIDTH/DOWNSCALE_DIVIDER);
    assert(grayFrame.size().height == NATIVE_HEIGHT/DOWNSCALE_DIVIDER);

    //robotexImage(grayFrame, dt);
    deltaImage(grayFrame, dt);

    /*printf("r %f p %f\r\n", roll, pitch);
    */

    // useless logging
#if LOG_VELOCITY == 1
    time_t tnow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    tm *date = std::localtime(&tnow);
    date->tm_hour = 0;
    date->tm_min = 0;
    date->tm_sec = 0;
    auto midnight = std::chrono::system_clock::from_time_t(std::mktime(date));
    std::chrono::duration<double> dif = std::chrono::system_clock::now() - midnight;

    double time = dif.count();
    logfile.precision(10);
    logfile << time << "," << movement.x/100.0f << "," << "0.0," << movement.y/100.0f << "\n"; 
#endif

    /*imshow("frame", image);
    cv::waitKey(1);*/
	// resize debug image to reduce network load (apparently too slow otherwise)

    if(distanceToGround > 130.0f) {
        velCtrl.setActive(true);
    } else {
        velCtrl.setActive(false);
    }

    // TODO: this doesn't belong here, velocity should be controlled even if theres no images
    this->controlPitch = velCtrl.controlPitch;
    this->controlRoll = velCtrl.controlRoll;

    screenToWorldPrev = screenToWorld;
    lastFrame = std::chrono::steady_clock::now();

#if 1
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::duration<double> duration = end-start;
	totalImagesReceived ++;
	std::chrono::duration<double> fromStartToNow = end-this->startTime;
	printf("imagecallback duration %fms\r\n", duration.count()*1000.0);
	//printf("processing fps %f\r\n", 1.0 / (fromStartToNow.count()/(double)totalImagesReceived));
#endif


    // camera rotation on drone
    // drone rotation
    /*Quat rollQ, pitchQ, yawQ, droneQ;
    quat_angle_axis(&rollQ, -this->roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -this->pitch, PITCH_AXIS);
    quat_angle_axis(&yawQ, 30.0f, (V3){0.0f, 0.0f, 1.0f});
    quat_mul(&droneQ, yawQ, rollQ);
    quat_mul(&rollQ, droneQ, pitchQ);
    droneQ = rollQ; // used rollQ as storage
    loc.showMapFromPoint((V3){500.0f, 250.0f, 845.0f}, droneQ);*/

}

// callback when roll is updated
void AutonomousController::rollCallback(const std_msgs::Float32 msg) {
    this->roll = msg.data;
}

// callback when pitch is updated
void AutonomousController::pitchCallback(const std_msgs::Float32 msg) {
    this->pitch = msg.data;
}

// callback when distance to ground is updated
void AutonomousController::rangeCallback(const std_msgs::Float32 msg) {
    this->distanceToGround = msg.data;
}

// callback when throttle value is updated
void AutonomousController::throttleCallback(const std_msgs::UInt16 msg) {
    this->throttle = msg.data;
}
