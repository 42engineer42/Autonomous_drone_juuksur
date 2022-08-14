
#include "autonomous_controller.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/UInt16.h>
#include <algorithm>
#include <numeric>
#include <chrono>

#define TTMATH_IMPLEMENTATION
#include "ttmath.h"
#undef TTMATH_IMPLEMENTATION

#define CENTER_DIST_FROM_CONTOUR 45.0f

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SIGNF(x) ((x) >= 0.0f ? 1.0f : -1.0f)

#define DEBUG_DRAW

bool DoesRectangleContainPoint(cv::RotatedRect rectangle, cv::Point2f point) {
    //Get the corner points.
    cv::Point2f corners[4];
    rectangle.points(corners);
    //Convert the point array to a vector.
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<cv::Point2f> contour(corners, lastItemPointer);
    //Check if the point is within the rectangle.
    double indicator = cv::pointPolygonTest(contour, point, false);
    bool rectangleContainsPoint = (indicator >= 0);
    return rectangleContainsPoint;
}

AutonomousController::AutonomousController(std::function<void(cv::Mat&, cv::Mat&)> debugImg) : fTracker(NATIVE_WIDTH/DOWNSCALE_DIVIDER, NATIVE_HEIGHT/DOWNSCALE_DIVIDER) {
    // most likely simulation
#ifdef HARDWARE_SIMULATOR
	float virtualPixelsPerMm = PIXELS_PER_MM / DOWNSCALE_DIVIDER;
	float fx = virtualPixelsPerMm * FOCAL_LENGTH;
	float cx = (float)(NATIVE_WIDTH/(DOWNSCALE_DIVIDER*2)); // basically half width (theoretical!)
	float cy = (float)(NATIVE_HEIGHT/(DOWNSCALE_DIVIDER*2)); 
	// camera space to screen space
	this->cam2Screen = { // column major order
		fx, 0.0f, 0.0f, 0.0f, fx, 0.0f, cx, cy, 1.0f
	};
    this->cvIntrinsic = cv::Mat(3, 3, CV_32F, 0.0f);
    cvIntrinsic.at<float>(0,0) = fx;
    cvIntrinsic.at<float>(1,1) = fx;
    cvIntrinsic.at<float>(0,2) = cx;
    cvIntrinsic.at<float>(1,2) = cy;
    cvIntrinsic.at<float>(2,2) = 1.0f;
	this->screen2Cam = {
		1.0f/fx, 0.0f, 0.0f, 0.0f, 1.0f/fx, 0.0f, -cx/fx, -cy/fx, 1.0f
	};

#else // real drone? use values from calibration
    // RPi camera v2
    /*const float divd = DOWNSCALE_DIVIDER/2;
    float fx = 1288.56369 / divd;
    float fy = 1287.03306 / divd;
    float cx = 807.664125 / divd;
    float cy = 605.320103 / divd;
    */
    // RPi camera v1
    const float divid = DOWNSCALE_DIVIDER;
    float fx = 2567.38554f / divid;
    float fy = 2559.04082f / divid;
    float cx = 1318.783560f / divid;
    float cy = 964.510610f / divid;
	// camera space to screen space
	this->cam2Screen = { // column major order
		fx, 0.0f, 0.0f, 0.0f, fy, 0.0f, cx, cy, 1.0f
	};
    this->cvIntrinsic = cv::Mat(3, 3, CV_32F, 0.0f);
    cvIntrinsic.at<float>(0,0) = fx;
    cvIntrinsic.at<float>(1,1) = fy;
    cvIntrinsic.at<float>(0,2) = cx;
    cvIntrinsic.at<float>(1,2) = cy;
    cvIntrinsic.at<float>(2,2) = 1.0f;
	this->screen2Cam = {
		1.0f/fx, 0.0f, 0.0f, 0.0f, 1.0f/fy, 0.0f, -cx/fx, -cy/fy, 1.0f
	};
#endif
	this->contourCenters = std::unique_ptr<std::vector<ContourCenter>>(new std::vector<ContourCenter>());
	this->prevContourCenters = std::unique_ptr<std::vector<ContourCenter>>(new std::vector<ContourCenter>());
	this->debugImg = debugImg;

    Quat q1, q2, q, invQ;
    quat_angle_axis(&q1, 180.0f, (V3){1.0f, 0.0f, 0.0f});
    quat_angle_axis(&q2, 90.0f, (V3){0.0f, 0.0f, 1.0f});
    quat_mul(&q, q1, q2);
    quat_angle_axis(&q1, -180.0f, (V3){1.0f, 0.0f, 0.0f});
    quat_angle_axis(&q2, -90.0f, (V3){0.0f, 0.0f, 1.0f});
    quat_mul(&invQ, q2, q1);
    loc.init(this->cam2Screen, q, invQ);

}

AutonomousController::~AutonomousController() {
}

void AutonomousController::reset() {
    this->controlPitch = 1500.0f;
    this->controlRoll = 1500.0f;
    this->contourCenters->clear();
    this->prevContourCenters->clear();
    this->contours.clear();
    this->trackIndices.clear();

    this->speedRampup = 0.0f;
    this->horizontalVelocity = (V2){0.0f, 0.0f};
    this->horizontalAcc = (V2){0.0f, 0.0f};

    this->platformLastSeen = (V2){(float)(NATIVE_WIDTH/(DOWNSCALE_DIVIDER*2)), 0.0f};

    this->targetDirection = (V2){0.0f, -1.0f};
    this->cpToGroundS = IMAGE_CENTER;
    this->active = false;
    this->movementRelativeToEnvironment = (V2){0.0f, 0.0f};

    this->controlVelocity = (V2){0.0f, 0.0f};

    this->clearToLand = true;
    this->landing = false;

    this->startTime = std::chrono::steady_clock::now();
    loc.reset();

    fTracker.init();
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

// calculate mathematical infinite plane based on orientation (assuming flat floor)
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
void AutonomousController::camToWorldRotation(Mat3& dst, float roll, float pitch) {
    Quat rollQ, pitchQ, rotQ;
    quat_angle_axis(&rollQ, -roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -pitch, PITCH_AXIS);
    quat_mul(&rotQ, rollQ, pitchQ);
    mat3_rotation(&dst, &rotQ);
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
static bool projectFromScreenToGround(V3& res, V2 screenPoint, V4 groundPlane, Mat3& screenToCam) {
    // homogenous screen coordinate
    V3 screenH = (V3) {screenPoint.x, screenPoint.y, 1.0};
    V3 rayDir;
    mat3_v3_mul(&rayDir, &screenToCam, screenH);
    v3_normalize(&rayDir); // not sure if necessary
    Ray ray;
    ray.origin = (V3){0.0f, 0.0f, 0.0f};
    ray.dir = rayDir;
    if(ray_intersect_plane(&res, ray, groundPlane)) {
        return true;
    }
    return false;
}


// find closest track contour in image coordinates
IVec2 AutonomousController::findClosestContourCenter(IVec2 imagePoint) {
    int count = contourCenters->size();
    float min = 99999.0f;
    int minIdx = -1;
    for(int i = 0; i < count; i++) {
        float xdif = (*contourCenters)[i].imagePoint.x - imagePoint.x;
        float ydif = (*contourCenters)[i].imagePoint.y - imagePoint.y;
        float sqDist = sqrtf(xdif*xdif + ydif*ydif);
        if(sqDist < min) {
            minIdx = i;
            min = sqDist;
        }
    }
    V2 p = (*contourCenters)[minIdx].imagePoint;
    return minIdx >= 0 ? (IVec2){(int)p.x, (int)p.y} : (IVec2){(int)IMAGE_CENTER.x, (int)IMAGE_CENTER.y};
}

// same as above but return index instead of coordinate
int AutonomousController::findClosestContourCenterIdx(IVec2 imagePoint) {
    int count = contourCenters->size();
    float min = 99999.0f;
    int minIdx = -1;
    for(int i = 0; i < count; i++) {
        float xdif = (*contourCenters)[i].imagePoint.x - imagePoint.x;
        float ydif = (*contourCenters)[i].imagePoint.y - imagePoint.y;
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
bool AutonomousController::calculateOpticalFlow(cv::Mat from, cv::Mat to, cv::Mat debugImage, Mat3& screenToWorld, V4 groundPlane, V2& of, float dt) {
    if(from.size() != to.size()) {
        return false;
    }
    static std::vector<cv::KeyPoint> k0;
    static std::vector<cv::Point2f> p0, p1;
    p0.clear();
    p1.clear();
#if 0
    goodFeaturesToTrack(from, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04f);
#else
    cv::FAST(from, k0, 40);
    cv::KeyPoint::convert(k0, p0);
#endif
    if(p0.size() == 0) {
        return false;
    }
    static std::vector<uchar> status;
    status.clear();
    cv::Mat err;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03f);


    cv::calcOpticalFlowPyrLK(from, to, p0, p1, status, err, cv::Size(15,15), 1, criteria);

    // we don't use the homography matrix, but the mask
    // the algorithm uses RANSAC so we can identify outliers
    // this way we should find values that don't fit on the floor plane well
    static std::vector<uchar> mask;
    mask.clear();
    cv::Mat H = cv::findHomography(p0, p1, cv::RANSAC, 3.0, mask);
    static std::vector<cv::Mat> rotations;
    static std::vector<cv::Mat> translations;
    static std::vector<cv::Mat> normals;
    bool foundHomography = false;
    
    if(H.rows == 3 && H.cols == 3) {
        // displacement calculation from homography (very inaccurate because it doesn't know rotation?)
        
        rotations.clear();
        translations.clear();
        normals.clear();
        int dccount = cv::decomposeHomographyMat(H, cvIntrinsic, rotations, translations, normals);
        if(dccount == 4) {
            float bestMatch = -999.0f;
            int bestIndex = 0;
            for(int i = 0; i < 4; i++) {
                // TODO: rotate by roll and pitch!
                V3 comp = (V3){0.0f, 0.0f, -1.0f}; // ground plane should be facing camera
                assert(normals[i].type == CV_64F);
                float nx = normals[i].at<double>(0);
                float ny = normals[i].at<double>(1);
                float nz = normals[i].at<double>(2);
                V3 hnorm = (V3){nx, ny, nz};
                float match = v3_dot(comp, hnorm);
                if(match > bestMatch) {
                    bestMatch = match;
                    bestIndex = i;
                }
            }
            assert(translations[bestIndex].type == CV_64F);
            // NOTE: OpenCV gives depth normalized displacement values
            // so we have to multiply by height and FPS
            float tx = translations[bestIndex].at<double>(0) * distanceToGround;
            float ty = translations[bestIndex].at<double>(1) * distanceToGround;
            float tz = translations[bestIndex].at<double>(2) * distanceToGround;
            testDirty = true;
            testV.x = tx/dt;
            testV.y = ty/dt;
        }
        //std::cout << "t = " << std::endl << " "  << translations[0] << std::endl << std::endl;
        assert(mask.size() == p0.size());
        foundHomography = true;
    }

    fTracker.opticalFlow(p0, p1);

    V2 sum = (V2){0.0f, 0.0f};
    int count = 0;
    int countS = 0;

    int excludedCount = 0;

    float minX = 20.0f;
    float maxX = (float)(NATIVE_WIDTH/DOWNSCALE_DIVIDER) - minX;
    float minY = 20.0f;
    float maxY = (float)(NATIVE_HEIGHT/DOWNSCALE_DIVIDER) - minY;
    for(uint i = 0; i < p0.size(); i++) {
        // filter out points near screen edges
        if(p0[i].x < minX || p0[i].x > maxX || p0[i].y < minY || p0[i].y > maxY 
                || p1[i].x < minX || p1[i].x > maxX || p1[i].y < minY || p1[i].y > maxY) {
            continue;
        }

        // optical flow didn't fail and point is part of homography model
        if(status[i] == 1 && (!foundHomography || mask[i] == 1)) {
            V3 p0Cam, p1Cam;
            countS++;
            // TODO: get rotation delta from gyroscope and use H=K*R*K^(-1) formula to remove rotation between frames
            // then get translation from that
            // this would rely less on roll and pitch being accurate
            // ideally could attach PTS (presentation timestamp) to image and compare it to mmal_port_parameter_get(MMAL_PARAMETER_SYSTEM_TIME) to get exact delays (or calculate exact timestamp within raspi_camera)
            // then integrate gyroscope data
            if(projectFromScreenToGround(p0Cam, (V2){p0[i].x, p0[i].y}, groundPlane, this->screenToWorldPrev)) {
                if(projectFromScreenToGround(p1Cam, (V2){p1[i].x, p1[i].y}, groundPlane, screenToWorld)) {
                    V3 flow;
                    v3_sub(&flow, p1Cam, p0Cam);
                    // we calculate average flow in world space
                    sum.x += flow.x;
                    sum.y += flow.y;
                    count++;
                }
            }
#ifdef DEBUG_DRAW
            cv::line(debugImage, p1[i], p0[i], cv::Scalar(252, 0, 173)); 
#endif
        }
        else if(foundHomography && mask[i] != 1) {
            excludedCount ++;
        }
    }
   
    if(count > 0) {
        sum.x /= (float)count;
        sum.y /= (float)count;
        of = sum;
        return true;
    } else {
        return false;
    }
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
    for(int i = 0; i < contourCenters->size(); i++) {
        closest.push_back(i);
    }
    // find triples of track contours that are in similar direction (and create 2 edges between them)
    for(int j = 0; j < contourCenters->size(); j++) {
        V2 cur = (*contourCenters)[j].imagePoint;
        // sort track contours by distance
        std::sort(closest.begin(), closest.end(), 
                [this, cur](const int& a, const int& b) -> bool
            {
                V2 difa, difb;
                v2_sub(&difa, (*contourCenters)[a].imagePoint, cur);
                v2_sub(&difb, (*contourCenters)[b].imagePoint, cur);
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
                    v2_sub(&toFirst, (*contourCenters)[closest[1]].imagePoint, cur);
                    v2_normalize(&toFirst);
                    // find index with direction different than first was
                    for(int m = 2; m < closest.size()-1; m++) {
                        V2 toM;
                        v2_sub(&toM, (*contourCenters)[closest[m]].imagePoint, cur);
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
                    v2_sub(&toFirst, (*contourCenters)[closest[fidx]].imagePoint, cur);
                    V2 toSecond;
                    v2_sub(&toSecond, (*contourCenters)[closest[k]].imagePoint, cur);
                    float distDif = fabs(v2_len(toFirst) - v2_len(toSecond));
                    v2_normalize(&toFirst);
                    v2_normalize(&toSecond);
                    V2 p = (*contourCenters)[closest[k]].imagePoint;
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
        for(int i = 0; i < contourCenters->size(); i++) {
            for(int j = 0; j < contourCenters->size(); j++) {
                if(i == j) {
                    continue;
                }
                V2 start = (*contourCenters)[i].imagePoint;
                V2 end = (*contourCenters)[j].imagePoint;
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
        V2 p1 = (*contourCenters)[trackIndices[i]].imagePoint;
        V2 p2 = (*contourCenters)[trackIndices[i+1]].imagePoint;
#ifdef DEBUG_DRAW
        cv::line(rgbImage, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 255, 255));
#endif
    }
}

// find goal point on track (for PID), was used for dumb line following
bool AutonomousController::findWaypoint(cv::Mat& rgbImage, V2& wp, V2 cpOnTrack) {
    V2 ints[2];
    V2 bestDirection = this->targetDirection;
    float bestDot = -FLT_MAX;
    bool found = false;
    // intersect track with circle then pick the one in best direction compared to current estimated target direction
    for(int i = 0; i < trackIndices.size(); i+=2) {
        V2 start, end, segDir;
        start = (*contourCenters)[trackIndices[i]].imagePoint;
        end = (*contourCenters)[trackIndices[i+1]].imagePoint;
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
                wp = ints[j];
            }
        }
    }
    // extrapolate from closest point on track (along the line segment)
    if(!found) {
        V2 cpOnTrack;
        int cpEdge;
        if(findClosestPointOnTrack(rgbImage, cpOnTrack, cpEdge)) {
            V2 start = (*contourCenters)[trackIndices[cpEdge]].imagePoint;
            V2 end = (*contourCenters)[trackIndices[cpEdge+1]].imagePoint;
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
                v2_add(&wp, cpOnTrack, dir);
                cv::RotatedRect ellipse(cv::Point(wp.x, wp.y), cv::Size(3,3), 0);
#ifdef DEBUG_DRAW
                cv::ellipse(rgbImage, ellipse, cv::Scalar(0, 0, 255), 3);
#endif
            }
        }
    }
    return found;
}

// calculate closest track point to drone
bool AutonomousController::findClosestPointOnTrack(cv::Mat& rgbImage, V2& point, int& edge) {
    float bestD2 = FLT_MAX;
    bool found = false;
    for(int i = 0; i < trackIndices.size(); i+=2) {
        V2 start, end, dif, segDir;
        start = (*contourCenters)[trackIndices[i]].imagePoint;
        end = (*contourCenters)[trackIndices[i+1]].imagePoint;

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
            point = closest;
            edge = i;
        }
    }
#ifdef DEBUG_DRAW
    cv::RotatedRect ellipse(cv::Point(point.x, point.y), cv::Size(3,3), 0);
    cv::ellipse(rgbImage, ellipse, cv::Scalar(0, 255, 255), 3);
#endif
    return found;
}

// find furthest visible track point from current drone position
bool AutonomousController::findFurthestConnectedNode(cv::Mat& rgbImage, V2& point, int& retNode, V2& dir) {
    // TODO: 1. find connected line segments (to closest point)
    V2 cpOnTrack;
    int edge;
    if(!findClosestPointOnTrack(rgbImage, cpOnTrack, edge)) {
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
            start = (*contourCenters)[trackIndices[i]].imagePoint;
            int other = (i&1) ? i-1 : i+1;
            if(trackIndices[i] == trackIndices[edge]) {
                V2 toOtherEnd;
                otherV = (*contourCenters)[trackIndices[other]].imagePoint;
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
                    dir = segDirCorrected;
                    break;
                }
            }
        }
    } while(found);
    retNode = edge;
    point = best;
    return true;
}

// additional track contour filtering (based on shape and size)
int AutonomousController::filterContour(cv::Mat debugImage, cv::RotatedRect r, double area, float matchedRectArea, float aspect) {
    if(area < MIN_CONTOUR_AREA) { // too small
        //cv::ellipse(debugImage, r, cv::Scalar(0, 100, 255), 3);
        return -1;
    }
    if(area > MAX_CONTOUR_AREA) { // too big
        //cv::ellipse(debugImage, r, cv::Scalar(0, 255, 100), 3);
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
    // calculate projected pixel size in cm^2
    float projectedWidth = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(HORIZONTAL_FOV_RAD/2.0f)*2.0f;
    float pixelWidthCm = projectedWidth/(NATIVE_WIDTH/(float)DOWNSCALE_DIVIDER);
    float projectedHeight = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(VERTICAL_FOV_RAD/2.0f)*2.0f;
    float pixelHeightCm = projectedHeight/(NATIVE_HEIGHT/(float)DOWNSCALE_DIVIDER);
    float projectedPixelArea = pixelWidthCm*pixelHeightCm;

    //grayFrame.convertTo(grayFrame, CV_8UC1, 1.0);
    // remove old contours
    this->contours.clear(); 
    findContours(grayFrame, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    this->contourCenters->clear();
    static std::vector<ContourCenter> centerCanditates;
    centerCanditates.clear();
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
            //cv::line(debugImage, start, start + cv::Point(line[0]*30.0f, line[1]*30.0f), cv::Scalar(200, 255, 100), 1);

            ContourCenter c;
            c.flags = 0;
            cv::Rect bounds = cv::boundingRect(contours[i]);

            // if it touches image edges it might be partial
            c.flags |= (bounds.x == 0 || bounds.x+bounds.width == grayFrame.cols || bounds.y == 0 || bounds.y+bounds.height == grayFrame.rows) ? Contour_Center_Flag_Partial : 0;

            c.imagePoint = (V2){(float)cx, (float)cy};
            c.imageDirection = (V2){line[0], line[1]};
            projectFromScreenToGround(c.camPoint, (V2){(float)cx, (float)cy}, groundPlane, this->screen2Cam);
            projectFromScreenToGround(c.worldPoint, (V2){(float)cx, (float)cy}, groundPlane, screenToWorld);

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
                this->contourCenters->push_back(c);
#ifdef DEBUG_DRAW
                cv::ellipse(debugImage, ellipse, cv::Scalar(255, 0, 0), 3);
#endif
            }
        }
    }

    // find center if it exists
    for(int i = 0; i < centerCanditates.size(); i++) {
        // TODO: find intersecting middle contour (by checking adjacent contours)
        float min = FLT_MAX;
        V3 dif;
        // find closest accepted contour
        for(int j = 0; j < this->contourCenters->size(); j++) {
            auto contour = &(*this->contourCenters)[j];
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
            for(int j = 0; j < this->contourCenters->size(); j++) {
                auto contour = &(*this->contourCenters)[j];
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
            contourCenters->push_back(*cont);
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
    V2 m = (V2){0.0f, 0.0f};
    float t = 0.0f;
    V4 groundPlaneWorld = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
    V2 curMovement = (V2){0.0f, 0.0f};
    bool ofvalid = calculateOpticalFlow(grayFramePrev, grayFrame, debugImage, screenToWorld, groundPlaneWorld, curMovement, dt);

    V2 curMovementPerS = curMovement; 
    curMovementPerS.x /= dt;
    curMovementPerS.y /= dt;

    if(ofvalid) {
        setMovementRelativeToEnvironment(curMovementPerS);
    }
}

// find goal point (for PID)
V2 AutonomousController::findTargetPoint() {
    // choose target point to move towards
    V2 bestTarget, waypoint, cpOnTrack;
    int dummy;
    findClosestPointOnTrack(debugImage, bestTarget, dummy);
    bool cpot = findClosestPointOnTrack(debugImage, cpOnTrack, dummy);
    if(findWaypoint(debugImage, waypoint, cpOnTrack)) {
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
    } else if(this->contourCenters->size() > 0) {
    bestTarget = (*this->contourCenters)[0].imagePoint;
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
    if(findClosestPointOnTrack(debugImage, cpOnTrack, cpEdge)) {
        V2 start = (*contourCenters)[trackIndices[cpEdge]].imagePoint;
        V2 end = (*contourCenters)[trackIndices[cpEdge+1]].imagePoint;
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

void AutonomousController::controlMovementToTarget(V2 targetPosS, IVec2 refPosS, double dt, float maxSpeed, bool useFgMovement) {
    // TODO: find offset vector and convert it into IMU coordinate system

    // to world coordinates
    V3 targetPosW = (V3){targetPosS.x, targetPosS.y, 0.0f};
    V3 curPosW = (V3){(float)refPosS.x, (float)refPosS.y, 0.0f};
    V4 groundPlane = (V4){0.0f, 0.0f, -1.0f, distanceToGround};

    bool success = true;

    success &= projectFromScreenToGround(targetPosW, targetPosS, groundPlane, this->screenToWorld);
    success &= projectFromScreenToGround(curPosW, (V2){(float)refPosS.x, (float)refPosS.y}, groundPlane, this->screenToWorld);

    float targetxspeed = 0.0f;
    float targetyspeed = 0.0f;

    if(success) {
        float errorx = targetPosW.x - curPosW.x;
        float errory = targetPosW.y - curPosW.y;

        // clamp speed
        float targetSpeed = sqrtf(errorx*errorx + errory*errory);
        if(targetSpeed > maxSpeed) {
            float mul = std::abs(maxSpeed/targetSpeed);
            errorx *= mul;
            errory *= mul;
        }

        // we have calculated error in world space, the velocity control system on
        //   the microcontroller uses IMU space (accelerometer axises)
        // world x - drone sideways
        // world y - drone forwards 
        // IMU x - drone forwards
        // IMU y - drone sideways
        
        targetxspeed = 0.5f*errory;
        targetyspeed = 0.5f*errorx;
    }
    
    //this->controlVelocity = (V2) {-targetxspeed, 0.0f};
    //this->controlVelocity = (V2) {0.0f, -targetyspeed};
    this->controlVelocity = (V2) {-targetxspeed, -targetyspeed};
}

#if true != true

// move towards point on camera image
void AutonomousController::controlMovementToTarget(V2 targetPosS, IVec2 curPosS, double dt, float maxSpeed, bool useFgMovement) {
    V2 droneMovement;
    if(!useFgMovement) {
        droneMovement = (V2){-movement.x, -movement.y};
    } else {
        droneMovement = (V2){-fgMovement.x, -fgMovement.y};
    }
    // to world coordinates
    V3 targetPosW = (V3){targetPosS.x, targetPosS.y, 0.0f};
    V3 curPosW = (V3){(float)curPosS.x, (float)curPosS.y, 0.0f};
    V4 groundPlane = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
    projectFromScreenToGround(targetPosW, targetPosS, groundPlane, this->screenToWorld);
    projectFromScreenToGround(curPosW, (V2){(float)curPosS.x, (float)curPosS.y}, groundPlane, this->screenToWorld);

    // TODO: use /control/velocity
    ROS_ERROR("controlMovementToTarget() not reimplemented!");
    
    //velCtrl.update(vdir, speed, droneMovement, dt);
#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetxspeed, curPosS.y+targetyspeed), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x), curPosS.y+(droneMovement.y)), cv::Scalar(0, 255, 255));
#endif
}

#endif

float powsum(std::vector<float>& v, float p) {
    float sum = 0.0f;
    for(int i = 0; i < v.size(); i++) {
        sum += powf(v[i], p);
    }
    return sum;
}

float powmsum(std::vector<float>& v, std::vector<float>& m, float p) {
    float sum = 0.0f;
    for(int i = 0; i < v.size(); i++) {
        sum += powf(v[i], p)*m[i];
    }
    return sum;
}

// least squares polynomial fit (only one degree of freedom - x^2)
V2 calculateOptimalAcceleration(std::vector<float>& timeV, std::vector<float> xV, std::vector<float>yV) {
    float ata = powsum(timeV, 4.0f);
    float y2x = powmsum(timeV, xV, 2.0f);
    float y2y = powmsum(timeV, yV, 2.0f);
    float cx = (1 / ata) * y2x;
    float cy = (1 / ata) * y2y;
    return (V2){cx*2.0f, cy*2.0f};
}

// move along track based on current position on map
void AutonomousController::controlMovementFromLocation(IVec2 curPosS, double dt) {
    V2 droneMovement2 = (V2){-movementRelativeToEnvironment.x, -movementRelativeToEnvironment.y};
    V2 droneMovement = (V2){-this->horizontalVelocity.y, -this->horizontalVelocity.x};
    // forward with 0 heading on image
    V2 forward = (V2){0.0f, -1.0f};
    V2 down = (V2){1.0f, 0.0f};
    // TODO: this assumes that heading is always the same but it might not be (gyro heading drifts)
    float heading = -90.0f * loc.forwardT;

    // in map coordinates
    float lookahead = (v2_len(droneMovement)/LOOKAHEAD_MAX_SPEED)*LOOKAHEAD_MULTIPLIER;
    this->lookAhead = lookahead;
    V2 targetLocation = loc.getPositionOnMap(lookahead); // 4.0 - look ahead (1.0 = distance between contours)

    float velMultiplier = loc.getVelocityMultiplier(lookahead);
    //float velMultiplier = 1.0f;
    float speed;
    float curSpeed = v2_len(droneMovement);
    if(loc.forwardDetermined && !this->landing) {
        speedRampup += dt / SPEED_RAMPUP_DURATION;
        speedRampup = CLAMP(speedRampup, 0.0f, 1.0f);
        speed = MAX_SPEED * speedRampup * velMultiplier;
        clearToLand = false;
    } else if(this->landing) {
        speedRampup -= dt / SPEED_RAMPUP_DURATION;
        speedRampup = CLAMP(speedRampup, 0.0f, 1.0f);
        speed = MAX_SPEED * speedRampup * velMultiplier;
        if(speedRampup == 0.0f) {
            clearToLand = true;
        } else {
            clearToLand = false;
        }
    } else {
        speed = 0.0f;
        speedRampup = 0.0f;
        clearToLand = true;
    }

    V2 currentLocation = (V2){loc.lastEstimation.x, loc.lastEstimation.y};
    V2 dirToTarget = targetLocation - currentLocation;
    dirToTarget = v2_rotate(dirToTarget, TT_DEG2RAD_F*heading);
    v2_normalize(&dirToTarget);

    V2 worldMovement = v2_rotate(droneMovement, -TT_DEG2RAD_F*heading);

    V2 screenDir;
    screenDir.x = dirToTarget.x * forward.x + dirToTarget.y * down.x;
    screenDir.y = dirToTarget.x * forward.y + dirToTarget.y * down.y;

    // for going slower in turns
    //float velMultiplier = loc.getVelocityMultiplier(0.0f);

    this->targetDirection = screenDir;

    this->controlVelocity = (V2) {-screenDir.y * speed, -screenDir.x * speed};

#ifdef DEBUG_DRAW

    float projectedWidth = (distanceToGround+(FOCAL_LENGTH/10.0f))*tanf(HORIZONTAL_FOV_RAD/2.0f)*2.0f;
    float pixelWidthCm = projectedWidth/(NATIVE_WIDTH/(float)DOWNSCALE_DIVIDER);
    //cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+screenDir.x*velCtrl.curVelocity, curPosS.y+screenDir.y*velCtrl.curVelocity), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x*pixelWidthCm), curPosS.y+(droneMovement.y*pixelWidthCm)), cv::Scalar(0, 255, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement2.x*pixelWidthCm), curPosS.y+(droneMovement2.y*pixelWidthCm)), cv::Scalar(255, 255, 0));
    //cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(toTrack.x*10), curPosS.y+(toTrack.y*10)), cv::Scalar(0, 80, 255));
    //cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(targetDMap.x*10), curPosS.y+(targetDMap.y*10)), cv::Scalar(0, 255, 80));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(targetDirection.x*speed*pixelWidthCm), curPosS.y+(targetDirection.y*speed*pixelWidthCm)), cv::Scalar(0, 255, 0));
    // draw curve considering acceleration
    const int steps = 50;
    const float timestep = 0.02f; // in seconds
    float startX = curPosS.x;
    float startY = curPosS.y;
    float hspeedX = droneMovement.x;
    float hspeedY = droneMovement.y;
    for(int i = 0; i < steps; i++) {
        float endX = startX + pixelWidthCm*(timestep*hspeedX- horizontalAcc.y*timestep*timestep*0.5f);
        float endY = startY + pixelWidthCm*(timestep*hspeedY- horizontalAcc.x*timestep*timestep*0.5f);

        cv::line(debugImage, cv::Point(startX, startY), cv::Point(endX, endY), cv::Scalar(0, 0, 255));

        startX = endX;
        startY = endY;
        hspeedX += -timestep*horizontalAcc.y;
        hspeedY += -timestep*horizontalAcc.x;
    }
    // draw map
    V3 dp3 = loc.lastEstimation;
    // current drone position
    V2 dp = (V2){dp3.x, dp3.y};
    V2 start = loc.getPositionOnMap(0.0f);
    V2 startLocal;
    V4 dpWorld = (V4){dp.x, dp.y, 0.0f, 1.0f};
    V4 dpScreen;
    mat4_v4_mul(&dpScreen, &loc.worldToScreen, dpWorld);
    dpScreen.x /= dpScreen.z;
    dpScreen.y /= dpScreen.z;
    cv::circle(debugImage, cv::Point(dpScreen.x, dpScreen.y), 3, cv::Scalar(0, 255, 255));
    v2_sub(&startLocal, start, dp);
    std::vector<float> ts, xs, ys;
    for(int i = 1; i < 20; i++) {
        // up to 2 seconds ahead
        V2 end = loc.getPositionForwardCm(i*speed*0.1f);
        V2 endLocal;
        v2_sub(&endLocal, end, dp);

        ts.push_back(0.1f*i);
        xs.push_back(endLocal.x - worldMovement.x);
        ys.push_back(endLocal.y - worldMovement.y);

        V4 startWorld = (V4){start.x, start.y, 0.0f, 1.0f};
        V4 endWorld = (V4){end.x, end.y, 0.0f, 1.0f};
        V4 startScreen, endScreen;
        mat4_v4_mul(&startScreen, &loc.worldToScreen, startWorld);
        mat4_v4_mul(&endScreen, &loc.worldToScreen, endWorld);
        startScreen.x /= startScreen.z;
        startScreen.y /= startScreen.z;
        endScreen.x /= endScreen.z;
        endScreen.y /= endScreen.z;
        cv::line(debugImage, cv::Point(startScreen.x, startScreen.y), cv::Point(endScreen.x, endScreen.y), cv::Scalar(0, 255, 255));
        start = end;
    }
    V2 optAcc = calculateOptimalAcceleration(ts, xs, ys);
    // TODO: get this from world space to screen space somehow (without changing units?)
    //optAcc = v2_rotate(optAcc, TT_DEG2RAD_F*heading);
    startX = curPosS.x;
    startY = curPosS.y;
    hspeedX = droneMovement.x;
    hspeedY = droneMovement.y;
    for(int i = 0; i < steps; i++) {
        float endX = startX + pixelWidthCm*(timestep*hspeedX+ optAcc.x*timestep*timestep*0.5f);
        float endY = startY + pixelWidthCm*(timestep*hspeedY+ optAcc.y*timestep*timestep*0.5f);

        cv::line(debugImage, cv::Point(startX, startY), cv::Point(endX, endY), cv::Scalar(255, 0, 0));

        startX = endX;
        startY = endY;
        hspeedX += timestep*optAcc.x;
        hspeedY += timestep*optAcc.y;
    }

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
    V2 droneMovement = (V2){-movementRelativeToEnvironment.x, -movementRelativeToEnvironment.y};
    V2 targetVelocity = targetDirection;

    // from current position to track
    V2 offsetFromBias;
    v2_sub(&offsetFromBias, biasPoint, (V2){(float)curPosS.x, (float)curPosS.y});
    float biasScale = CLAMP(v2_len(offsetFromBias)/MAX_OFFSET_FROM_TRACK, 0.0f, 1.0f)*0.25f;
    v2_normalize(&offsetFromBias);
    float det = targetDirection.x*offsetFromBias.y-offsetFromBias.x*targetDirection.y;
    float angle = (det>=0.0f?1.0f:-1.0f)*biasScale;
    targetVelocity.x = targetVelocity.x*cosf(angle)-targetVelocity.y*sinf(angle);
    targetVelocity.y = targetVelocity.x*sinf(angle)+targetVelocity.y*cosf(angle);

    V2 mdir = droneMovement;
    v2_normalize(&mdir);
    float targetSpeed =  MAX_DIRECTIONAL_SPEED*fabs(v2_dot(mdir, targetVelocity));
    //targetVelocity = v2_scale(targetVelocity, targetSpeed);

#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetVelocity.x, curPosS.y+targetVelocity.y), cv::Scalar(0, 0, 255));
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+(droneMovement.x), curPosS.y+(droneMovement.y)), cv::Scalar(0, 255, 255));
#endif
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
    sprintf(buf, "SPD: %.2f", v2_len(this->horizontalVelocity));
    cv::putText(debugImage, buf, cv::Point(20, 75), cv::FONT_HERSHEY_SIMPLEX, 0.25, textColor);
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
        ROS_WARN("closest point to ground is off screen!\r\n");
    }

    // calculate horizontal movement 
    if(dt > 0.001) {
        calculateMovement(grayFrame, dt);
    }    
    IVec2 groundProj(cpToGroundS.x, cpToGroundS.y);

    this->targetDirection = findTargetDirection();
    IVec2 curPosS(cpToGroundS.x, cpToGroundS.y);
#ifdef DEBUG_DRAW
    cv::line(debugImage, cv::Point(curPosS.x, curPosS.y), cv::Point(curPosS.x+targetDirection.x*100, curPosS.y+targetDirection.y*100), cv::Scalar(0, 255, 0));
#endif // DEBUG_DRAW
    
    V2 cpOnTrack;
    int dummy;

    if(!findClosestPointOnTrack(debugImage, cpOnTrack, dummy)) {
        cpOnTrack = (V2){(float)curPosS.x, (float)curPosS.y};
    }

    V2 furthestPoint = cpOnTrack;
    V2 furthestDirection = this->targetDirection;
    if(findFurthestConnectedNode(debugImage, furthestPoint, dummy, furthestDirection)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(furthestPoint.x, furthestPoint.y);
        cv::Size ellipseSize(3,3);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(80, 155, 80), 3);
#endif // DEBUG_DRAW
    }
    controlMovementToDirection(furthestDirection, curPosS, cpOnTrack, dt);

#ifdef DEBUG_DRAW
    drawTelemetry(debugImage);
    this->debugImg(debugImage, loc.image);
#endif

    prevContourCenters.swap(contourCenters);
    grayFramePrev = grayFrame;
}

void AutonomousController::robotexImage(cv::Mat& grayFrame, double dt) {
    threshold(grayFrame, this->binaryFrame, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //threshold(grayFrame, this->binaryFrame, 70, 255, CV_THRESH_BINARY);
    cvtColor(this->binaryFrame, this->debugImage, CV_GRAY2BGR);
    updateContours(this->binaryFrame); // 0.1ms on PC

    V3 cpToGround; // closest point to ground
    // closest point to ground in image coordinates
    if(calculateCPToGround(&cpToGround, groundPlane)) {
        cpToGroundS = worldPointToScreen(cpToGround, &cam2Screen);
        cv::Point cvPoint(cpToGroundS.x, cpToGroundS.y);
        cv::Size ellipseSize(6,6);
        cv::RotatedRect ellipse(cvPoint, ellipseSize, 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, ellipse, cv::Scalar(255, 255, 0), 3);
#endif
    } else {
        ROS_WARN("Closest point to ground is off screen!\r\n");
    }

    // calculate horizontal movement 
    if(dt > 0.001) {
        calculateMovement(grayFrame, dt);
    }

    IVec2 groundProj(cpToGroundS.x, cpToGroundS.y);
    int idx = findClosestContourCenterIdx(groundProj);
    V2 bestTarget = cpToGroundS;
    if(idx >= 0) {
        bestTarget = (*this->contourCenters)[idx].imagePoint;
    }

    fTracker.setCamHeight(distanceToGround);
    if(!fTracker.isTracking()) {
        fTracker.pickAndStartTrackingFeature();
    }

    V2 trackedFeatureSP, trackedFeature;
    trackedFeatureSP = cpToGroundS;
    trackedFeature = cpToGroundS;
    if(fTracker.getTrackedFeature(trackedFeatureSP, trackedFeature)) {
        cv::Point tfspcv(trackedFeatureSP.x, trackedFeatureSP.y);
        cv::Point tfcv(trackedFeature.x, trackedFeature.y);
        cv::RotatedRect e1(tfspcv, cv::Size(3,3), 0);
        cv::RotatedRect e2(tfcv, cv::Size(6,6), 0);
#ifdef DEBUG_DRAW
        cv::ellipse(debugImage, e1, cv::Scalar(0, 0, 100), 3);
        cv::ellipse(debugImage, e2, cv::Scalar(0, 0, 255), 3);
#endif
    }

#if 1 // normal track code
    // start moving as soon as we have a decent understanding where we are facing
    if(loc.forwardDetermined) {
        controlMovementFromLocation(groundProj, dt);
    } else {
        controlMovementToTarget(trackedFeature, (IVec2){(int)trackedFeatureSP.x, (int)trackedFeatureSP.y}, dt, MAX_SPEED, false);
    }
#else // hover in place
    controlMovementToTarget(trackedFeature, groundProj, dt, MAX_SPEED, false);
#endif

#ifdef DEBUG_DRAW
    drawTelemetry(debugImage);
#endif // DEBUG_DRAW

    if(distanceToGround > 35.0f) {
        // NOTE: world movment at 0 degrees
        //V2 droneWorldMovement = (V2){movement.y, -movement.x};
        V2 droneWorldMovement = (V2){this->horizontalVelocity.x, -this->horizontalVelocity.y};
        loc.update(distanceToGround, this->roll, this->pitch, 0.0f, droneWorldMovement, dt, *this->contourCenters);

        loc.drawParticles(*this->contourCenters, this->lookAhead);
    }

#ifdef DEBUG_DRAW
    this->debugImg(debugImage, loc.image);
#endif

    prevContourCenters.swap(contourCenters);
    grayFramePrev = grayFrame;
}

// callback when new image is captured by camera
void AutonomousController::imageCallback(const dev_msgs::CameraFrame::ConstPtr& msg)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    double dt = 1.0 / 40.0;

    Mat3 camToWorldR;
    camToWorldRotation(camToWorldR, this->roll, this->pitch);
    mat3_mul(&this->screenToWorld, &camToWorldR, &this->screen2Cam);
    V4 groundPlaneWorld = (V4){0.0f, 0.0f, -1.0f, distanceToGround};
    // ground plane in camera space
    this->groundPlane = calculateGroundPlane(this->distanceToGround, this->roll, this->pitch);

    // get cv Mat
    cv_bridge::CvImagePtr cvPtrMono = cv_bridge::toCvCopy((*msg).image, sensor_msgs::image_encodings::MONO8);
    cv::Mat grayFrame = cvPtrMono->image;
    assert(grayFrame.size().width == NATIVE_WIDTH/DOWNSCALE_DIVIDER);
    assert(grayFrame.size().height == NATIVE_HEIGHT/DOWNSCALE_DIVIDER);

    ros::Time pts = (*msg).presentationTimestamp;
    // for some reason Unity just doesn't understand how time works
#ifdef HARDWARE_RASPI_3
    double dtSec = (pts - lastPTS).toSec();
    if(dtSec > 0.0 && dtSec < 1.0) {
        dt = dtSec;
    }
#endif

    robotexImage(grayFrame, dt);
    //deltaImage(grayFrame, dt);

    screenToWorldPrev = screenToWorld;
    lastFrame = std::chrono::steady_clock::now();

    // delta time measurement
#if 1
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::duration<double> duration = end-start;
	totalImagesReceived ++;
	std::chrono::duration<double> fromStartToNow = end-this->startTime;
	ROS_INFO("imagecallback duration %fms\r\n", duration.count()*1000.0);
#endif
    this->lastPTS = pts;
}

// callback when roll is updated
void AutonomousController::rollCallback(const std_msgs::Float32 msg) {
    this->roll = msg.data;
}

// callback when pitch is updated
void AutonomousController::pitchCallback(const std_msgs::Float32 msg) {
    this->pitch = msg.data;
}

void AutonomousController::velXCallback(const std_msgs::Float32 msg) {
    this->horizontalVelocity.x = msg.data;
}

void AutonomousController::velYCallback(const std_msgs::Float32 msg) {
    this->horizontalVelocity.y = msg.data;
}
void AutonomousController::accXCallback(const std_msgs::Float32 msg) {
    this->horizontalAcc.x = msg.data;
}
void AutonomousController::accYCallback(const std_msgs::Float32 msg) {
    this->horizontalAcc.y = msg.data;
}

// callback when distance to ground is updated
void AutonomousController::rangeCallback(const std_msgs::Float32 msg) {
    if(msg.data < 5) {
        this->distanceToGround = 5;
    } else {
        this->distanceToGround = msg.data;
    }
}

void AutonomousController::setMovementRelativeToEnvironment(V2 movement) {
    this->movementRelativeToEnvironment = movement;
    movementRelEDirty = true;
}
