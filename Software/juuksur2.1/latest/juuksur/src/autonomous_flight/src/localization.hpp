#include <vector>
#include <float.h>
#include "ttmath.h"
#include "params.hpp"

#include "structures.hpp"
#include "BezierSpline.hpp"
#include <opencv/cv.h>

#define MAP_NODE_COUNT 47

class Localization {
public:
    Localization();
    void reset();
    void init(Mat3& camToScreen, Quat camLocalRot, Quat camInvLocalRot);
    float getVelocityMultiplier(float advance);
    void createMap(double data[][2], int count);
    void showMapFromParticle(LocParticle p, float height, float roll, float pitch, cv::Mat& image);
    void update(float height, float roll, float pitch, float yaw, V2 hMovement, V2 cpToGround, float dt, std::vector<ContourCenter>& cs);
    void drawParticles(std::vector<ContourCenter>& cs, LocParticle bestParticle);
    int getClosestMapEdge(V2 position, V2& cp);
    float getMapT(V2 pos);
    V2 getMapDirection(float t);
    V2 getPositionOnMap(float advance);
    cv::Mat image;
    MapNode map[47];
    BezierSpline smoothMap;
    V3 lastEstimation = (V3){0.0f, 0.0f, 0.0f};
    float lastHeading = 0.0f;

    bool forwardDetermined;
    float forwardT; // 1.0 or -1.0
    float locT;

private:
    void generateLookupTable();
    void resample(std::vector<LocParticle>& wp);
    V3 calculatePositionFromParticles();
    float compareParticleToMeasurement(LocParticle& p, float curHeight, float curPitch, float curRoll, V2 cpToGround, std::vector<ContourCenter>& cs, cv::Mat *debug);

    std::vector<LocParticle> particles;
    std::vector<LocParticle> prevParticles;
    Mat3 camToScreen;
    Quat camLocalRot;
    Quat camInvLocalRot;
    float lastyaw = -1.0f;;
};

