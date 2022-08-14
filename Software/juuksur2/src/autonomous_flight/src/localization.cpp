#include "localization.hpp"

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <random>
#include <assert.h>
#include <math.h>
#include <chrono>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SIGNF(x) ((x)>=0?1.0f:-1.0f)

// Map of Robotex drone race track (in meters)
static double landmarks[MAP_NODE_COUNT][2] =
    {{5.0, 2.493}, {5.329, 2.28}, {5.689, 2.082}, {6.078, 1.898}, {6.437, 1.686}, {6.796, 1.473}, {7.201, 1.289}, {7.605, 1.204}, {8.024, 1.261}, {8.398, 1.416}, {8.713, 1.7}, {8.907, 2.054}, {8.982, 2.45}, {8.922, 2.861}, {8.713, 3.215}, {8.443, 3.499}, {8.054, 3.683}, {7.65, 3.739}, {7.231, 3.669}, {6.841, 3.484}, {6.467, 3.286}, {6.093, 3.088}, {5.734, 2.875}, {5.374, 2.691}, {4.626, 2.28}, {4.192, 2.096}, {3.877, 1.87}, {3.518, 1.671}, {3.144, 1.473}, {2.769, 1.289}, {2.365, 1.204}, {1.946, 1.261}, {1.557, 1.431}, {1.257, 1.7}, {1.048, 2.054}, {0.988, 2.45}, {1.048, 2.833}, {1.228, 3.187}, {1.512, 3.47}, {1.901, 3.669}, {2.32, 3.739}, {2.754, 3.669}, {3.129, 3.499}, {3.488, 3.3}, {3.847, 3.102}, {4.237, 2.89}, {4.611, 2.691}};

// suggested velocity multiplier at track vertices
/*static float velMults[MAP_NODE_COUNT] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};*/

static float velMults[MAP_NODE_COUNT] = {
    1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 
    0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.9f, 1.0f, 1.0f, 
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.7f, 0.7f, 
    0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 
    0.9f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};


#define LOOKUP_TABLE_WIDTH 500
#define LOOKUP_TABLE_HEIGHT 250

// voronoi diagram of track edges
static uint8_t proximityLookup[LOOKUP_TABLE_HEIGHT][LOOKUP_TABLE_WIDTH];


Localization::Localization( ){   
    createMap(landmarks, MAP_NODE_COUNT);
}

// generate voronoi diagram of track edges
void Localization::generateLookupTable() {
    int h,w,i;
    const float scale = 2.0f;
    for(h = 0; h < LOOKUP_TABLE_HEIGHT; h++) {
        for(w = 0; w < LOOKUP_TABLE_WIDTH; w++) {
            V2 pos = (V2){w*scale, h*scale};
            int minIdx = 0;
            float min = FLT_MAX;
            for(i = 1; i < MAP_NODE_COUNT; i++) {
                V2 start = (V2){map[i].pos.x, map[i].pos.y};
                V3 *end3 = &map[(i+1)%MAP_NODE_COUNT].pos;
                V2 end = (V2){end3->x, end3->y};
                V2 p = closest_point_on_line_seg(pos, start, end);  
                float xdif = p.x - pos.x;
                float ydif = p.y - pos.y;
                float d2 = xdif*xdif + ydif*ydif;
                if(d2 < min) {
                    min = d2;
                    minIdx = i;
                }
            }
            proximityLookup[h][w] = minIdx;
        }
    }
}

// sample particles completely randomly
LocParticle sampleRandom() {
    LocParticle p;
    p.pos.x = rand() % 1000;
    p.pos.y = rand() % 500;
    p.heading = rand() % 360;
    //p.heading = 330.0f;
    return p;
}

// sample particles given approximate start position
LocParticle sampleInitial() {
    LocParticle p;
    p.pos = (V2){120.0f, 250.0f};
    // TODO: drone will be placed with biased direction
    p.heading = rand() % 360;
    p.heading -= 180;
    static std::default_random_engine generator;
    std::normal_distribution<float> nd(0.0, 30.0f);
    p.pos.x += nd(generator);
    p.pos.y += nd(generator);
    return p;
}

void Localization::init(Mat3& camToScreen, Quat camLocalRot, Quat camInvLocalRot) {
    this->camToScreen = camToScreen;
    this->camLocalRot = camLocalRot;
    this->camInvLocalRot = camInvLocalRot;
    reset();
}

void Localization::reset() {
    prevParticles.clear();
    for(int i = 0; i < PARTICLE_COUNT; i++) {
        prevParticles.push_back(sampleInitial());
    }
    // middle point of left track edge is at around 35th node
    locT = 35.0f;
    forwardDetermined = false;
    forwardT = 0.0f;
}

// generate bezier curve map and precompute static values
void Localization::createMap(double data[][2], int count) {
    V2 v2map[MAP_NODE_COUNT];
    for(int i = 0; i < MAP_NODE_COUNT; i++) {
        v2map[i] = (V2){(float)landmarks[i][0]*100.0f, (float)landmarks[i][1]*100.0f};
    }
    for(int i = 0; i < count; i++) {
        MapNode *node = this->map + i;
        node->pos = (V3){(float)data[i][0]*100.0f, (float)data[i][1]*100.0f, 0.0f};
        int mod = (i-1) % count;
        node->prev = mod < 0 ? count - mod : mod;
        node->next = (i+1)%count;
        V2 nextPos = (V2){(float)data[node->next][0]*100.0f, (float)data[node->next][1]*100.0f};
        V2 curPos = (V2){node->pos.x, node->pos.y};
        v2_sub(&node->dir, nextPos, curPos);
        v2_normalize(&node->dir);
    }
    generateLookupTable();
    smoothMap.Init(v2map, MAP_NODE_COUNT, true);
    image = cv::Mat::zeros(125, 250, CV_8UC3);
}

// debug draw map from particle's point of view (for debugging)
void Localization::showMapFromParticle(LocParticle p, float height, float roll, float pitch, cv::Mat& image) {
    Mat4 worldToCam;

    Quat rot, rollQ, pitchQ, yawQ;
    quat_angle_axis(&rollQ, -roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -pitch, PITCH_AXIS);
    quat_angle_axis(&yawQ, p.heading, (V3){0.0f, 0.0f, -1.0f});
    quat_mul(&rot, yawQ, rollQ);
    quat_mul(&rollQ, rot, pitchQ);
    rot = rollQ;

    Quat totalRot;
    V3 pos = v3_scale((V3){p.pos.x, p.pos.y, height}, -1.0f);
    quat_mul(&totalRot, this->camLocalRot, rot);
    totalRot.w *= -1.0f;
    mat4_rt(&worldToCam, totalRot, pos);

    Mat3 camToScreen = this->camToScreen;
    camToScreen.m11 *= -1.0f;

    std::vector<V2> screenPoints;
    for(int i = 0; i < MAP_NODE_COUNT; i++) {
        V3 ppos = map[i].pos;
        V4 worldPoint = (V4) { ppos.x, ppos.y, ppos.z, 1.0f};
        V4 camPoint;
        mat4_v4_mul(&camPoint, &worldToCam, worldPoint); 
        V3 camPoint3 = (V3){camPoint.x, camPoint.y, camPoint.z};
        V3 screenPoint3;
        if(camPoint3.z > 0.05f) {
            mat3_v3_mul(&screenPoint3, &camToScreen, camPoint3);
            V2 sp = (V2){screenPoint3.x/screenPoint3.z, screenPoint3.y/screenPoint3.z};
            screenPoints.push_back(sp);
        }
    }
    for(int i = 0; i < screenPoints.size(); i++) {
        V2 p = screenPoints[i];
        cv::circle(image, cv::Point((int)p.x, (int)p.y), 3, cv::Scalar(255, 0, 0), -1);
    }
}

// clamp map index (map is a loop)
int clampMapIndex(int idx) {
    idx %= MAP_NODE_COUNT;
    if(idx < 0) {
        idx = MAP_NODE_COUNT + idx;
    }
    assert(idx >= 0 && idx < MAP_NODE_COUNT);
    return idx;
}

// TODO: this is wrong, movement should be with uncertainty
LocParticle updateParticle(float yaw, V2 hMovement, double dt, LocParticle p) {
    // TODO: trapezoid rule?
    float cosine = cosf(TT_DEG2RAD_F*p.heading);
    float sine = sinf(TT_DEG2RAD_F*p.heading);
    float xd = cosine*hMovement.x - sine*hMovement.y;
    float yd = sine*hMovement.x + cosine*hMovement.y;
    p.pos.x += xd*dt;
    p.pos.y += yd*dt;
    p.heading += yaw;
    
    return p;
}

// calculate probablity 0..1 that the particle is the actual drone position
float Localization::compareParticleToMeasurement(LocParticle& p, float height, float pitch, float roll, std::vector<ContourCenter>& cs, cv::Mat *debug) {
    Mat4 camToWorld;
    Quat rot, rollQ, pitchQ, yawQ;
    quat_angle_axis(&rollQ, -roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -pitch, PITCH_AXIS);
    quat_angle_axis(&yawQ, p.heading, (V3){0.0f, 0.0f, -1.0f});
    quat_mul(&rot, yawQ, rollQ);
    quat_mul(&rollQ, rot, pitchQ);
    rot = rollQ;
    Quat totalRot;
    quat_mul(&totalRot, this->camLocalRot, rot);

    mat4_tr(&camToWorld, (V3){p.pos.x, p.pos.y, height}, totalRot);

    float sumX = 0.0f;
    float sumY = 0.0f;
    float minX;
    float minY;
    int ccount = cs.size();
    for(int i = 0; i < ccount; i++) {
        // NOTE: no idea why x coordinate is inverted, probably camera is mirrored or something?
        // its probably the y axis on image, because we rotate cameara by 90 degrees
        ContourCenter *cc = &cs[i];
        V3 *cap = &cc->camPoint;
        V4 posInCam = (V4) {-cap->x, cap->y, cap->z, 1.0f};
        V4 posInWorld;
        mat4_v4_mul(&posInWorld, &camToWorld, posInCam);

        if(debug != NULL) {
            cv::circle(*debug, cv::Point((int)posInWorld.x, (int)posInWorld.y), 3, cv::Scalar(255, 0, 0), -1);
        }

        int x = CLAMP((int)(posInWorld.x/2.0f), 0, LOOKUP_TABLE_WIDTH-1);
        int y = CLAMP((int)(posInWorld.y/2.0f), 0, LOOKUP_TABLE_HEIGHT-1);
        int closest = proximityLookup[y][x];
        V2 start = (V2){map[closest].pos.x, map[closest].pos.y};
        V3 *end3 = &map[(closest+1)%MAP_NODE_COUNT].pos;
        V2 end = (V2){end3->x, end3->y};
        V2 p2 = closest_point_on_line_seg((V2){posInWorld.x, posInWorld.y}, start, end);
        float xdif = p2.x - posInWorld.x;
        float ydif = p2.y - posInWorld.y;
        sumX += fabs(xdif)/**xdif*/;
        sumY += fabs(ydif)/**ydif*/;
    }
    if(ccount > 0) {
        sumX /= (float)ccount;
        sumY /= (float)ccount;
    }
    //float matchScalar = 1.0f - std::min(1.0f, sum/4096.0f);
    float matchScalarX = 1.0f - std::min(1.0f, sumX/100.0f);
    float matchScalarY = 1.0f - std::min(1.0f, sumY/100.0f);

    return matchScalarX*matchScalarY;
}

// resample particles
// TODO: uncertainty should be at particle movement not sampling!?
void Localization::resample(std::vector<LocParticle>& wp) {
    particles.clear();
    // TODO: cache data structures to avoid allocations
    float wsum = 0.0f;
    std::vector<float> sumArr;
    for(int i = 0; i < wp.size(); i++) {
        wsum += wp[i].weight;
        sumArr.push_back(wsum);
    }
    std::vector<float> randoms;
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_real_distribution<float> distr(0.0, wsum);
    for(int i = 0; i < PARTICLE_COUNT-RANDOM_PARTICLES; i++) {
        randoms.push_back(distr(eng));
    }
    std::sort(randoms.begin(), randoms.end());
    int i = 0;
    int head = 0;
    // NOTE: this should be O(n) (thus highest complexity is the sort)
    while(i < randoms.size()) {
        float cur = randoms[i];
        while(sumArr[head] < cur) {
            head++;
            assert(head < wp.size());
        }
        assert(sumArr[head] >= cur);
        // not neccessary, but just in case
        if(head >= wp.size()) {
            head = wp.size() - 1;
        }
        // TODO: sample from distribution near particle not the perticle itself
        auto& p = wp[head];
        //if(p.pos.x >= -30.0f && p.pos.x < 1030 && p.pos.y >= -30.0f && p.pos.y < 530) {
            static std::default_random_engine generator;
            std::normal_distribution<float> nd(0.0, 2.5f/p.weight);
            p.pos.x += nd(generator);
            p.pos.y += nd(generator);
            p.heading += nd(generator)*0.5f;
            p.heading = fmodf(p.heading, 360.0f);
            if(p.heading < 0.0f) {
                p.heading = 360.0f + p.heading;
            }
            particles.push_back(p);
        //} else {
            //particles.push_back(sampleRandom());
        //}
        i++;
    }
    for(int i = 0 ; i < RANDOM_PARTICLES; i++) {
        auto p = sampleRandom();
        particles.push_back(p);
    }
}

// calculate position from current particle cloud
V3 Localization::calculatePositionFromParticles(float roll, float pitch, float height) {
    int count = PARTICLE_COUNT*0.5f;
    V3 sum = (V3){0.0f, 0.0f, 0.0f};
    V2 dir = (V2){0.0f, 0.0f};
    float weightSum = 0.0f;
    for(int i = 0; i < count; i++) {
        float w = particles[i].weight;
        sum.x += particles[i].pos.x*w;
        sum.y += particles[i].pos.y*w;
        
        V2 cdir = v2_rotate((V2){1.0f, 0.0f}, particles[i].heading*TT_DEG2RAD_F);
        v2_add(&dir, dir, v2_scale(cdir, w));

        weightSum += w;
    }
    sum.x /= (float)weightSum;
    sum.y /= (float)weightSum;

    dir.x /= (float)weightSum;
    dir.y /= (float)weightSum;
    sum.z = atan2f(dir.y, dir.x)*TT_RAD2DEG_F;

    // check adjacent track nodes for the 
    float min = FLT_MAX;
    int minIdx = floorf(locT);
    float minLocT = locT;
    for(int i = -3; i <= 3; i++) {
        int startNode = clampMapIndex(floorf(locT) + i);
        int endNode = clampMapIndex(floorf(locT) + i + 1);
        V2 start = (V2){map[startNode].pos.x, map[startNode].pos.y};
        V2 end = (V2){map[endNode].pos.x, map[endNode].pos.y};
        V2 cp = closest_point_on_line_seg((V2){sum.x, sum.y}, start, end);
        float t = closest_point_on_line_seg_t((V2){sum.x, sum.y}, start, end);
        float xdif = cp.x - sum.x;
        float ydif = cp.y - sum.y;
        float d2 = xdif*xdif + ydif*ydif;
        if(d2 < min) {
            min = d2;
            minIdx = startNode;
            minLocT = startNode + t;
        }
    }
    locT = minLocT;

    // TODO: determine direction
    if(!forwardDetermined) {
        // compare top half only
        count = PARTICLE_COUNT/2;
        int agree = 0;
        for(int i = 0; i < count; i++) {
            float h = particles[i].heading;
            h = fmodf(h, 360.0f);
            if(h > 180.0f) {
                h = -(180.0f - (h - 180.0f));
            }
            if(SIGNF(sum.z) == SIGNF(h)) {
                agree++;
            }
        }
        // 80% particles agree with direction
        if((float)agree/(float)count > 0.9f) {
            forwardT = SIGNF(sum.z);
            forwardDetermined = true;
        }
    }

    // calculate world to screen matrix
    Mat4 worldToCam;
    Quat rot, rollQ, pitchQ, yawQ;
    quat_angle_axis(&rollQ, -roll, ROLL_AXIS);
    quat_angle_axis(&pitchQ, -pitch, PITCH_AXIS);
    quat_angle_axis(&yawQ, sum.z, (V3){0.0f, 0.0f, -1.0f});
    quat_mul(&rot, yawQ, rollQ);
    quat_mul(&rollQ, rot, pitchQ);
    rot = rollQ;
    Quat totalRot;
    V3 pos = v3_scale((V3){sum.x, sum.y, height}, -1.0f);
    quat_mul(&totalRot, this->camLocalRot, rot);
    totalRot.w *= -1.0f;
    mat4_rt(&worldToCam, totalRot, pos);
    Mat3 camToScreen2 = this->camToScreen;
    camToScreen2.m11 *= -1.0f;
    mat4_cv_camera(&this->worldToScreen, &camToScreen2, &worldToCam);

    return sum;
}

// get closest map edge given position
int Localization::getClosestMapEdge(V2 pos, V2& cp) {
    int x = CLAMP((int)(pos.x/2.0f), 0, LOOKUP_TABLE_WIDTH);
    int y = CLAMP((int)(pos.y/2.0f), 0, LOOKUP_TABLE_HEIGHT);
    int idx = proximityLookup[y][x];
    int next = map[idx].next;
    V2 start = (V2){map[idx].pos.x, map[idx].pos.y};
    V2 end = (V2){map[next].pos.x, map[next].pos.y};
    cp = closest_point_on_line_seg(pos, start, end);
    return idx;
}

// very inefficient approximation because 't' in bezier curves doesn't have uniform 'density'
// TODO: very buggy!
V2 Localization::getPositionForwardCm(float cmf) {
    V2 start = getPositionOnMap(0.0f);
    float step = 0.0001f;
    float t = step;
    float dist = 0.0f;
    V2 cur, dif;
    while(dist < cmf) {
        cur = getPositionOnMap(t);
        v2_sub(&dif, cur, start);
        dist += v2_len(dif);
        t += step;
        start = cur;
    }
    return cur;
}

// get map direction given advance (advance is for turning ahead)
V2 Localization::getMapDirection(float advance) {
    V2 d;
    if(!forwardDetermined) {
        //return smoothMap.Derivative(locT);
        return (V2){0.0f, 0.0f};
    }
    d = smoothMap.Derivative(locT + forwardT*advance);
    d.x *= forwardT;
    d.y *= forwardT;
    v2_normalize(&d);
    return d;
}

// get velocity multiplier given advance (advance is for breaking/accelerating ahead)
float Localization::getVelocityMultiplier(float advance) {
    float t = locT; 
    if(forwardDetermined) {
        t = t + forwardT*advance;
    }
    // 46.1 -> 46.1
    // 47.1 ->  0.1
    t = fmodf(t, (float)MAP_NODE_COUNT);
    // -1.1 -> 45.9
    // -0.5 -> 46.5
    t = (t >= 0.0f) ? t : (float)MAP_NODE_COUNT+t;
    int start = (int)floorf(t);
    int end = (start + 1) % MAP_NODE_COUNT;
    float vs = velMults[start];
    float ve = velMults[end];
    // lerp
    float ret = vs + (ve-vs)*fmodf(t, 1.0f);
    return ret;
}

V2 Localization::getPositionOnMap(float advance) {
    return smoothMap.Evaluate(locT + forwardT*advance);
}

// update particle filter
void Localization::update(float height, float roll, float pitch, float yaw, V2 hMovement, float dt, std::vector<ContourCenter>& cs) {
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    float max = 0.0f;
    int maxi = 0;
    for(int i = 0; i < prevParticles.size(); i++) {
        prevParticles[i] = updateParticle(0.0f, hMovement, dt, prevParticles[i]);
        float w = compareParticleToMeasurement(prevParticles[i], height, pitch, roll, cs, NULL);
        prevParticles[i].weight = w;
        if(w > max) {
            max = w;
            maxi = i;
        }
    }

    //cv::Mat image = cv::Mat::zeros(154, 205, CV_8UC3);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::duration<double> duration = end-start;
	printf("particle update duration %fms\r\n", duration.count()*1000.0);
    // resample
	start = std::chrono::steady_clock::now();
    resample(prevParticles);
	end = std::chrono::steady_clock::now();
	duration = end-start;
	printf("particle resample duration %fms\r\n", duration.count()*1000.0);

    V3 pos = calculatePositionFromParticles(roll, pitch, height);
    this->lastEstimation = pos;

    this->prevParticles = particles;
}

// draw particle filter (for debugging)
void Localization::drawParticles(std::vector<ContourCenter>& cs, float lookahead) {
    float scale = 0.25f;
    image = cv::Mat::zeros(125, 250, CV_8UC3);
    for(int i = 0; i < MAP_NODE_COUNT; i++) {
        /*if(i == 24) {
        cv::circle(image, cv::Point((int)map[i].pos.x*scale, (int)map[i].pos.y*scale), 3, cv::Scalar(0,0,255), -1);
        } else {*/
        cv::circle(image, cv::Point((int)map[i].pos.x*scale, (int)map[i].pos.y*scale), 3, cv::Scalar(255,255,255), -1);
        //}
        /*for(int j = 0; j < 10; j++) {
            float t0 = i + j*0.1f;
            float t1 = i + (j+1)*0.1f;
            V2 startPoint = smoothMap.Evaluate(t0);
            V2 endPoint = smoothMap.Evaluate(t1);
            //V2 startPoint;
            //V2 endPoint;
            cv::line(image, cv::Point((int)startPoint.x*scale, (int)startPoint.y*scale), cv::Point((int)endPoint.x*scale, (int)endPoint.y*scale), cv::Scalar(100, 255, 255));
        }*/
        //cv::line(image, cv::Point((int)map[i].pos.x*scale, (int)map[i].pos.y*scale), cv::Point((int)map[i].pos.x*scale+map[i].dir.x*10, (int)map[i].pos.y*scale+map[i].dir.y*10), cv::Scalar(100,255,255));
        // map node connection line
        //cv::line(image, cv::Point((int)map[i].pos.x*scale, (int)map[i].pos.y*scale), cv::Point((int)map[(i+1)%MAP_NODE_COUNT].pos.x*scale, (int)map[(i+1)%MAP_NODE_COUNT].pos.y*scale), cv::Scalar(252, 0, 173)); 
    }
    for(int i = 0; i < prevParticles.size(); i+=4) {
        auto p = prevParticles[i];
        V2 forward = (V2){1.0f, 0.0f};
        float cosine = cosf(TT_DEG2RAD_F*p.heading);
        float sine = sinf(TT_DEG2RAD_F*p.heading);
        float x = cosine*forward.x - sine*forward.y;
        float y = sine*forward.x + cosine*forward.y;
        forward.x = x;
        forward.y = y;
        auto col = cv::Scalar(0, (int)(CLAMP(p.weight*255.0f, 0.0f, 255.0f)));
        cv::circle(image, cv::Point((int)(p.pos.x*scale), (int)(p.pos.y*scale)), 1, col, 0, -1);
        // direction line
        //cv::line(image, cv::Point((int)(p.pos.x*scale), (int)(p.pos.y*scale)), cv::Point((int)(p.pos.x+forward.x*8.0f)*scale, (int)(p.pos.y+forward.y*8.0f)*scale), col, 1);
    }
    // estimated position
    cv::circle(image, cv::Point((int)(this->lastEstimation.x*scale), (int)(this->lastEstimation.y*scale)), 3, cv::Scalar(0, 0, 255), -1);
    // lookahead point
    V2 lh = getPositionOnMap(lookahead);
    cv::circle(image, cv::Point((int)(lh.x*scale), (int)(lh.y*scale)), 3, cv::Scalar(255, 43, 88), -1);
    // estimated direction
    float ecosine = cosf(TT_DEG2RAD_F*this->lastEstimation.z);
    float esine = sinf(TT_DEG2RAD_F*this->lastEstimation.z);
    float ex = ecosine*1.0f;
    float ey = esine*1.0f;
    cv::line(image, cv::Point((int)(this->lastEstimation.x*scale), (int)(this->lastEstimation.y*scale)), cv::Point((int)(this->lastEstimation.x+ex*10.0f)*scale, (int)(this->lastEstimation.y+ey*10.0f)*scale), cv::Scalar(0, 0, 255), 1);

    V2 mpos = smoothMap.Evaluate(locT);
    cv::circle(image, cv::Point((int)mpos.x*scale, (int)mpos.y*scale), 3, cv::Scalar(122, 0, 255), -1);

/*    LocParticle p;
    p.pos.x = 120.0f;
    p.pos.y = 250.0f;
    p.heading = 315.0f;
    compareParticleToMeasurement(p, 175.0f, 0.0f, 0.0f, (V2){100.0f, 100.0f}, cs, &image);
    compareParticleToMeasurement(bp, 175.0f, 0.0f, 0.0f, (V2){100.0f, 100.0f}, cs, &image);*/
}
