class HeightController {
public:
    HeightController() {}
    // height from ground in cm
    virtual void heightUpdate(float newheight, double timestamp) = 0;
    // acceleration upwards (projected up, not just Z component of accelerometer!)
    // in cm/s^2 where towards ground is positive
    //virtual void accelerationUpdate(float newacc, double timestamp) = 0;
    virtual void velocityUpdate(float newvel, double timestamp) = 0;
    virtual void setTarget(float target) = 0;
    // controller iteration, same rate or slower as measurements!
    virtual void update(double timestamp) = 0;
    virtual void reset() = 0;
    // current throttle from 0..1
    virtual float getThrottle() = 0;
};

// third order height controller
class TOHeightController : public HeightController {
public:
    TOHeightController();
    virtual void heightUpdate(float newheight, double timestamp);
    virtual void accelerationUpdate(float newacc, double timestamp);
    virtual void velocityUpdate(float newvel, double timestamp);
    virtual void setTarget(float target);
    virtual void update(double timestamp);
    virtual float getThrottle();
    virtual void reset();
private:
    float targetHeight;
    float lastHeight;
    double lastUpdateTimestamp;
    float speed;
    float acceleration;
    float curThrottle;
};

// PID height controller
class PIDHeightController : public HeightController {
public:
    PIDHeightController();
    virtual void heightUpdate(float newheight, double timestamp);
    virtual void accelerationUpdate(float newacc, double timestamp);
    virtual void velocityUpdate(float newvel, double timestamp);
    virtual void setTarget(float target);
    virtual void update(double timestamp);
    virtual float getThrottle();
    virtual void reset();
private:
    double prevUpdateTime;
    float value;
    float setPoint;
    float prevError;
    float integral;
    float output;
    float velocity;

    const float KP = 0.001f;
    const float KI = 0.0001f;
    const float KD = 0.0025f;
    /*const float KP = 0.003f;
    const float KI = 0.0025f;
    const float KD = 0.0025f;*/
};

// LQR height controller (basically LQR tuned PID)
class LQRHeightController : public HeightController {
public:
    LQRHeightController();
    virtual void heightUpdate(float newheight, double timestamp);
    virtual void accelerationUpdate(float newacc, double timestamp);
    virtual void velocityUpdate(float newacc, double timestamp);
    virtual void setTarget(float target);
    virtual void update(double timestamp);
    virtual float getThrottle();
    virtual void reset();
private:
    float velocity; // in cm/s
    float height; // in cm
    float setPoint; // in cm
    float acc; // cm/s^2
    float output; // 0..1
    float integral; 
    double prevTimeStamp; // in seconds

    // from matlab
    const float K1 = 2.8142f;
    const float K2 = 3.4599f;
    const float K3 = 1.0f;

    const float THROTTLE_GAIN = 2.5f;
    const float MAX_SPEED = 100.0f; // cm/s
};

class CPIDHeightController : public HeightController {
public:
    CPIDHeightController();
    virtual void heightUpdate(float newheight, double timestamp);
    virtual void accelerationUpdate(float newacc, double dimestamp);
    virtual void velocityUpdate(float newvel,  double timestamp);
    virtual void setTarget(float target);
    virtual void update(double timestamp);
    virtual float getThrottle();
    virtual void reset();
private:

    const float KPH = 0.60f;

    const float KPV = 0.0020f;
    const float KIV = 0.00020f;
    const float START_THROTTLE = 0.33f;

    float height;
    float targetHeight;    
    float velocity;
    float velocityIntegral;
    //float acceleration;
    double prevTimeStamp;

    float output;
};
