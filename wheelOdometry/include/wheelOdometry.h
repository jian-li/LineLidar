#ifndef _WHEELODOMETRY_H
#define _WHEELODOMETRY_H

#include <stdint.h>

namespace CleanBot
{

#define CTRLPERIOD 0.01     // low-level control period of bot, 10ms
#define WHEELCF 0.226            // circumference of wheel, 226mm
#define WHEELBASELINE 0.23      // base line of 2 wheels


class wheelOdometry
{
public:
    wheelOdometry();
    ~wheelOdometry();
    
    void SetPosXY(double px, double py);
    void SetAngle(double th);
    
    // position and angle
    double mPosX;
    double mPosY;
    double mTheta;
    // velocity
    double mVelX;
    double mVelY;
    double mVelTheta;
    
    // first message flag
    bool mbStartflag;
    
    // message data
    uint8_t mprePZ;
    uint16_t mpreRC,mpreLC,mpreTC;
    int16_t mpreLD,mpreRD;
    
    // wheel parameters
    double mParamBotLLPeriod;   // low-level control period of bot, 10ms
    double mParamBotWheelBsl;   // base line of 2 wheels, 230mm
    double mParamInvBotWheelBsl;    // 1/mParamBotWheelBsl
    double mParamEncStepDis;	// distance of an encoder step
    //double mParamBotWheelCf;    // circumference of wheel, 226mm
    //uint16_t mParamBotEncRoundCnt;  // encoder count of a round
};


}

#endif
