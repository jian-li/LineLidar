#include "wheelOdometry.h"

namespace CleanBot
{

wheelOdometry::wheelOdometry() :
  mPosX(0.0), mPosY(0.0), mTheta(0.0), mbStartflag(true),
  mVelX(0.0), mVelY(0.0), mVelTheta(0.0), 
  mprePZ(0), mpreRC(0), mpreLC(0), mpreTC(0), mpreLD(0), mpreRD(0)
{
    // can get from config files
    mParamBotLLPeriod = 0.01;
    //mParamBotEncRoundCnt = 1254;
    //mParamBotWheelCf = 0.230;	//0.226
    
    //only need to adjust 0.18341
    mParamEncStepDis = 0.18341*0.001;//0.18341*0.001; 
    
    mParamBotWheelBsl = 1311.5*mParamEncStepDis; //0.2399;	//0.23 //0.24053353
    mParamInvBotWheelBsl = 1.0/mParamBotWheelBsl;
}

wheelOdometry::~wheelOdometry()
{
    
}


void wheelOdometry::SetPosXY(double px, double py)
{
    mPosX = px;
    mPosY = py;
}

void wheelOdometry::SetAngle(double th)
{
    mTheta = th;
}


}
