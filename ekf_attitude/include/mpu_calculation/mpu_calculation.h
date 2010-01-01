#include <iostream>

//定义数据类型
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;

#define UserGyrResolution ((float) 0.007633587786f)
#define MPU9250A_4g ((float)0.000122070312f)
//#define Gravity_g 10
#define Byte16(Type, ByteH, ByteL) (Type)(((uint16)(ByteH)<<8)|((uint16)(ByteL)))

extern float MagTrueX,MagTrueY,MagTrueZ;
