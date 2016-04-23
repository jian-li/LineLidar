#ifndef __PIXHAWK_EKF_H
#define __PIXHAWK_EKF_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <mpu_calculation.h>
#include <iostream>


// Global variables
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define pi 3.141592657f
#define earthRate 0.000072921f
#define earthRadius 6378145.0f
#define earthRadiusInv  1.5678540e-7f

/* Local gravitational acceleration
 * www.wolframalpha.com   Search gravitational acceleration BeiJing
 * total field : 9.80267 m/s2
 * down : 9.80261
 * west : 3.6*10e-4
 * south: 0.03239
 */
#define GRAVITY_MSS 9.80267f

/*Local gps and local Mag_declination
 *
 *www.magnetic-declination.com
 *Beijing:
 *Lat:39du54'27''N = 39.9075
 *Lon:116du23'50''E = 116.39722
 *Magnetic declination:-6du46' =-6.76666
 *Inclination:58du59' = 58.98333
 *Magnetic field strength:54561.4 nT
 *
*/
#define freq 200  //ahrs freq
#define Local_Lat 39.9075
#define Local_Lon 116.39722
#define Local_MagDeclination -6.76666




void Global_Initialise(float roll,float pitch,float yaw);
void pixhawk_ekf(double gyrx,double gyry,double gyrz,double accx,double accy,double accz);
extern bool newDataMag;

#endif
