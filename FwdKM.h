#ifndef FWDKM_H
#define FWDKM_H

#ifdef __cplusplus
extern "C" {
#endif

#define JNT_NUMBER 3
#define CART_COORD_DIM 3
#define POSE_FRAME_DIM 6
#define POSS_NUMBER 4

#ifndef M_PI
	#define M_PI 3.14159265359
#endif

#define JNT0_L -M_PI/2
#define JNT0_U M_PI/2
#define JNT1_L 0.0
#define JNT1_U 130.0/180.0*M_PI
#define JNT2_L -M_PI/2
#define JNT2_U 0.0

#define MAGNET_EE_OFFSET 2.1

#define JNT_ANGLES_OUT_OF_BOUND -100

// enum _Axis {
// 	none = 0,
// 	xAxis = 1,
// 	yAxis = 2,
// 	zAxis = 3
// } Axis;
typedef struct _threeDOFsFwd {
	double l1, l2, l3;	// lengths of the three links
	double baseHeight;
	double a1, a2, a3, a4;	// four angles between links, where a2 is fixed
} threeDOFsFwd;


int initFwdKM();

int getJntPosByAngle(const double jntArray[JNT_NUMBER],
						double allPoss[JNT_NUMBER][CART_COORD_DIM], int numOfPoss);

int getEEPoseByJnts(const double jntArray[JNT_NUMBER], double eePos[POSE_FRAME_DIM]);

int getAllPossByJnts(const double jntArray[JNT_NUMBER], double allPoss[4][POSE_FRAME_DIM]);

int finishFwdKM();

#ifdef __cplusplus
}
#endif

#endif
