#ifndef FWDKM_H
#define FWDKM_H

#define JNT_NUMBER 3
#define CART_COORD_DIM 3
#define JA0_L -M_PI/2
#define JA0_U M_PI/2
#define JA1_L 0.0
#define JA1_U 120.0/180.0*M_PI
#define JA2_L -M_PI/2
#define JA2_U 0.0

// enum _Axis {
// 	none = 0,
// 	xAxis = 1,
// 	yAxis = 2,
// 	zAxis = 3
// } Axis;
typedef struct _threeDOFs {
	double l1, l2, l3;	// lengths of the three links
	double baseHeight;
	double a1, a2, a3, a4;	// four angles between links, where a2 is fixed
} threeDOFs;


threeDOFs* init(double linkLength[JNT_NUMBER], double baseHeight,
			double initJntAngles[JNT_NUMBER]);

double* getPoseByJnts(double jntArray[JNT_NUMBER]);

#endif