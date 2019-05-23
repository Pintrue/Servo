#include "FwdKM.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TO_DECIMAL_PLACE(v, n) (roundf(v * pow(10, n)) / pow(10, n))


static threeDOFsFwd* arm;


int initFwdKM() {
	/* DESCRIPTION of the arm */
	double linkLength[3] = {5.9908, 10.7575, 18.7299};
	double initJntAngles[3] = {0.0, atan2(2.0, 10.57), atan2(3.5, 18.4)};
	double baseHeight = 4.20;


	arm = (threeDOFsFwd*) malloc(sizeof(threeDOFsFwd));

	arm->l1 = linkLength[0];
	arm->l2 = linkLength[1];
	arm->l3 = linkLength[2];

	arm->baseHeight = baseHeight;

	arm->a1 = initJntAngles[0];
	arm->a2 = 0.58337;
	arm->a3 = initJntAngles[1];
	arm->a4 = initJntAngles[2];

	return 0;
}


int getEEPoseByJnts(const double jntArray[JNT_NUMBER], double eePos[POSE_FRAME_DIM]) {
	/**
	 * Calculate y from a side view, where y has the following eqaution
	 * 
	 * 					y = d2 + d3 + d4 + d5
	 **/


	if (jntArray[0] > JNT0_U || jntArray[0] < JNT0_L ||
		jntArray[1] > JNT1_U || jntArray[1] < JNT1_L ||
		jntArray[2] > JNT2_U || jntArray[2] < JNT2_L) {
		
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	arm->a1 += jntArray[0];
	arm->a3 += jntArray[1];
	arm->a4 -= jntArray[2] + jntArray[1];

	// double a1 = arm->a1 + jntArray[0];
	// double a3 = arm->a3 + jntArray[1];
	// double a4 = arm->a4 - jntArray[2] - jntArray[1];

	// arm->a1 = a1;
	// arm->a3 = a3;
	// arm->a4 = a4;

	double d2 = arm->baseHeight;
	double d3 = arm->l1 * sin(arm->a2);
	double d4 = arm->l2 * sin(arm->a3);
	double d5 = arm->l3 * sin(arm->a4);

	double y = d2 + d3 + d4 + d5;

	/**
	 * Calculate x and z from a top view, where x has the following eqaution
	 * 
	 * 					z = d1 * cos(a1)
	 * and where z has the following equation
	 * 
	 * 					x = d1 * sin(a1)
	 * and d1 has the following equation
	 * 
	 * 					d1 = d6 - d7 + d8
	 **/

	double d6 = arm->l3 * cos(arm->a4);
	double d7 = arm->l2 * cos(arm->a3);
	double d8 = arm->l1 * cos(arm->a2);

	double d1 = d6 - d7 + d8;

	double z = d1 * cos(arm->a1);
	double x = d1 * sin(arm->a1);

	eePos[0] = TO_DECIMAL_PLACE(x, 2); eePos[1] = TO_DECIMAL_PLACE(y, 2); eePos[2] = TO_DECIMAL_PLACE(z, 2);
	eePos[3] = jntArray[1] + jntArray[2];
	eePos[4] = jntArray[0];
	eePos[5] = 0;

	return 0;
}


int finish() {
	free(arm);
	return 0;
}


int main() {
	initFwdKM();

	double delta[3] = {-1.732664e-01 ,1.745329e-01 ,-1.282639e-02};
	double eePos[POSE_FRAME_DIM];
	int res = getEEPoseByJnts(delta, eePos);
	if (res < 0) {
		printf("Angle out of bound.\n");
	} else {
		printf("[ ");
		for (int i = 0; i < 6; ++i) {
			printf("%f ", eePos[i]);
		}
		printf("]\n");
	}
	return 0;
}