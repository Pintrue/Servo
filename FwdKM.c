#include "FwdKM.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TO_DECIMAL_PLACE(v, n) (roundf(v * pow(10, n)) / pow(10, n))


static threeDOFsFwd* arm;


int initFwdKM() {
	/* DESCRIPTION of the arm */
	// double linkLength[3] = {5.9908, 10.7575, 18.7299};
	// double initJntAngles[3] = {0.0, atan2(2.0, 10.57), atan2(3.5, 18.4)};
	// double baseHeight = 4.20;

	double linkLength[3] = {sqrt(3.5*3.5+3.9*3.9), sqrt(1.7*1.7+10.5*10.5), sqrt(3.5*3.5+16.5*16.5)};
	double initJntAngles[3] = {0.0, atan2(1.7, 10.50), atan2(3.5, 16.5)};
	double baseHeight = 2.9;


	arm = (threeDOFsFwd*) malloc(sizeof(threeDOFsFwd));

	arm->l1 = linkLength[0];
	arm->l2 = linkLength[1];
	arm->l3 = linkLength[2];

	arm->baseHeight = baseHeight;

	arm->a1 = initJntAngles[0];
	// arm->a2 = 0.58337;
	arm->a2 = atan2(3.5, 3.9);
	arm->a3 = initJntAngles[1];
	arm->a4 = initJntAngles[2];

	return 0;
}

/**
 * numOfPoss > 1 -> calculate and return both the coordinates 
 * of shoulder and forearm joints;
 * 
 * otherwise, just the shoulder joint coordinate.
 */
int getJntPosByAngle(const double jntArray[JNT_NUMBER],
						double allPoss[2][CART_COORD_DIM], int numOfPoss) {
	if (jntArray[0] > JNT0_U || jntArray[0] < JNT0_L ||
		jntArray[1] > JNT1_U || jntArray[1] < JNT1_L ||
		jntArray[2] > JNT2_U || jntArray[2] < JNT2_L) {
		
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	arm->a1 += jntArray[0];
	arm->a3 += jntArray[1];
	arm->a4 -= jntArray[2] + jntArray[1];

	double shoulderCoord[CART_COORD_DIM], forearmCoord[CART_COORD_DIM];

	/* calculate the coordinate of shoulder arm joint */

	/* side-view */
	double shoulderSideX = arm->l1 * cos(arm->a2); // 2D x-position
	double shoulderY = arm->l1 * sin(arm->a2) + arm->baseHeight;	// 2D y-position

	/* top-view */
	double shoulderX = shoulderSideX * sin(arm->a1);
	double shoulderZ = shoulderSideX * cos(arm->a1);

	shoulderCoord[0] = shoulderX; shoulderCoord[1] = shoulderY; shoulderCoord[2] = shoulderZ;
	for (int i = 0; i < CART_COORD_DIM; ++i) {
		allPoss[0][i] = shoulderCoord[i];
	}

	--numOfPoss;

	if (numOfPoss > 0) {
		/* calculate the coordinate of forearm joint */
	
		/* side-view */
		double forearmSideX = arm->l2 * cos(arm->a3) - shoulderSideX;
		double forearmY = arm->l2 * sin(arm->a3) + shoulderY;

		/* top-view */
		double forearmX = forearmSideX * sin(M_PI + arm->a1);
		double forearmZ = forearmSideX * cos(M_PI + arm->a1);

		forearmCoord[0] = forearmX; forearmCoord[1] = forearmY; forearmCoord[2] = forearmZ;
		for (int i = 0; i < CART_COORD_DIM; ++i) {
		allPoss[1][i] = forearmCoord[i];
		}
	}

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

	double d1 = d6 - d7 + d8 + MAGNET_EE_OFFSET;

	double z = d1 * cos(arm->a1);
	double x = d1 * sin(arm->a1);

	eePos[0] = TO_DECIMAL_PLACE(x, 2); eePos[1] = TO_DECIMAL_PLACE(y, 2); eePos[2] = TO_DECIMAL_PLACE(z, 2);
	eePos[3] = jntArray[1] + jntArray[2];
	eePos[4] = jntArray[0];
	eePos[5] = 0;

	return 0;
}


int getAllPossByJnts(const double jntArray[JNT_NUMBER], double allPoss[POSS_NUMBER][POSE_FRAME_DIM]) {
	/* set the pose for neck joint */
	allPoss[0][0] = 0; allPoss[0][1] = arm->baseHeight; allPoss[0][2] = 0;
	allPoss[0][3] = 0; allPoss[0][4] = jntArray[0]; allPoss[0][5] = 0;

	int check;
	initFwdKM();
	double poss[2][CART_COORD_DIM];
	check = getJntPosByAngle(jntArray, poss, 2);
	if (check < 0) {
		printf("Joint angle sent to getAllPossByJnts() is out of bound.\n");
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < CART_COORD_DIM; ++j) {
			allPoss[i + 1][j] = poss[i][j];
		}
	}

	allPoss[1][4] = jntArray[0]; allPoss[2][4] = jntArray[0];
	allPoss[2][3] = jntArray[1];

	initFwdKM();
	double eePos[POSE_FRAME_DIM];
	check = getEEPoseByJnts(jntArray, eePos);
	if (check < 0) {
		// shouldnt get here
		printf("Joint angle sent to getAllPossByJnts() is out of bound.\n");
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	for (int i = 0; i < POSE_FRAME_DIM; ++i) {
		allPoss[3][i] = eePos[i];
	}

	return 0;
}


int finishFwdKM() {
	free(arm);
	return 0;
}


threeDOFsFwd* getCache() {
	return arm;
}


// int main() {

// 	// double delta[3] = {-1.732664e-01 ,1.745329e-01 ,-1.282639e-02};
// 	double delta[3] = {0.5, 0.0, -0.7};

// 	double allPoss[4][6];
// 	double eePos[POSE_FRAME_DIM];

// 	initFwdKM();
// 	int res = getAllPossByJnts(delta, allPoss);
// 	if (res < 0) {
// 		printf("Angle out of bound.\n");
// 	} else {
// 		// printf("[ ");
// 		for (int i = 0; i < 4; ++i) {
// 			printf("[ ");
// 			for (int j = 0; j < 6; ++j) {
// 				printf("%f ", allPoss[i][j]);
// 			}
// 			printf("]\n");
// 		}
// 	}
// 	finishFwdKM();


// 	// initFwdKM();
// 	// int res = getEEPoseByJnts(delta, eePos);
// 	// if (res < 0) {
// 	// 	printf("Angle out of bound.\n");
// 	// } else {
// 	// 	printf("[ ");
// 	// 	for (int i = 0; i < 6; ++i) {
// 	// 		printf("%f ", eePos[i]);
// 	// 	}
// 	// 	printf("]\n");
// 	// }

// 	// return 0;
// }
