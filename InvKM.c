#include "InvKM.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static threeDOFsInv* arm;


int initInvKM() {
	/* DESCRIPTION of the arm */
	double linkLength[3] = {5.9908, 10.7575, 18.7299};
	double initJntAngles[3] = {0.0, atan2(2.0, 10.57), atan2(3.5, 18.4)};
	double baseHeight = 4.20;


	arm = (threeDOFsInv*) malloc(sizeof(threeDOFsInv));

	arm->l1 = linkLength[0];
	arm->l2 = linkLength[1];
	arm->l3 = linkLength[2];

	arm->initA1 = initJntAngles[0];
	arm->initA2 = 0.58337;
	arm->initA3 = initJntAngles[1];
	arm->initA4 = initJntAngles[2];

	return 0;
}


int getJntsByEEPos(const double eePos[POSE_FRAME_DIM], double jntArray[JNT_NUMBER]) {
	double d1 = atan2(eePos[0], eePos[2]);
	// printf("d1 is equal to %f\n", d1);

	// /* calculate the coordinate of shoulder arm */
	// double shoulderCoord[CART_COORD_DIM];
	// /* in side-view */
	// double sideX = arm->l1 * cos(arm->initA2); // 2D x-position
	// double shoulderY = arm->l1 * sin(arm->initA2) + arm->baseHeight;	// 2D y-position

	// double shoulderX = sideX * sin(d1);
	// double shoulderZ = sideX * cos(d1);

	// shoulderCoord[0] = shoulderX; shoulderCoord[1] = shoulderY; shoulderCoord[2] = shoulderZ;

	initFwdKM();
	double angle[3] = {0, 0, 0};
	double jntPoss[2][3];
	getJntPosByAngle(angle, jntPoss, 1);
	finish();

	double shoulderSideX = sqrt(pow(jntPoss[0][0], 2) + pow(jntPoss[0][2], 2));
	double shoulderY = jntPoss[0][1];
	double eeSideX = sqrt(pow(eePos[0], 2) + pow(eePos[2], 2));

	double dShoulderEEX = eeSideX - shoulderSideX;
	double dShoulderEEY = eePos[1] - shoulderY;
	double dShoulderEEAngle = atan2(dShoulderEEY, dShoulderEEX);

	double l4 = sqrt(pow(dShoulderEEX , 2) + pow(dShoulderEEY, 2));
	/* use law of cosine to solve the triangle formed by shoulder, forearm joints and EE */
	double loC3 = acos((pow(arm->l2, 2) + pow(l4, 2) - pow(arm->l3, 2)) / (2 * arm->l2 * l4));
	double loC4 = acos((pow(arm->l2, 2) + pow(arm->l3, 2) - pow(l4, 2)) / (2 * arm->l2 * arm->l3));

	double d2 = M_PI - dShoulderEEAngle - loC3 - arm->initA3;
	double d3 = arm->initA3 + arm->initA4 - loC4;

	printf("[ ");
	printf("%f ", d1);
	printf("%f ", d2);
	printf("%f ", d3);
	printf("]\n");

	if (d1 > JNT0_U || d1 < JNT0_L ||
		d2 > JNT1_U || d2 < JNT1_L ||
		d3 > JNT2_U || d3 < JNT2_L) {
		return JNT_ANGLES_OUT_OF_BOUND;
	}

	jntArray[0] = d1;
	jntArray[1] = d2;
	jntArray[2] = d3;
	return 0;
}


int finishInvKM() {
	free(arm);
	return 0;
}


int main() {
	initInvKM();

	double eePos[POSE_FRAME_DIM] = {8.605749e+00, 0.000000e+00, 2.05e+01};
	// double eePos[POSE_FRAME_DIM] = {-3.563496e+00, 5.000000e-01, 1.765392e+01};
	// double eePos[POSE_FRAME_DIM] = {-2.360000, 11.800000, 13.460000};
	double jntArray[JNT_NUMBER];

	if (getJntsByEEPos(eePos, jntArray) >= 0) {
		printf("[ ");
		for (int i = 0; i < 3; ++i) {
			printf("%f ", jntArray[i]);
		}
		printf("]\n");
	} else {
		printf("EE Pos not in eligible range.\n");
	}

	// initFwdKM();
	// double angle[3] = {0, 0, 0};
	// double allPoss[2][3];
	// getJntPosByAngle(angle, allPoss, 2);
	// for (int i = 0; i < 2; ++i) {
	// 	printf("[ ");
	// 	for (int j = 0; j < 3; ++j) {
	// 		printf("%f ", allPoss[i][j]);
	// 	}
	// 	printf("]\n");
	// }
}